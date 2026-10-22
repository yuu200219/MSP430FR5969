/*-----------------------------------------------------------------------*/
/* Low level disk I/O module for FatFs on MSP430FR5969   (C)ChaN, 2025   */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Basic definitions of FatFs */
#include "diskio.h"		/* Declarations FatFs MAI */
#include "driverlib.h"
#include <msp430.h>

/* MMC/SD card type flags (CardType) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

/* MMC card commands */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */

/* Physical drive number */
#define DEV_MMC		0

/* SPI pins for SD card (EUSCI_A1) */
#define SD_CS_PORT  GPIO_PORT_P1
#define SD_CS_PIN   GPIO_PIN4

static volatile DSTATUS Stat = STA_NOINIT;	/* Physical drive status */
static BYTE CardType;			/* Card type flags */


/*-----------------------------------------------------------------------*/
/* SPI Controls (Platform dependent)                                     */
/*-----------------------------------------------------------------------*/
#define SPI_TIMEOUT 10000
static void xmit_spi (BYTE dat)
{
    uint32_t timeout = SPI_TIMEOUT;
    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, dat);
    while (EUSCI_A_SPI_isBusy(EUSCI_A1_BASE)) {
        if (--timeout == 0) {
            return; 
        }
    }
    EUSCI_A_SPI_receiveData(EUSCI_A1_BASE); // Dummy read to clear IFG
}

static BYTE rcvr_spi (void)
{
    uint32_t timeout = SPI_TIMEOUT;
    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, 0xFF);
    while (EUSCI_A_SPI_isBusy(EUSCI_A1_BASE)) {
        if (--timeout == 0) {
            return 0xFF; 
        }
    }
    return EUSCI_A_SPI_receiveData(EUSCI_A1_BASE);
}

static void rcvr_spi_multi (BYTE *buff, UINT btr)
{
    do {
        EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, 0xFF);
        while (EUSCI_A_SPI_isBusy(EUSCI_A1_BASE));
        *buff++ = EUSCI_A_SPI_receiveData(EUSCI_A1_BASE);
    } while (--btr);
}

#if FF_FS_READONLY == 0
static void xmit_spi_multi (const BYTE *buff, UINT btx)
{
    do {
        EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, *buff++);
        while (EUSCI_A_SPI_isBusy(EUSCI_A1_BASE));
        EUSCI_A_SPI_receiveData(EUSCI_A1_BASE);
    } while (--btx);
}
#endif


/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static int wait_ready (UINT wt)
{
    BYTE d;

    do {
        d = rcvr_spi();
    } while (d != 0xFF && --wt);

    return (d == 0xFF) ? 1 : 0;
}


/*-----------------------------------------------------------------------*/
/* Deselect card and release SPI                                         */
/*-----------------------------------------------------------------------*/

static void deselect (void)
{
    GPIO_setOutputHighOnPin(SD_CS_PORT, SD_CS_PIN);
    rcvr_spi();	/* Dummy clock (force DO hi-z for some cards) */
}


/*-----------------------------------------------------------------------*/
/* Select card and wait for ready                                        */
/*-----------------------------------------------------------------------*/

static int select (void)
{
    GPIO_setOutputLowOnPin(SD_CS_PORT, SD_CS_PIN);
    rcvr_spi();	/* Dummy clock (force DO enabled) */

    if (wait_ready(500)) return 1;	/* OK */

    deselect();
    return 0;	/* Failed */
}


/*-----------------------------------------------------------------------*/
/* Receive a data packet from the card                                   */
/*-----------------------------------------------------------------------*/

static int rcvr_datablock (
    BYTE *buff,			/* Data buffer */
    UINT btr			/* Data block length (512 or 1024) */
)
{
    BYTE token;
    UINT wt = 2000;

    do {							/* Wait for data packet in timeout of 200ms */
        token = rcvr_spi();
    } while ((token == 0xFF) && --wt);
    if(token != 0xFE) return 0;		/* If not valid data token, return with error */

    rcvr_spi_multi(buff, btr);		/* Receive the data block into buffer */
    rcvr_spi();						/* Discard CRC */
    rcvr_spi();

    return 1;						/* Return with success */
}


/*-----------------------------------------------------------------------*/
/* Send a data packet to the card                                        */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0
static int xmit_datablock (
    const BYTE *buff,	/* 512 byte data block to be transmitted */
    BYTE token			/* Data/Stop token */
)
{
    BYTE resp;

    if (!wait_ready(500)) return 0;

    xmit_spi(token);					/* Xmit data token */
    if (token != 0xFD) {	/* Is data token */
        xmit_spi_multi(buff, 512);		/* Xmit the data block to the MMC */
        xmit_spi(0xFF);					/* CRC (Dummy) */
        xmit_spi(0xFF);
        resp = rcvr_spi();				/* Receive data response */
        if ((resp & 0x1F) != 0x05)		/* If not accepted, return with error */
            return 0;
    }

    return 1;
}
#endif


/*-----------------------------------------------------------------------*/
/* Send a command packet to the card                                     */
/*-----------------------------------------------------------------------*/

static BYTE send_cmd (		/* Return value: R1 response (bit7==1:Send failed) */
    BYTE cmd,		/* Command index */
    DWORD arg		/* Argument */
)
{
    BYTE n, res;

    if (cmd & 0x80) {	/* ACMD<n> is the command sequence of CMD55-CMD<n> */
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
    }

    /* Select the card and wait for ready except to stop multiple block read */
    if (cmd != CMD12) {
        deselect();
        if (!select()) return 0xFF;
    }

    /* Send command packet */
    xmit_spi(0x40 | cmd);				/* Start + Command index */
    xmit_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
    xmit_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
    xmit_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
    xmit_spi((BYTE)arg);				/* Argument[7..0] */
    n = 0x01;							/* Dummy CRC + Stop */
    if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) + Stop */
    if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) + Stop */
    xmit_spi(n);

    /* Receive command response */
    if (cmd == CMD12) rcvr_spi();		/* Skip a stuff byte when stop reading */
    n = 10;								/* Wait for a valid response in timeout of 10 attempts */
    do
        res = rcvr_spi();
    while ((res & 0x80) && --n);

    return res;			/* Return with the response value */
}


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
    if (pdrv != DEV_MMC) return STA_NOINIT;

    return Stat;
}


/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
    BYTE n, cmd, ty, ocr[4];

    if (pdrv != DEV_MMC) return STA_NOINIT;

    deselect();
    for (n = 100; n; n--) rcvr_spi();	/* 80 dummy clocks */

    ty = 0;
    if (send_cmd(CMD0, 0) == 1) {			/* Put the card SPI mode */
        if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
            rcvr_spi_multi(ocr, 4);							/* Get trailing return value of R7 resp */
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {			/* The card can work at vdd range of 2.7-3.6V */
                for (n = 1000; n && send_cmd(ACMD41, 1UL << 30); n--) __delay_cycles(8000);	/* Wait for leaving idle state (ACMD41 with HCS bit) */
                if (n && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
                    rcvr_spi_multi(ocr, 4);
                    ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 */
                }
            }
        } else {							/* SDv1 or MMCv3 */
            if (send_cmd(ACMD41, 0) <= 1) 	{
                ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
            } else {
                ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
            }
            for (n = 1000; n && send_cmd(cmd, 0); n--) __delay_cycles(8000);	/* Wait for leaving idle state */
            if (!n || send_cmd(CMD16, 512) != 0)	/* Set R/W block length to 512 */
                ty = 0;
        }
    }
    CardType = ty;
    deselect();

    if (ty) {			/* Initialization succeded */
        Stat &= ~STA_NOINIT;		/* Clear STA_NOINIT */
    } else {			/* Initialization failed */
        Stat |= STA_NOINIT;
    }

    return Stat;
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE pdrv,		/* Physical drive nmuber to identify the drive */
    BYTE *buff,		/* Data buffer to store read data */
    LBA_t sector,	/* Start sector in LBA */
    UINT count		/* Number of sectors to read */
)
{
    if (pdrv != DEV_MMC || (Stat & STA_NOINIT)) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

    if (count == 1) {	/* Single block read */
        if ((send_cmd(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
            && rcvr_datablock(buff, 512))
            count = 0;
    }
    else {				/* Multiple block read */
        if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
            do {
                if (!rcvr_datablock(buff, 512)) break;
                buff += 512;
            } while (--count);
            send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
        }
    }
    deselect();

    return count ? RES_ERROR : RES_OK;
}


/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
    BYTE pdrv,			/* Physical drive nmuber to identify the drive */
    const BYTE *buff,	/* Data to be written */
    LBA_t sector,		/* Start sector in LBA */
    UINT count			/* Number of sectors to write */
)
{
    if (pdrv != DEV_MMC || (Stat & STA_NOINIT)) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

    if (count == 1) {	/* Single block write */
        if ((send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
            && xmit_datablock(buff, 0xFE))
            count = 0;
    }
    else {				/* Multiple block write */
        if (CardType & CT_SDC) send_cmd(ACMD23, count);
        if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
            do {
                if (!xmit_datablock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN_TOKEN */
                count = 1;
        }
    }
    deselect();

    return count ? RES_ERROR : RES_OK;
}

// DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
//     // 這裡我們假設 SD 卡已經完成初始化並進入 Data Token 階段
//     // 啟動 DMA 通道 (假設使用 DMA_CHANNEL_1)
//     DMA_setSrcAddress(DMA_CHANNEL_1, (uint32_t)buff, DMA_DIRECTION_INCREMENT);
//     DMA_setDstAddress(DMA_CHANNEL_1, EUSCI_A_SPI_getTransmitBufferAddress(EUSCI_A1_BASE), DMA_DIRECTION_UNCHANGED);
    
//     // 傳送一個 Block (512 bytes)
//     DMA_setTransferSize(DMA_CHANNEL_1, 512); 
//     DMA_enableTransfers(DMA_CHANNEL_1);
//     // 啟動 SPI 的第一次觸發 (必須手動丟一個 Dummy byte 讓 TX 空出來，DMA 才會開始工作)
//     EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, 0xFF);

//     while ((DMA_getInterruptStatus(DMA_CHANNEL_1) & DMA_getInterruptStatusString(DMA_CHANNEL_1)) == 0);
//     DMA_clearInterrupt(DMA_CHANNEL_1);

//     while (EUSCI_A_SPI_isBusy(EUSCI_A1_BASE));

//     while (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == GPIO_INPUT_PIN_LOW) {
//         // 原地等待 SD 卡內部的 Flash 燒錄完成
//     }

//     // 為了相容 FatFs，這裡建議使用「等待中斷」機制
//     __bis_SR_register(LPM0_bits | GIE); // CPU 睡覺，由 DMA 中斷喚醒

//     // while ((DMA_getInterruptStatus(DMA_CHANNEL_1) & DMAIV_DMA1IFG) == 0);
//     // DMA_clearInterrupt(DMA_CHANNEL_1);   
    
//     return RES_OK;
// }

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE pdrv,		/* Physical drive nmuber (0..) */
    BYTE cmd,		/* Control code */
    void *buff		/* Buffer to send/receive control data */
)
{
    DRESULT res;
    BYTE n, csd[16];
    DWORD csize;

    if (pdrv != DEV_MMC) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    res = RES_ERROR;
    switch (cmd) {
    case CTRL_SYNC :		/* Wait for end of internal write process of the drive */
        if (select()) res = RES_OK;
        break;

    case GET_SECTOR_COUNT :	/* Get drive capacity in unit of sector (DWORD) */
        if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
            if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
                csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
                *(LBA_t*)buff = csize << 10;
            } else {					/* SDC ver 1.XX or MMC */
                n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                *(LBA_t*)buff = csize << (n - 9);
            }
            res = RES_OK;
        }
        break;

    case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
        if (CardType & CT_SD2) {	/* SDC ver 2.00 */
            *(DWORD*)buff = 16UL << (csd[10] >> 4); // Dummy or simplified
            res = RES_OK;
        } else {					/* SDC ver 1.XX or MMC */
            if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
                if (CardType & CT_SD1) {	/* SDC ver 1.XX */
                    *(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
                } else {					/* MMC */
                    *(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
                }
                res = RES_OK;
            }
        }
        break;

    default:
        res = RES_PARERR;
    }

    deselect();

    return res;
}

/*-----------------------------------------------------------------------*/
/* Real Time Clock Functions                                             */
/*-----------------------------------------------------------------------*/

extern volatile uint32_t global_tick;

DWORD get_fattime (void)
{
    /* Calculate elapsed time from global_tick (1 tick = 200ms) */
    uint32_t total_sec = global_tick / 5;
    uint32_t hour = (total_sec / 3600) % 24;
    uint32_t min = (total_sec / 60) % 60;
    uint32_t sec = total_sec % 60;

    /* Return a date/time: 2026-07-25 + calculated time */
    return ((DWORD)(2026 - 1980) << 25)
         | ((DWORD)7 << 21)
         | ((DWORD)25 << 16)
         | ((DWORD)hour << 11)
         | ((DWORD)min << 5)
         | ((DWORD)(sec / 2));
}
