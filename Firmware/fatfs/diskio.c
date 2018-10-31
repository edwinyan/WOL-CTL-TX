/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
//#include "usbdisk.h"	/* Example: USB drive control */
//#include "atadrive.h"	/* Example: ATA drive control */
#include "sd_drv.h"		/* Example: MMC/SDC contorl */

/* Definitions of physical drive number for each media */
#define ATA		0
#define MMC		1
#define USB		2


/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
	DSTATUS stat = STA_NOINIT;

	switch (pdrv) 
    {
    	case ATA :
    		stat = STA_NODISK; //ATA_disk_initialize();
            break;
    	case MMC :
    		stat = sd_drv_initialize();
    		break;
    	case USB :
    		stat = STA_NODISK; //USB_disk_initialize();
    		break; 
	}
    
	return stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{
	DSTATUS stat = STA_NOINIT;

	switch (pdrv) 
    {
    	case ATA :
    		stat = STA_NODISK; //ATA_disk_status();
            break;
    	case MMC :
    		stat = sd_drv_status();
    		break;
    	case USB :
    		stat = STA_NODISK; //USB_disk_status();
            break;
	}
    
	return stat;
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	DRESULT res = RES_ERROR;
	switch (pdrv) 
    {
    	case ATA :
    		res = RES_ERROR; //ATA_disk_read(buff, sector, count);
            break;
    	case MMC :
    		res = sd_drv_rd(buff, sector, count);
            break;
    	case USB :
    		res = RES_ERROR; //USB_disk_read(buff, sector, count);
            break;
        default:
            break;
	}
	return res;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	DRESULT res = RES_ERROR;

	switch (pdrv) 
    {
 	    case MMC :
    		res = sd_drv_wr((u8 *)buff, sector, count);
            break;
        default:
            break;
    }
    
	return res;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res = RES_ERROR;

	switch (pdrv) 
    {
 	    case MMC :
    		res = sd_drv_ioctl(cmd, buff);
            break;
        default:
            break;
    }
    
	return res;
}

