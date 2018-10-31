#include "stm32f4xx_sdio.h"
#include "sd_init.h" 
#include "sd_config.h"
#include "sd_rw.h"
#include "sd_drv.h"


sd_cardinfo_t sd_cardinfo;

DSTATUS sd_drv_initialize(void)
{
    sd_error_enum status;
    u8 state;
    
    state = sd_detect();
    if(state != SD_PRESENT)
    {
        return STA_NODISK;
    }
    
    status = sd_init();
    if(status != SD_OK)
    {
        return STA_NOINIT; 
    }
    status =sd_getcardinfo(&sd_cardinfo);
    if( status==SD_OK)
    {
        MSG(SYS_INFO "capacity is: %d\r\n",sd_cardinfo.CardCapacity);

        MSG(SYS_INFO "MaxBusClkFrec is: %d\r\n",sd_cardinfo.SD_csd.MaxBusClkFrec);
                
        status = sd_selectdeselect((uint32_t) (sd_cardinfo.RCA << 16));
    }
    if( status == SD_OK)
    {
        MSG(SYS_INFO "sd selected success !\r\n");
        status = sd_enable_widebusoperation(SDIO_BusWide_4b);

    }
    if( status == SD_OK)
    {
        MSG(SYS_INFO "sd bus config success !\r\n");
        //status =sd_erase(0x12345678,0x1234567A);

    }
    if( status == SD_OK)
    {
        MSG(SYS_INFO "sd erase success !\r\n"); 
    }
    return 0;
}

DSTATUS sd_drv_status(void)
{
    if (sd_detect() == SD_NOT_PRESENT)
    {
    return STA_NODISK;	// No card in the socket
    }
    
    return 0; 
}

DRESULT sd_drv_rd(u8 *buf, u32 sector_start, u16 count)
{

    sd_error_enum status;

    if (sd_detect() == SD_NOT_PRESENT)
    return RES_NOTRDY;	// No card in the socket

    if(count==1)            /* 1个sector的读操作 */      
    {       
        status = sd_readblock( buf ,sector_start*SD_SECTOR_U8_SIZE , SD_SECTOR_U8_SIZE);
    }                                                
    else                    /* 多个sector的读操作 */     
    {   
        status = sd_readmultiblocks( buf ,sector_start*SD_SECTOR_U8_SIZE ,SD_SECTOR_U8_SIZE,count);
    }                                                
    
    while(sd_getstatus() != SD_TRANSFER_OK);
    if(status == SD_OK)
    {
        return RES_OK;
    }
    else
    {
        MSG(SYS_INFO "status is %d\r\n",status);
        return RES_ERROR;
    }   
}

DRESULT sd_drv_wr(u8 *buf, u32 sector_start, u16 count)
{
    sd_error_enum status;

    if (sd_detect() == SD_NOT_PRESENT)
    return RES_NOTRDY;	// No card in the socket

    if(count==1)            /* 1个sector的写操作 */      
    {       
        status = sd_writeblock( buf ,sector_start*SD_SECTOR_U8_SIZE , SD_SECTOR_U8_SIZE);
    }                                                
    else                    /* 多个sector的写操作 */     
    {   
        status = sd_writemultiblocks( buf ,sector_start*SD_SECTOR_U8_SIZE ,SD_SECTOR_U8_SIZE,count);
    }  
    
    while(sd_getstatus() != SD_TRANSFER_OK);
    
    if(status == SD_OK)
    {
        return RES_OK;
    }
    else
    {
        MSG(SYS_INFO "status is %d\r\n",status);
        return RES_ERROR;
    }   
}

DRESULT sd_drv_ioctl(u8 cmd, u8 *cmd_buf)
{
    DRESULT res;
    sd_error_enum status;
    if (sd_detect() == SD_NOT_PRESENT)
    return RES_NOTRDY;	// No card in the socket
    
    res = RES_ERROR;

    switch (cmd) 
    {
        case CTRL_SYNC :		// Make sure that no pending write process
    	while (sd_getstatus() == SD_TRANSFER_BUSY)
    	    ;
    	res = RES_OK;
    	break;

        case GET_SECTOR_COUNT :	// Get number of sectors on the disk (unsigned long)
    	*(unsigned long*)cmd_buf = (sd_cardinfo.SD_csd.DeviceSize + 1) * 1024;
    	res = RES_OK;
    	break;

        case GET_SECTOR_SIZE :	// Get R/W sector size (unsigned short)
    	*(unsigned short*)cmd_buf = sd_cardinfo.CardBlockSize;
    	res = RES_OK;
    	break;

        case SD_WIDEBUS_4BIT :  // set SD card to 4Bit databus
        status = sd_enable_widebusoperation(SDIO_BusWide_4b);
        if(status == SD_OK)
            {
                res = RES_OK;
            }
        else
            {
                res = RES_PARERR;
            }
        break;

        case SD_WIDEBUS_1BIT :  // set SD card to 1Bit databus
        status = sd_enable_widebusoperation(SDIO_BusWide_1b);
        if(status == SD_OK)
            {
                res = RES_OK;
            }
        else
            {
                res = RES_PARERR;
            }
        break;

        case GET_CARD_CAPACITY :  // set SD card to 1Bit databus
        *(unsigned long*)cmd_buf = sd_cardinfo.CardCapacity;
        res = RES_OK;
        break;
        
        default:
    	res = RES_PARERR;
    	break;
    }
    
    return res;
}
