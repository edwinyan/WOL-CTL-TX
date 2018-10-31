#ifndef _SD_DRV_H_
#define _SD_DRV_H_

#include "common.h"
#include "diskio.h"

DSTATUS sd_drv_initialize(void);
DSTATUS sd_drv_status(void);
DRESULT sd_drv_rd(u8 *buf, u32 sector_start, u16 count);
DRESULT sd_drv_wr(u8 *buf, u32 sector_start, u16 count);
DRESULT sd_drv_ioctl(u8 cmd, u8 *cmd_buf);
#endif
