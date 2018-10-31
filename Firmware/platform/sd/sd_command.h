#ifndef _SD_COMMAND_H_
#define _SD_COMMAND_H_

void sdio_sendcommand(u32 argument,u8 command_index,u32 response);
void sdio_dataconfig(u32 data_timeout,u32 data_length,u32 data_blocksize,u32 transfer_direction);
#endif
