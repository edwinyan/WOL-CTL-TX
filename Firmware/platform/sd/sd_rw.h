#ifndef _SD_RW_H_
#define _SD_RW_H_

#include "sd_response.h"
#include "sd_command.h"




/**
  * @brief  SD DMA Config
  */
#define SDIO_FIFO_ADDRESS                (0x40012c80L)
#define SD_SDIO_DMA                      DMA2
#define SD_SDIO_DMA_STREAM3	             3

#define SD_SDIO_DMA_STREAM               DMA2_Stream3
#define SD_SDIO_DMA_CHANNEL              DMA_Channel_4
#define SD_SDIO_DMA_FLAG_FEIF            DMA_FLAG_FEIF3
#define SD_SDIO_DMA_FLAG_DMEIF           DMA_FLAG_DMEIF3
#define SD_SDIO_DMA_FLAG_TEIF            DMA_FLAG_TEIF3
#define SD_SDIO_DMA_FLAG_HTIF            DMA_FLAG_HTIF3
#define SD_SDIO_DMA_FLAG_TCIF            DMA_FLAG_TCIF3


sd_error_enum sd_readblock(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize);
sd_error_enum sd_writeblock(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize);
sd_error_enum sd_readmultiblocks(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks);
sd_error_enum sd_writemultiblocks(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks);

#endif
