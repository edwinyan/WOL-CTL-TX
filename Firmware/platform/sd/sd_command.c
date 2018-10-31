#include "common.h"
#include "sd_command.h"



void sdio_sendcommand(u32 argument,u8 command_index,u32 response)
{
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;

    SDIO_CmdInitStructure.SDIO_Argument = argument;
    SDIO_CmdInitStructure.SDIO_CmdIndex = command_index;
    SDIO_CmdInitStructure.SDIO_Response = response;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;

    SDIO_SendCommand(&SDIO_CmdInitStructure);
}

void sdio_dataconfig(u32 data_timeout,u32 data_length,u32 data_blocksize,u32 transfer_direction)
{
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    
    SDIO_DataInitStructure.SDIO_DataTimeOut = data_timeout;
    SDIO_DataInitStructure.SDIO_DataLength = data_length;
    SDIO_DataInitStructure.SDIO_DataBlockSize = data_blocksize;
    SDIO_DataInitStructure.SDIO_TransferDir = transfer_direction;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;

    SDIO_DataConfig(&SDIO_DataInitStructure);
}

