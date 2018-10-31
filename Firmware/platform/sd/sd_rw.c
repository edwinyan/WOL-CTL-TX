#include "stm32f4xx_sdio.h"
#include "sd_config.h"
#include "sd_rw.h"

#define NULL 0
extern uint32_t g_cardtype;
extern uint32_t g_rca;
__IO uint32_t stop_condition = 0;
__IO sd_error_enum transfer_error = SD_OK;


STATIC sd_error_enum sd_stoptransfer(void)
{
    sd_error_enum errorstatus = SD_OK;

    /*!< Send CMD12 STOP_TRANSMISSION  */
    sdio_sendcommand(0x0,SDIO_STOP_TRANSMISSION,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_STOP_TRANSMISSION);

    return(errorstatus);
}

/* Configures the DMA2 Channel4 for SDIO Tx request. */
STATIC void sd_dma_txconfig(uint32_t *BufferSRC, uint32_t BufferSize) 
{
    DMA_InitTypeDef SDDMA_InitStructure;

    DMA_ClearFlag( SD_SDIO_DMA_STREAM,   SD_SDIO_DMA_FLAG_FEIF 
                                       | SD_SDIO_DMA_FLAG_DMEIF 
                                       | SD_SDIO_DMA_FLAG_TEIF 
                                       | SD_SDIO_DMA_FLAG_HTIF 
                                       | SD_SDIO_DMA_FLAG_TCIF);

    // DMA2 Stream3 disable
    DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

    // DMA2 Stream3 Config
    DMA_DeInit(SD_SDIO_DMA_STREAM);
    SDDMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
    SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
    SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferSRC;
    SDDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    SDDMA_InitStructure.DMA_BufferSize = BufferSize;
    SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    SDDMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
    DMA_Init(SD_SDIO_DMA_STREAM, &SDDMA_InitStructure);

    DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

    // DMA2 Stream3 enable
    DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}


/* Configures the DMA2 Channel4 for SDIO Rx request. */
STATIC void sd_dma_rxconfig(uint32_t *BufferDST, uint32_t BufferSize) 
{
    DMA_InitTypeDef SDDMA_InitStructure;

    DMA_ClearFlag(SD_SDIO_DMA_STREAM,  SD_SDIO_DMA_FLAG_FEIF 
                                     | SD_SDIO_DMA_FLAG_DMEIF 
                                     | SD_SDIO_DMA_FLAG_TEIF 
                                     | SD_SDIO_DMA_FLAG_HTIF 
                                     | SD_SDIO_DMA_FLAG_TCIF);

    // DMA2 Stream3 disable
    DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

    // DMA2 Stream3 Config
    DMA_DeInit(SD_SDIO_DMA_STREAM);

    SDDMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
    SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
    SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferDST;
    SDDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    SDDMA_InitStructure.DMA_BufferSize = BufferSize;
    SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    SDDMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
    DMA_Init(SD_SDIO_DMA_STREAM, &SDDMA_InitStructure);

    DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

    // DMA2 Stream3 enable
    DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}

sd_error_enum sd_readblock(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
    sd_error_enum errorstatus = SD_OK;
    uint32_t timeout;
    uint32_t REG_ERR_STATUS;
    
    timeout = SD_DATATIMEOUT;
    transfer_error = SD_OK;

    SDIO->DCTRL = 0x0;


    if (g_cardtype == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = SD_SECTOR_U8_SIZE;
        ReadAddr  = ReadAddr >>SD_SECTOR_BIT_EXP;
    }

    /* Set Block Size for Card */ 
    sdio_sendcommand((uint32_t) BlockSize,SDIO_SET_BLOCKLEN,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_SET_BLOCKLEN);
    if (SD_OK != errorstatus)
    {
        return(errorstatus);
    }

    sdio_dataconfig(SD_DATATIMEOUT,BlockSize,SDIO_DataBlockSize_512b,SDIO_TransferDir_ToSDIO);

    /*!< Send CMD17 READ_SINGLE_BLOCK */
    sdio_sendcommand((uint32_t) ReadAddr,SDIO_READ_SINGLE_BLOCK,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_READ_SINGLE_BLOCK);
    if (errorstatus != SD_OK)
    {
        return(errorstatus);
    }

    /*!< Config and start DMA transmit */
    sd_dma_rxconfig((uint32_t *)readbuff, BlockSize);
    SDIO_DMACmd(ENABLE);
    
    
    /*!< Waiting for DMA transfer complete */
    while(((SDIO->STA & SDIO_FLAG_RXACT)) && (timeout > 0))
    {
        timeout--;  
    }

    REG_ERR_STATUS = SDIO->STA & SDIO_STA_ALLERRBITS;
    
    if (timeout == 0) 
    {
        errorstatus = SD_DATA_TIMEOUT;
        goto Escape;
    }
    else if(REG_ERR_STATUS & SDIO_FLAG_DCRCFAIL)
    {
        errorstatus = SD_DATA_CRC_FAIL;
        goto Escape;
    }
    else if (REG_ERR_STATUS & SDIO_FLAG_RXOVERR)
    {
        errorstatus = SD_RX_OVERRUN;
        goto Escape;
    }
    else if (REG_ERR_STATUS & SDIO_FLAG_STBITERR)
    {
        errorstatus = SD_START_BIT_ERR;
        goto Escape;
    }
    else //default
    {
        errorstatus = SD_OK;
        ASSERT_R(REG_ERR_STATUS == 0);
    }
    
    Escape:
        /*!< Clear all the static flags */
        SDIO_ClearFlag(SDIO_STATIC_FLAGS);

        return(errorstatus);
}


sd_error_enum sd_writeblock(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
    sd_error_enum errorstatus = SD_OK;
    uint32_t timeout;
    uint32_t REG_ERR_STATUS;
    
    timeout = SD_DATATIMEOUT;
    transfer_error = SD_OK;

    SDIO->DCTRL = 0x0;


    if (g_cardtype == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        BlockSize = SD_SECTOR_U8_SIZE;
        WriteAddr = WriteAddr >>SD_SECTOR_BIT_EXP;
    }

    /* Set Block Size for Card */ 
    sdio_sendcommand((uint32_t) BlockSize,SDIO_SET_BLOCKLEN,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
        return(errorstatus);
    }
    
    /* SDIO DATA CONFIG*/
    sdio_dataconfig(SD_DATATIMEOUT,BlockSize,SDIO_DataBlockSize_512b,SDIO_TransferDir_ToCard);

    /*!< Send CMD24 WRITE_SINGLE_BLOCK */
    sdio_sendcommand((uint32_t) WriteAddr,SDIO_WRITE_SINGLE_BLOCK,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_WRITE_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
    {
        return(errorstatus);
    }

    /*!< Config and start DMA transmit */
    sd_dma_txconfig((uint32_t *)writebuff, BlockSize);
    SDIO_DMACmd(ENABLE);


    
    /*!< Waiting for DMA transfer complete */
    while(((SDIO->STA & SDIO_FLAG_TXACT)) && (timeout > 0))
    {
        timeout--;  
    }

    REG_ERR_STATUS = SDIO->STA & SDIO_STA_ALLERRBITS;
    
    if (timeout == 0 )
    {
        errorstatus = SD_DATA_TIMEOUT;
        goto Escape;
    } 
    else if(REG_ERR_STATUS & SDIO_FLAG_DCRCFAIL)
    {
        errorstatus = SD_DATA_CRC_FAIL;
        goto Escape;
    }
    else if (REG_ERR_STATUS & SDIO_FLAG_TXUNDERR)
    {
        errorstatus = SD_TX_UNDERRUN;
        goto Escape;
    }
    else if (REG_ERR_STATUS & SDIO_FLAG_STBITERR)
    {
        errorstatus = SD_START_BIT_ERR;
        goto Escape;
    }
    else //default
    {
        errorstatus = SD_OK;
        ASSERT_R(REG_ERR_STATUS == 0);
    }
    
    Escape:
    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}


sd_error_enum sd_readmultiblocks(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks) 
{
    sd_error_enum errorstatus = SD_OK;
    uint32_t timeout;
    uint32_t REG_ERR_STATUS;
    
    timeout = SD_DATATIMEOUT;

    transfer_error = SD_OK;
    stop_condition = 1;

    /*Clear all DPSM configuration*/
    SDIO->DCTRL = 0x0;

    if (g_cardtype == SDIO_HIGH_CAPACITY_SD_CARD)	
    {
        BlockSize = SD_SECTOR_U8_SIZE;
        ReadAddr  = ReadAddr >>SD_SECTOR_BIT_EXP;
    }

    // Set Block Size for Card
    sdio_sendcommand((uint32_t) BlockSize,SDIO_SET_BLOCKLEN,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_SET_BLOCKLEN);

    if (SD_OK != errorstatus) 
    {
        goto Escape;
    }

    sdio_dataconfig(SD_DATATIMEOUT,NumberOfBlocks * BlockSize,SDIO_DataBlockSize_512b,SDIO_TransferDir_ToSDIO);

    // Send CMD18 READ_MULT_BLOCK with argument data address
    sdio_sendcommand((uint32_t) ReadAddr,SDIO_READ_MULT_BLOCK,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_READ_MULT_BLOCK);

    if (errorstatus != SD_OK) 
    {
        goto Escape;
    }

    sd_dma_rxconfig((uint32_t *)readbuff, (NumberOfBlocks * BlockSize));
    SDIO_DMACmd(ENABLE);

    /*!< Waiting for DMA transfer complete */
    while(((SDIO->STA & SDIO_FLAG_RXACT)) && (timeout > 0))
    {
        timeout--;  
    }

    REG_ERR_STATUS = SDIO->STA & SDIO_STA_ALLERRBITS;
    
    if (timeout == 0) 
    {
        errorstatus = SD_DATA_TIMEOUT;
        goto Escape;
    }
    else if(REG_ERR_STATUS & SDIO_FLAG_DCRCFAIL)
    {
        errorstatus = SD_DATA_CRC_FAIL;
        goto Escape;
    }
    else if (REG_ERR_STATUS & SDIO_FLAG_RXOVERR)
    {
        errorstatus = SD_RX_OVERRUN;
        goto Escape;
    }
    else if (REG_ERR_STATUS & SDIO_FLAG_STBITERR)
    {
        errorstatus = SD_START_BIT_ERR;
        goto Escape;
    }
    else //default
    {
        errorstatus = SD_OK;
        ASSERT_R(REG_ERR_STATUS == 0);
    }

    if (stop_condition == 1)
    {
        errorstatus = sd_stoptransfer();
    }

    Escape:
    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}

sd_error_enum sd_writemultiblocks(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks) 
{
    sd_error_enum errorstatus = SD_OK;
    uint32_t timeout;
    uint32_t REG_ERR_STATUS;
    
    timeout = SD_DATATIMEOUT;
    transfer_error = SD_OK;
    stop_condition = 1;

    /*Clear all DPSM configuration*/
    SDIO->DCTRL = 0x0;

    if (g_cardtype == SDIO_HIGH_CAPACITY_SD_CARD) 
    {
        BlockSize = SD_SECTOR_U8_SIZE;
        WriteAddr = WriteAddr >>SD_SECTOR_BIT_EXP;
    }
    
    // set ACMD23(pre-erease) to improve MultiWrite Speed
    // To improve performance
    sdio_sendcommand((uint32_t)(g_rca << 16),SDIO_APP_CMD,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_APP_CMD);

    if (errorstatus != SD_OK) 
    {
        goto Escape;
    }
    
    // To improve performance
    sdio_sendcommand((uint32_t)NumberOfBlocks,SDIO_SET_BLOCK_COUNT,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_SET_BLOCK_COUNT);

    if (errorstatus != SD_OK)
    {
        goto Escape;
    }
    
    // Set Block Size for Card
    sdio_sendcommand((uint32_t) BlockSize,SDIO_SET_BLOCKLEN,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_SET_BLOCKLEN);

    if (SD_OK != errorstatus) 
    {
        goto Escape;
    }
    
    // Send CMD25 WRITE_MULT_BLOCK with argument data address
    sdio_sendcommand((uint32_t) WriteAddr,SDIO_WRITE_MULT_BLOCK,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_WRITE_MULT_BLOCK);

    if (SD_OK != errorstatus) 
    {
        goto Escape;
    }

    sdio_dataconfig(SD_DATATIMEOUT,NumberOfBlocks * BlockSize,SDIO_DataBlockSize_512b,SDIO_TransferDir_ToCard);

    /*!< Config and start DMA transmit */
    sd_dma_txconfig((uint32_t *)writebuff, (NumberOfBlocks * BlockSize));
    SDIO_DMACmd(ENABLE);

    /*!< Waiting for DMA transfer complete */
    while(((SDIO->STA & SDIO_FLAG_TXACT)) && (timeout > 0))
    {
        timeout--;  
    }
    
    REG_ERR_STATUS = SDIO->STA & SDIO_STA_ALLERRBITS;
    
    if (timeout == 0 )
    {
        errorstatus = SD_DATA_TIMEOUT;
        goto Escape;
    } 
    else if(REG_ERR_STATUS & SDIO_FLAG_DCRCFAIL)
    {
        errorstatus = SD_DATA_CRC_FAIL;
        goto Escape;
    }
    else if (REG_ERR_STATUS & SDIO_FLAG_TXUNDERR)
    {
        errorstatus = SD_TX_UNDERRUN;
        goto Escape;
    }
    else if (REG_ERR_STATUS & SDIO_FLAG_STBITERR)
    {
        errorstatus = SD_START_BIT_ERR;
        goto Escape;
    }
    else //default
    {
        errorstatus = SD_OK;
        ASSERT_R(REG_ERR_STATUS == 0);
    }

    if (stop_condition == 1)
    {
        errorstatus = sd_stoptransfer();
    }


    Escape:
    /*!< Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}

