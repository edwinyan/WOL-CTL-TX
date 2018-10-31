#include "sd_config.h"
#include "bsp_os.h"

extern uint32_t g_cardtype;
extern uint32_t g_csdtab[4];
extern uint32_t g_cidtab[4];
extern uint32_t g_rca;

uint8_t sd_detect(void)//检测SD是否插入
{
    __IO uint8_t status = SD_PRESENT;

    /*!< Check GPIO to detect SD */
    if (GPIO_ReadInputDataBit(SD_DETECT_PORT, SD_DETECT_PIN) != Bit_RESET)
    {
        status = SD_NOT_PRESENT;
    }
    return status;
}

void sd_reset(void)
{
    GPIO_ResetBits(SD_POWER_PORT, SD_POWER_PIN);
    BSP_OS_TimeDly(1);
    GPIO_SetBits(SD_POWER_PORT, SD_POWER_PIN);
    BSP_OS_TimeDly(1);
}

STATIC sd_error_enum sd_sendstatus(uint32_t *pcardstatus)
{
    sd_error_enum errorstatus = SD_OK;

    if (pcardstatus == NULL)
    {
        errorstatus = SD_INVALID_PARAMETER;
        return(errorstatus);
    }

    /*SEND CMD13 TO GET SD CARD STATUS*/
    sdio_sendcommand((uint32_t) g_rca << 16,SDIO_SEND_STATUS,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_SEND_STATUS);

    if (errorstatus != SD_OK)
    {
        return(errorstatus);
    }

    *pcardstatus = SDIO_GetResponse(SDIO_RESP1);

    return(errorstatus);
}


STATIC sd_cardstate_enum sd_getstate(void)
{
    uint32_t resp1 = 0;

    if(sd_detect()== SD_PRESENT)
    {
        if (sd_sendstatus(&resp1) != SD_OK)
        {
            return SD_CARD_ERROR;
        }
        else
        {
            return (sd_cardstate_enum)((resp1 >> 9) & 0x0F);
        }
    }
    else
    {
        return SD_CARD_ERROR;
    }
}


sd_transferstate_enum sd_getstatus(void)
{
    sd_cardstate_enum cardstate =  SD_CARD_TRANSFER;

    cardstate = sd_getstate();

    if (cardstate == SD_CARD_TRANSFER)
    {
        return(SD_TRANSFER_OK);
    }
    else if(cardstate == SD_CARD_ERROR)
    {
        return (SD_TRANSFER_ERROR);
    }
    else
    {
        return(SD_TRANSFER_BUSY);
    } 
}


/**
  * @brief  Find the SD card SCR register value.
  * @param  rca: selected card address.
  * @param  pscr: pointer to the buffer that will contain the SCR value.
  * @retval SD_Error: SD Card Error code.
  */
STATIC sd_error_enum find_scr(uint16_t rca, uint32_t *pscr)
{
    uint32_t index = 0;
    sd_error_enum errorstatus = SD_OK;
    uint32_t tempscr[2] = {0, 0};
    uint32_t time_deley;

    /* Set Block Size To 8 Bytes */
    /* Send CMD55 APP_CMD with argument as card's RCA */
    sdio_sendcommand((uint32_t)8,SDIO_SET_BLOCKLEN,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_SET_BLOCKLEN);

    if (errorstatus != SD_OK)
    {
        return(errorstatus);
    }

    /* Send CMD55 APP_CMD with argument as card's RCA */
    sdio_sendcommand((uint32_t) g_rca << 16,SDIO_APP_CMD,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_APP_CMD);

    if (errorstatus != SD_OK)
    {
        return(errorstatus);
    }

    sdio_dataconfig(SD_DATATIMEOUT,0x08,SDIO_DataBlockSize_8b,SDIO_TransferDir_ToSDIO);

    /*time for SDIO controller to handle operate*/
    for(time_deley=0;time_deley<20;time_deley++)
        ;
    /* Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
    sdio_sendcommand(0x0,SDIO_SD_APP_SEND_SCR,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_SD_APP_SEND_SCR);

    if (errorstatus != SD_OK)
    {
        return(errorstatus);
    }

    while (!(SDIO->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
    {
        if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
        {
            *(tempscr + index) = SDIO_ReadData();
            index++;
            if(index == 2)
            {
                break;
            }
        }
    }

    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
    {
        SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
    {
        SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
        errorstatus = SD_DATA_CRC_FAIL;
        return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
    {
        SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
        errorstatus = SD_RX_OVERRUN;
        return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
    {
        SDIO_ClearFlag(SDIO_FLAG_STBITERR);
        errorstatus = SD_START_BIT_ERR;
        return(errorstatus);
    }
    /* Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    *(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) 
                | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

    *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) 
            | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);
    return(errorstatus);
}


/**
  * @brief  Enables or disables the SDIO wide bus mode.
  * @param  NewState: new state of the SDIO wide bus mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval SD_Error: SD Card Error code.
  */
STATIC sd_error_enum sd_enwidebus(FunctionalState NewState)
{
    sd_error_enum errorstatus = SD_OK;
    uint32_t scr[2] = {0, 0};

    if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
    {
        errorstatus = SD_LOCK_UNLOCK_FAILED;
        return(errorstatus);
    }

    /* Get SCR Register */
    errorstatus = find_scr(g_rca, scr);
    
    if (errorstatus != SD_OK)
    {
        return(errorstatus);
    }

    /* If wide bus operation to be enabled */
    if (NewState == ENABLE)
    {
        /* If requested card supports wide bus operation */
        if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)
        {
            /* Send CMD55 APP_CMD with argument as card's RCA.*/
            sdio_sendcommand((uint32_t) g_rca << 16,SDIO_APP_CMD,SDIO_Response_Short);

            errorstatus = cmd_resp1_error(SDIO_APP_CMD);

            if (errorstatus != SD_OK)
            {
                return(errorstatus);
            }

            /* Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
            sdio_sendcommand(0x2,SDIO_APP_SD_SET_BUSWIDTH,SDIO_Response_Short);
            errorstatus = cmd_resp1_error(SDIO_APP_SD_SET_BUSWIDTH);
            
            if (errorstatus != SD_OK)
            {
                return(errorstatus);
            }
            return(errorstatus);
        }
        else
        {
            errorstatus = SD_REQUEST_NOT_APPLICABLE;
            return(errorstatus);
        }
    }   /* If wide bus operation to be disabled */
    else
    {
        /* If requested card supports 1 bit mode operation */
        if ((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO)
        {   
            /* Send CMD55 APP_CMD with argument as card's RCA.*/
            sdio_sendcommand((uint32_t) g_rca << 16,SDIO_APP_CMD,SDIO_Response_Short);

            errorstatus = cmd_resp1_error(SDIO_APP_CMD);

            if (errorstatus != SD_OK)
            {
                return(errorstatus);
            }

            /* Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
            sdio_sendcommand(0x00,SDIO_APP_SD_SET_BUSWIDTH,SDIO_Response_Short);
            errorstatus = cmd_resp1_error(SDIO_APP_SD_SET_BUSWIDTH);

            if (errorstatus != SD_OK)
            {
                return(errorstatus);
            }

            return(errorstatus);
        }
        else
        {
        errorstatus = SD_REQUEST_NOT_APPLICABLE;
        return(errorstatus);
        }
    }
}


/**
  * @brief  Returns information about specific card.
  * @param  cardinfo: pointer to a SD_CardInfo structure that contains all SD card 
  *         information.
  * @retval SD_Error: SD Card Error code.
  */
sd_error_enum sd_getcardinfo(sd_cardinfo_t *cardinfo)
{
    sd_error_enum errorstatus = SD_OK;
    uint8_t tmp = 0;

    cardinfo->CardType = (uint8_t)g_cardtype;
    cardinfo->RCA = (uint16_t)g_rca;

    /*!< Byte 0 */
    tmp = (uint8_t)((g_csdtab[0] & 0xFF000000) >> 24);
    cardinfo->SD_csd.CSDStruct = (tmp & 0xC0) >> 6;
    cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
    cardinfo->SD_csd.Reserved1 = tmp & 0x03;

    /*!< Byte 1 */
    tmp = (uint8_t)((g_csdtab[0] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.TAAC = tmp;

    /*!< Byte 2 */
    tmp = (uint8_t)((g_csdtab[0] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.NSAC = tmp;

    /*!< Byte 3 */
    tmp = (uint8_t)(g_csdtab[0] & 0x000000FF);
    cardinfo->SD_csd.MaxBusClkFrec = tmp;

    /*!< Byte 4 */
    tmp = (uint8_t)((g_csdtab[1] & 0xFF000000) >> 24);
    cardinfo->SD_csd.CardComdClasses = tmp << 4;

    /*!< Byte 5 */
    tmp = (uint8_t)((g_csdtab[1] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
    cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;

    /*!< Byte 6 */
    tmp = (uint8_t)((g_csdtab[1] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;
    cardinfo->SD_csd.Reserved2 = 0; /*!< Reserved */

    if ((g_cardtype == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (g_cardtype == SDIO_STD_CAPACITY_SD_CARD_V2_0))
    {
        cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

        /*!< Byte 7 */
        tmp = (uint8_t)(g_csdtab[1] & 0x000000FF);
        cardinfo->SD_csd.DeviceSize |= (tmp) << 2;

        /*!< Byte 8 */
        tmp = (uint8_t)((g_csdtab[2] & 0xFF000000) >> 24);
        cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

        cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
        cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

        /*!< Byte 9 */
        tmp = (uint8_t)((g_csdtab[2] & 0x00FF0000) >> 16);
        cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
        cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
        cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;
        /*!< Byte 10 */
        tmp = (uint8_t)((g_csdtab[2] & 0x0000FF00) >> 8);
        cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;

        cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
        cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
        cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
        cardinfo->CardCapacity *= cardinfo->CardBlockSize;
    }
    else if (g_cardtype == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        /*!< Byte 7 */
        tmp = (uint8_t)(g_csdtab[1] & 0x000000FF);
        cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;

        /*!< Byte 8 */
        tmp = (uint8_t)((g_csdtab[2] & 0xFF000000) >> 24);

        cardinfo->SD_csd.DeviceSize |= (tmp << 8);

        /*!< Byte 9 */
        tmp = (uint8_t)((g_csdtab[2] & 0x00FF0000) >> 16);

        cardinfo->SD_csd.DeviceSize |= (tmp);

        /*!< Byte 10 */
        tmp = (uint8_t)((g_csdtab[2] & 0x0000FF00) >> 8);

        cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) * 512 * 1024;
        cardinfo->CardBlockSize = 512;    
    }


    cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.EraseGrMul = (tmp & 0x3F) << 1;

    /*!< Byte 11 */
    tmp = (uint8_t)(g_csdtab[2] & 0x000000FF);
    cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
    cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);

    /*!< Byte 12 */
    tmp = (uint8_t)((g_csdtab[3] & 0xFF000000) >> 24);
    cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
    cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
    cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;

    /*!< Byte 13 */
    tmp = (uint8_t)((g_csdtab[3] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
    cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.Reserved3 = 0;
    cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);

    /*!< Byte 14 */
    tmp = (uint8_t)((g_csdtab[3] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
    cardinfo->SD_csd.FileFormat = (tmp & 0x0C) >> 2;
    cardinfo->SD_csd.ECC = (tmp & 0x03);

    /*!< Byte 15 */
    tmp = (uint8_t)(g_csdtab[3] & 0x000000FF);
    cardinfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
    cardinfo->SD_csd.Reserved4 = 1;


    /*!< Byte 0 */
    tmp = (uint8_t)((g_cidtab[0] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ManufacturerID = tmp;

    /*!< Byte 1 */
    tmp = (uint8_t)((g_cidtab[0] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.OEM_AppliID = tmp << 8;

    /*!< Byte 2 */
    tmp = (uint8_t)((g_cidtab[0] & 0x000000FF00) >> 8);
    cardinfo->SD_cid.OEM_AppliID |= tmp;

    /*!< Byte 3 */
    tmp = (uint8_t)(g_cidtab[0] & 0x000000FF);
    cardinfo->SD_cid.ProdName1 = tmp << 24;

    /*!< Byte 4 */
    tmp = (uint8_t)((g_cidtab[1] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdName1 |= tmp << 16;

    /*!< Byte 5 */
    tmp = (uint8_t)((g_cidtab[1] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.ProdName1 |= tmp << 8;

    /*!< Byte 6 */
    tmp = (uint8_t)((g_cidtab[1] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ProdName1 |= tmp;

    /*!< Byte 7 */
    tmp = (uint8_t)(g_cidtab[1] & 0x000000FF);
    cardinfo->SD_cid.ProdName2 = tmp;

    /*!< Byte 8 */
    tmp = (uint8_t)((g_cidtab[2] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdRev = tmp;

    /*!< Byte 9 */
    tmp = (uint8_t)((g_cidtab[2] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.ProdSN = tmp << 24;

    /*!< Byte 10 */
    tmp = (uint8_t)((g_cidtab[2] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ProdSN |= tmp << 16;

    /*!< Byte 11 */
    tmp = (uint8_t)(g_cidtab[2] & 0x000000FF);
    cardinfo->SD_cid.ProdSN |= tmp << 8;

    /*!< Byte 12 */
    tmp = (uint8_t)((g_cidtab[3] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdSN |= tmp;

    /*!< Byte 13 */
    tmp = (uint8_t)((g_cidtab[3] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
    cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;

    /*!< Byte 14 */
    tmp = (uint8_t)((g_cidtab[3] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ManufactDate |= tmp;

    /*!< Byte 15 */
    tmp = (uint8_t)(g_cidtab[3] & 0x000000FF);
    cardinfo->SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
    cardinfo->SD_cid.Reserved2 = 1;

    return(errorstatus);
}

/*
 * 函数名：SD_SelectDeselect
 * 描述  ：Selects od Deselects the corresponding card
 * 输入  ：-addr 选择卡的地址
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：外部调用
 */
sd_error_enum sd_selectdeselect(uint32_t addr)
{
    sd_error_enum errorstatus = SD_OK;

    /* Send CMD7 SDIO_SEL_DESEL_CARD */
    sdio_sendcommand(addr,SDIO_SEL_DESEL_CARD,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_SEL_DESEL_CARD);

    return(errorstatus);
}



/*
 * 函数名：SD_EnableWideBusOperation
 * 描述  ：配置卡的数据宽度(但得看卡是否支持)
 * 输入  ：-WideMode 指定SD卡的数据线宽
 *         具体可配置如下
 *         @arg SDIO_BusWide_4b: 4-bit data transfer
 *         @arg SDIO_BusWide_1b: 1-bit data transfer (默认)
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：外部调用
 */
sd_error_enum sd_enable_widebusoperation(uint32_t WideMode)
{
    SDIO_InitTypeDef SDIO_InitStructure;
    sd_error_enum errorstatus = SD_OK;

    /* MMC Card doesn't support this feature */
    if (SDIO_MULTIMEDIA_CARD == g_cardtype)
    {
        errorstatus = SD_UNSUPPORTED_FEATURE;
        return(errorstatus);
    }
    else if (   (SDIO_STD_CAPACITY_SD_CARD_V1_1 == g_cardtype) 
             || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == g_cardtype) 
             || (SDIO_HIGH_CAPACITY_SD_CARD == g_cardtype)   )
    {
        if (SDIO_BusWide_8b == WideMode)
        {
            errorstatus = SD_UNSUPPORTED_FEATURE;
            return(errorstatus);
        }
        else if (SDIO_BusWide_4b == WideMode)
        {
            errorstatus = sd_enwidebus(ENABLE);
            if (SD_OK == errorstatus)
            {
                /* Configure the SDIO peripheral */
                SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
                SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
                SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
                SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
                SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_4b;
                SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
                SDIO_Init(&SDIO_InitStructure);
            }
        }
        else
        {
            errorstatus = sd_enwidebus(DISABLE);
            if (SD_OK == errorstatus)
            {
                /* Configure the SDIO peripheral */
                SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
                SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
                SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
                SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
                SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
                SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
                SDIO_Init(&SDIO_InitStructure);
            }
        }
    }
    return(errorstatus);
}



/**
  * @brief  Allows to erase memory area specified for the given card.
  * @param  startaddr: the start address.
  * @param  endaddr: the end address.
  * @retval SD_Error: SD Card Error code.
  */
sd_error_enum sd_erase(uint32_t startaddr, uint32_t endaddr)
{
    sd_error_enum errorstatus = SD_OK;
    uint32_t delay = 0;
    __IO uint32_t maxdelay = 0;
    uint8_t cardstate = 0;

    /* Check if the card coomnd class supports erase command */
    if (((g_csdtab[1] >> 20) & SD_CCCC_ERASE) == 0)
    {
        errorstatus = SD_REQUEST_NOT_APPLICABLE;
        return(errorstatus);
    }

    maxdelay = 168000 / ((SDIO->CLKCR & 0xFF) + 2);

    if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
    {
        errorstatus = SD_LOCK_UNLOCK_FAILED;
        return(errorstatus);
    }

    if (g_cardtype == SDIO_HIGH_CAPACITY_SD_CARD)
    {
        startaddr =startaddr >> SD_SECTOR_BIT_EXP;
        endaddr   =endaddr >> SD_SECTOR_BIT_EXP;
    }

    /* According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) */
    if (   (SDIO_STD_CAPACITY_SD_CARD_V1_1 == g_cardtype) 
        || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == g_cardtype) 
        || (SDIO_HIGH_CAPACITY_SD_CARD == g_cardtype)   )
    {
        /* Send CMD32 SD_ERASE_GRP_START with argument as addr  */
        sdio_sendcommand(startaddr,SDIO_SD_ERASE_GRP_START,SDIO_Response_Short);

        errorstatus = cmd_resp1_error(SDIO_SD_ERASE_GRP_START);
        if (errorstatus != SD_OK)
        {
            return(errorstatus);
        }

        /* Send CMD33 SD_ERASE_GRP_END with argument as addr  */
        sdio_sendcommand(endaddr,SDIO_SD_ERASE_GRP_END,SDIO_Response_Short);

        errorstatus = cmd_resp1_error(SDIO_SD_ERASE_GRP_END);
        if (errorstatus != SD_OK)
        {
            return(errorstatus);
        }
    }

    /* Send CMD38 ERASE */
    sdio_sendcommand(0x00,SDIO_ERASE,SDIO_Response_Short);

    errorstatus = cmd_resp1_error(SDIO_ERASE);

    if (errorstatus != SD_OK)
    {
        return(errorstatus);
    }

    for (delay = 0; delay < maxdelay; delay++)
    {}

    /* Wait till the card is in programming state */
    cardstate=sd_getstate();

    while ((errorstatus == SD_OK) && ((SD_CARD_PROGRAMMING == cardstate) || (SD_CARD_RECEIVING == cardstate)))
    {
        cardstate=sd_getstate();
    }
        return(errorstatus);
}

