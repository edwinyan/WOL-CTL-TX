#include "common.h"
#include "sd_response.h"
#include "bsp_os.h"

 /*card error status table*/
card_error_status_t card_error_status_tab[ERROR_STATUS_ENUM]={{SD_OCR_ADDR_OUT_OF_RANGE    ,SD_ADDR_OUT_OF_RANGE},
                                                              {SD_OCR_ADDR_MISALIGNED      ,SD_ADDR_MISALIGNED},
                                                              {SD_OCR_BLOCK_LEN_ERR        ,SD_BLOCK_LEN_ERR},
                                                              {SD_OCR_ERASE_SEQ_ERR        ,SD_ERASE_SEQ_ERR},
                                                              {SD_OCR_BAD_ERASE_PARAM      ,SD_BAD_ERASE_PARAM},
                                                              {SD_OCR_WRITE_PROT_VIOLATION ,SD_WRITE_PROT_VIOLATION},
                                                              {SD_OCR_LOCK_UNLOCK_FAILED   ,SD_LOCK_UNLOCK_FAILED},
                                                              {SD_OCR_COM_CRC_FAILED       ,SD_COM_CRC_FAILED},
                                                              {SD_OCR_ILLEGAL_CMD          ,SD_ILLEGAL_CMD},
                                                              {SD_OCR_CARD_ECC_FAILED      ,SD_CARD_ECC_FAILED},
                                                              {SD_OCR_CC_ERROR             ,SD_CC_ERROR},
                                                              {SD_OCR_GENERAL_UNKNOWN_ERROR,SD_GENERAL_UNKNOWN_ERROR},
                                                              {SD_OCR_STREAM_WRITE_OVERRUN ,SD_STREAM_WRITE_OVERRUN},
                                                              {SD_OCR_CID_CSD_OVERWRIETE   ,SD_CID_CSD_OVERWRITE},
                                                              {SD_OCR_WP_ERASE_SKIP        ,SD_WP_ERASE_SKIP},
                                                              {SD_OCR_CARD_ECC_DISABLED    ,SD_CARD_ECC_DISABLED},
                                                              {SD_OCR_ERASE_RESET          ,SD_ERASE_RESET},
                                                              {SD_OCR_AKE_SEQ_ERROR        ,SD_AKE_SEQ_ERROR},
                                                              {SD_R6_GENERAL_UNKNOWN_ERROR ,SD_GENERAL_UNKNOWN_ERROR},
                                                              {SD_R6_ILLEGAL_CMD           ,SD_ILLEGAL_CMD},
                                                              {SD_R6_COM_CRC_FAILED        ,SD_COM_CRC_FAILED}
                                                             };

 
/**
  * @brief  Checks for error conditions for CMD0.
  * @param  None
  * @retval sd_error_enum: SD Card Error code.
  */
sd_error_enum cmd_error(void)
{
    sd_error_enum errorstatus = SD_OK;
    uint32_t timeout;
    uint32_t status;

    timeout = SDIO_CMD0TIMEOUT; /* 10000 */
    status = SDIO->STA;
    
    while (!(status & (SDIO_STA_ALLERRBITS |SDIO_FLAG_CMDSENT)) && (timeout > 0))
    {
        status = SDIO->STA;
        timeout--;
    }

    status = SDIO->STA & SDIO_STA_ALLERRBITS;
    if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT))
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        goto Escape;
    }
    else if(status & SDIO_FLAG_CCRCFAIL)
    {
        errorstatus = SD_CMD_CRC_FAIL;
        goto Escape;
    }
    else
    {
        errorstatus = SD_OK;
        ASSERT_R(status == 0);
    }
    Escape:
    /* Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}



/**
  * @brief  Checks for error conditions for R1.
  *   response
  * @param  cmd: The sent command index.
  * @retval sd_error_enum: SD Card Error code.
  */
sd_error_enum cmd_resp1_error(uint8_t cmd)
{
    sd_error_enum errorstatus = SD_OK;
    uint8_t check_count;
    uint32_t status;
    uint32_t response_r1;

    status = SDIO->STA;

    while (!(status & (SDIO_STA_ALLERRBITS | SDIO_FLAG_CMDREND)))
    {
        status = SDIO->STA;
    }

    status = SDIO->STA & SDIO_STA_ALLERRBITS;
    
    if (status & SDIO_FLAG_CTIMEOUT)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL)
    {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
        return(errorstatus);
    }
    else
    {
        ASSERT_R(status == 0);
    }

    /* Check response received is of desired command */
    if (SDIO_GetCommandResponse() != cmd)
    {
        errorstatus = SD_ILLEGAL_CMD;
        return(errorstatus);
    }

    /* Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    /* We have received response, retrieve it for analysis  */
    response_r1 = SDIO_GetResponse(SDIO_RESP1);

    if ((response_r1 & SD_OCR_ERRORBITS) == SD_ALLZERO)
    {
        return(errorstatus);
    }
    else
    {
        for(check_count=R1_CHECK_START;check_count<R1_CHECK_END;check_count++)
        {
            if(response_r1 & card_error_status_tab[check_count].card_status)
            {
                return(card_error_status_tab[check_count].error_code);
            }
        }
    }
    
    return(errorstatus);
}



/**
  * @brief  Checks for error conditions for R2 (CID or CSD).
  *   response.
  * @param  None
  * @retval sd_error_enum: SD Card Error code.
  */
sd_error_enum cmd_resp2_error(void)
{
    sd_error_enum errorstatus = SD_OK;
    uint32_t status;

    status = SDIO->STA;

    while (!(status & (SDIO_STA_ALLERRBITS | SDIO_FLAG_CMDREND)))
    {
        status = SDIO->STA;
    }

    status = SDIO->STA & SDIO_STA_ALLERRBITS;
    
    if (status & SDIO_FLAG_CTIMEOUT)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        goto Escape;
    }
    else if (status & SDIO_FLAG_CCRCFAIL)
    {
        errorstatus = SD_CMD_CRC_FAIL;
        goto Escape;
    }

    else
    {
        ASSERT_R(status == 0);
        errorstatus = SD_OK;
    }
    Escape:
    /* Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}



/**
  * @brief  Checks for error conditions for R3 (OCR).
  *   response.
  * @param  None
  * @retval sd_error_enum: SD Card Error code.
  */
sd_error_enum cmd_resp3_error(void)
{
    sd_error_enum errorstatus = SD_OK;
    uint32_t status;

    status = SDIO->STA;

    while (!(status & (SDIO_STA_ALLERRBITS | SDIO_FLAG_CMDREND )))
    {
        status = SDIO->STA;
    }

    status = SDIO->STA & SDIO_STA_ALLERRBITS;
    
    if (status & SDIO_FLAG_CTIMEOUT)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL)
    {
        //WE DONT CARE ABOUT R3 CRCERROR BIT
        SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    }
    else
    {
        ASSERT_R(status == 0);
    }

    /* Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);
    return(errorstatus);
}



/**
  * @brief  Checks for error conditions for R6 (RCA).
  *   response.
  * @param  cmd: The sent command index.
  * @param  prca: pointer to the variable that will contain the SD
  *   card relative address RCA. 
  * @retval sd_error_enum: SD Card Error code.
  */
sd_error_enum cmd_resp6_error(uint8_t cmd, uint16_t *prca)
{
    sd_error_enum errorstatus = SD_OK;
    uint8_t check_count;
    uint32_t status;
    uint32_t response_r1;

    status = SDIO->STA;

    while (!(status & (SDIO_STA_ALLERRBITS | SDIO_FLAG_CMDREND)))
    {
        status = SDIO->STA;
    }

    status = SDIO->STA & SDIO_STA_ALLERRBITS;
    
    if (status & SDIO_FLAG_CTIMEOUT)
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL)
    {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
        return(errorstatus);
    }
    else
    {
        ASSERT_R(status == 0);
    }

    /* Check response received is of desired command */
    if (SDIO_GetCommandResponse() != cmd)
    {
        errorstatus = SD_ILLEGAL_CMD;
        return(errorstatus);
    }

    /* Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    /* We have received response, retrieve it.  */
    response_r1 = SDIO_GetResponse(SDIO_RESP1);

    if (SD_ALLZERO == (response_r1 & SD_R6_ERRORBITS))
    {
        *prca = (uint16_t) (response_r1 >> 16);
        return(errorstatus);
    }
    else
    {
        for(check_count=R6_CHECK_START;check_count<R6_CHECK_END;check_count++)
        {
            if(response_r1 & card_error_status_tab[check_count].card_status)
            {
                return(card_error_status_tab[check_count].error_code);
            }
        }
    }
    return(errorstatus);
}


/**
  * @brief  Checks for error conditions for R7.
  *   response.
  * @param  None
  * @retval sd_error_enum: SD Card Error code.
  */
sd_error_enum cmd_resp7_error(void)
{
    sd_error_enum errorstatus = SD_OK;
    uint32_t status;
    uint32_t timeout = SDIO_CMD0TIMEOUT;

    status = SDIO->STA;

    while (!(status & (SDIO_STA_ALLERRBITS | SDIO_FLAG_CMDREND)) && (timeout > 0))
    {
        timeout--;
        status = SDIO->STA;
    }

    status = SDIO->STA & SDIO_STA_ALLERRBITS;
    
    if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT))
    {
        /* Card is not V2.0 complient or card does not support the set voltage range */
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
        return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL)
    {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
        return(errorstatus);
    }
    else
    {
        ASSERT_R(status == 0);
        errorstatus = SD_OK;
    }

    /* Clear all the static flags */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);  
    
    return(errorstatus);
}

