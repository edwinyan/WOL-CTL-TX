#include "common.h"
#include "sd_init.h"
#include "stm32f4xx_gpio.h"

#define SDIO_SEND_IF_COND               (0x00000008L)

#define SD_CHECK_PATTERN                (0x000001AAL)
#define SD_VOLTAGE_WINDOW_SD            (0x80100000L)
#define SD_HIGH_CAPACITY                (0x40000000L)
#define SD_STD_CAPACITY                 (0x00000000L)

#define SD_MAX_VOLT_TRIAL               (0x0000FFFFL)

uint32_t g_cardtype =  SDIO_STD_CAPACITY_SD_CARD_V1_1;
uint32_t g_csdtab[4], g_cidtab[4], g_rca = 0;


STATIC void sd_gpio_configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIOC and GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_SDIO);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_SDIO);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SDIO);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SDIO);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SDIO);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_SDIO);

    /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure PD.02 CMD line */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Configure PC.12 pin: CLK pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure PC.13 pin: SD_DET pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure PC.14 pin: SD_PWR pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Enable the SDIO APB2 Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);

    /* Enable the DMA2 Clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);


}


/*
 * 函数名：SD_PowerON
 * 描述  ：确保SD卡的工作电压和配置控制时钟
 * 输入  ：无
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：在 SD_Init() 调用
 */
STATIC sd_error_enum sd_power_on(void)
{
    SDIO_InitTypeDef SDIO_InitStructure;
    sd_error_enum errorstatus = SD_OK;
    uint32_t response = 0, count = 0;
    bool validvoltage = FALSE;
    uint32_t SDType = SD_STD_CAPACITY;

    /* Power ON Sequence -------------------------------------------------------*/
    /* Configure the SDIO peripheral */
    SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV; /*  SDIOCLK = 48MHz, SDIO_CK = PCLK2/(118 + 2) = 400 KHz */
    SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising ;
    SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
    SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
    SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
    SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
    SDIO_Init(&SDIO_InitStructure);

    /* Set Power State to ON */
    SDIO_SetPowerState(SDIO_PowerState_ON);

    /* Enable SDIO Clock */
    SDIO_ClockCmd(ENABLE);

    /* CMD0: GO_IDLE_STATE -------------------------------------------------------*/
    /* No CMD response required */
    sdio_sendcommand(0x0,SDIO_GO_IDLE_STATE,SDIO_Response_No);

    errorstatus = cmd_error();

    if (errorstatus != SD_OK)
    {
        /* CMD Response TimeOut (wait for CMDSENT flag) */
        goto Escape;    
    }

    /* CMD8: SEND_IF_COND --------------------------------------------------------*/
    /* Send CMD8 to verify SD card interface operating condition */
    /* Argument: - [31:12]: Reserved (shall be set to '0')
    - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
    - [7:0]: Check Pattern (recommended 0xAA) */
    /* CMD Response: R7 */
    sdio_sendcommand(SD_CHECK_PATTERN,SDIO_SEND_IF_COND,SDIO_Response_Short);

    errorstatus = cmd_resp7_error();
    if (errorstatus == SD_OK)
    {
        g_cardtype = SDIO_STD_CAPACITY_SD_CARD_V2_0; /* SD Card 2.0 */
        SDType = SD_HIGH_CAPACITY;
    }
    else
    {
        /* CMD55 */
        sdio_sendcommand(0x00,SDIO_APP_CMD,SDIO_Response_Short);
        errorstatus = cmd_resp1_error(SDIO_APP_CMD);
    }
    /* CMD55 */
    sdio_sendcommand(0x00,SDIO_APP_CMD,SDIO_Response_Short);
    errorstatus = cmd_resp1_error(SDIO_APP_CMD);

    /* If errorstatus is Command TimeOut, it is a MMC card */
    /* If errorstatus is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch)or SD card 1.x */
    if (errorstatus == SD_OK)
    {
        /* SD CARD */
        /* Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
        while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
        {

            /* SEND CMD55 APP_CMD with RCA as 0 */
            sdio_sendcommand(0x00,SDIO_APP_CMD,SDIO_Response_Short);

            errorstatus = cmd_resp1_error(SDIO_APP_CMD);

            if (errorstatus != SD_OK)
            {
                goto Escape;  
            }
            sdio_sendcommand(SD_VOLTAGE_WINDOW_SD | SDType,SDIO_SD_APP_OP_COND,SDIO_Response_Short);

            errorstatus = cmd_resp3_error();
            if (errorstatus != SD_OK)
            {
                goto Escape;  
            }

            response = SDIO_GetResponse(SDIO_RESP1);
            validvoltage = (bool) (((response >> 31) == 1) ? 1 : 0);
            count++;
        }
        if (count >= SD_MAX_VOLT_TRIAL)
        {
            errorstatus = SD_INVALID_VOLTRANGE;
            goto Escape;  
        }

        if (response &= SD_HIGH_CAPACITY)
        {
            g_cardtype = SDIO_HIGH_CAPACITY_SD_CARD;
        }

    }/* else MMC Card */
    /*Error process*/
    Escape:
        return(errorstatus);
}


/*
 * 函数名：SD_InitializeCards
 * 描述  ：初始化所有的卡或者单个卡进入就绪状态
 * 输入  ：无
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：在 SD_Init() 调用
 */
STATIC sd_error_enum sd_initialize_cards(void)
{
    sd_error_enum errorstatus = SD_OK;
    uint16_t rca = 0x01;

    if (SDIO_GetPowerState() == SDIO_PowerState_OFF)
    {
        errorstatus = SD_REQUEST_NOT_APPLICABLE;
        goto Escape;  
    }

    if (SDIO_SECURE_DIGITAL_IO_CARD != g_cardtype)
    {
        /* Send CMD2 ALL_SEND_CID */
        sdio_sendcommand(0x0,SDIO_ALL_SEND_CID,SDIO_Response_Long);

        errorstatus = cmd_resp2_error();

        if (SD_OK != errorstatus)
        {
            goto Escape;  
        }

        g_cidtab[0] = SDIO_GetResponse(SDIO_RESP1);
        g_cidtab[1] = SDIO_GetResponse(SDIO_RESP2);
        g_cidtab[2] = SDIO_GetResponse(SDIO_RESP3);
        g_cidtab[3] = SDIO_GetResponse(SDIO_RESP4);
    }
    if ((   SDIO_STD_CAPACITY_SD_CARD_V1_1 == g_cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == g_cardtype) 
        || (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == g_cardtype) || (SDIO_HIGH_CAPACITY_SD_CARD == g_cardtype))
    {
        /* Send CMD3 SET_REL_ADDR with argument 0 */
        /* SD Card publishes its RCA. */
        sdio_sendcommand(0x0,SDIO_SET_REL_ADDR,SDIO_Response_Short);

        errorstatus = cmd_resp6_error(SDIO_SET_REL_ADDR, &rca);

        if (SD_OK != errorstatus)
        {
            goto Escape;  
        }
    }

    if (SDIO_SECURE_DIGITAL_IO_CARD != g_cardtype)
    {
        g_rca = rca;

        /* Send CMD9 SEND_CSD with argument as card's RCA */
        sdio_sendcommand((uint32_t)(rca << 16),SDIO_SEND_CSD,SDIO_Response_Long);

        errorstatus = cmd_resp2_error();

        if (SD_OK != errorstatus)
        {
            goto Escape;  
            //return(errorstatus);
        }

        g_csdtab[0] = SDIO_GetResponse(SDIO_RESP1);
        g_csdtab[1] = SDIO_GetResponse(SDIO_RESP2);
        g_csdtab[2] = SDIO_GetResponse(SDIO_RESP3);
        g_csdtab[3] = SDIO_GetResponse(SDIO_RESP4);
    }

    errorstatus = SD_OK; /* All cards get intialized */

    /*Error process*/
    Escape:
        return(errorstatus);
}



/*
 * 函数名：SD_Init
 * 描述  ：初始化SD卡，使卡处于就绪状态(准备传输数据)
 * 输入  ：无
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：外部调用
 */
sd_error_enum sd_init(void)
{
    SDIO_InitTypeDef SDIO_InitStructure;
    sd_error_enum errorstatus = SD_OK;

    /*SD Card GPIO init */
    sd_gpio_configuration();

    SDIO_DeInit();
        
    /* SD 卡上电检测 */
    errorstatus = sd_power_on();

    if (errorstatus != SD_OK)
    {
        /* CMD Response TimeOut (wait for CMDSENT flag) */
        goto Escape;
    }   

    /* SD卡识别 */
    errorstatus = sd_initialize_cards();

    if (errorstatus != SD_OK)
    {
        /* CMD Response TimeOut (wait for CMDSENT flag) */
        goto Escape;
    } 
    /* Configure the SDIO peripheral */
    /*  SDIOCLK = 48 MHz, SDIO_CK = PCLK2/(0 + 2) = 24 MHz */  
    SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
    SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
    SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
    SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
    SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
    SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
    SDIO_Init(&SDIO_InitStructure);


    /*Error process*/
    Escape:
        return(errorstatus);
}



