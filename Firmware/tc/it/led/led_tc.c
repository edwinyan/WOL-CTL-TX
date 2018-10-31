#include "common.h"
#include "led_drv.h"
#include "led_tc.h"
#include "tc_common.h"
#include "bsp.h"
#include "bsp_os.h"

STATIC void led_tc_01(void)
{
    uint32_t loop = 20;

    tc_printf_banner("DBG LED 1Hz test ");
    while(loop--)
    {
        LED_R_ON;
        BSP_OS_TimeDly(500);
        LED_R_OFF;
        BSP_OS_TimeDly(500);
    }
    LED_R_ON;
    tc_printf_tail("DBG LED 1Hz test ");
}

STATIC void led_tc_02(void)
{
    uint32_t loop = 20;

    tc_printf_banner("GPS LED 1Hz test ");
    while(loop--)
    {
        LED_G_ON;
        BSP_OS_TimeDly(500);
        LED_G_OFF;
        BSP_OS_TimeDly(500);
    }
    LED_G_ON;
    tc_printf_tail("GPS LED 1Hz test ");
}

STATIC void led_tc_03(void)
{
    uint32_t loop = 20;

    tc_printf_banner("RDY LED 1Hz test ");
    while(loop--)
    {
        LED_B_ON;
        BSP_OS_TimeDly(500);
        LED_B_OFF;
        BSP_OS_TimeDly(500);
    }
    LED_B_ON;
    tc_printf_tail("RDY LED 1Hz test ");
}


void led_tc_run(void)
{
    led_tc_01();
    led_tc_02();
    led_tc_03();
}



