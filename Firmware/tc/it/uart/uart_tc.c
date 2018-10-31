#include "common.h"
#include "uart_tc.h"
#include "bsp.h"
#include "bsp_os.h"
#include "tc_common.h"
#include "uart_drv.h"

STATIC void uart_tc_01(void)
{
    uint32_t loop = 100;
    float f = (float)100.0;

    tc_printf_banner("Uart Dbg printf test ");
    
    f = f/(float)3.0;
    MSG("float test: %f\r\n",f);
    while(loop--)
    {
        MSG("loop: %d\r\n",loop);
    }
    tc_printf_tail("Uart Dbg printf test ");
}

STATIC void uart_tc_02(void)
{   
    u8 read;
    
    tc_printf_banner("Uart Dbg echo test (Press 'Q' to exit) ");
    while(1)
    {
        if(1 == uart_drv_dbg_recv(&read,2))
        {
            MSG("%c",read);
            if('Q' == read)
            {
                break;
            }
        }
    }
    tc_printf_tail("Uart Dbg echo test ");
}

STATIC void uart_tc_03(void)
{
    
}


void uart_tc_run(void)
{
    uart_tc_01();
    uart_tc_02();
    uart_tc_03();
}

