#include "common.h"
#include "tc_common.h"
#include "./led/led_tc.h"
#include "./uart/uart_tc.h"
#include "./sd/sd_tc.h"
//#include "./spi/spi_tc.h"
#include "./button/button_tc.h"
#include "./w25qxx/w25qxx_tc.h"
#include "./flash/flash_tc.h"
#include "./adc/adc_tc.h"
#include "./pwm/pwm_tc.h"

void tc_printf_banner(char *desc)
{
    OS_ERR  err;
    MSG(SYS_INFO "%s started @ %d\r\n",desc,OSTimeGet(&err));
}

void tc_printf_tail(char *desc)
{
    OS_ERR  err;
    MSG(SYS_INFO "%s finished @ %d\r\n",desc,OSTimeGet(&err));
}


void tc_run_all(void)
{
    OS_ERR  err;
    tc_printf_banner("All testing cases");
    //led_tc_run();
    //uart_tc_run();
   // sd_tc_run();
   //	button_tc_run();
   //w25qxx_tc_run();
   //flash_tc_run();
   
	
    tc_printf_tail("All testing cases");
    
    while(1)
    {
    	//button_tc_run();
		//adc_tc_run();
		pwm_tc_run();
        OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

