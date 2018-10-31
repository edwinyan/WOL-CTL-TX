#include "common.h"
#include "pwm_drv.h"
#include "pwm_tc.h"
#include "tc_common.h"
#include "bsp.h"
#include "bsp_os.h"

u32 compare = 0;

STATIC void pwm_tc_01(void)
{

	//compare += 10;

	//if(compare >= 500)
	//	compare =0;
	
    tc_printf_banner("DBG pwm test ");


	TIM_SetCompare1(TIM5,312);

    tc_printf_tail("DBG pwm test ");
}




void pwm_tc_run(void)
{
    pwm_tc_01();

}



