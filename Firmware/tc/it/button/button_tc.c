#include "common.h"
#include "button_drv.h"
#include "button_tc.h"
#include "tc_common.h"
#include "bsp.h"
#include "bsp_os.h"

#if(TX == 1)
STATIC void button_tc_01(void)
{
	OS_ERR      err;
	
    tc_printf_banner("DBG button test ");

	if(button_value_get(BUTTON_SRC_PAIR) == 0)
	{
		OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_HMSM_STRICT, &err);
		if(button_value_get(BUTTON_SRC_PAIR) == 0) //È¥¶¶¶¯
		{
			MSG("pair code button pressed");
		}
		
	}

    tc_printf_tail("DBG button test ");
}

STATIC void button_tc_02(void)
{
//	OS_ERR      err;
	//u8 button_high = 0;
	//u8 button_low =0;
	u8 count;
	u8 button[BUTTON_SRC_NUM]={0};
	
    tc_printf_banner("DBG button test ");

	//for(count=1;count<BUTTON_SRC_NUM;count++)
	button[0] = button_value_get(BUTTON_SRC_SHUTTLE2);
	button[1] = button_value_get(BUTTON_SRC_SHUTTLE1);
	button[2] = button_value_get(BUTTON_SRC_SHUTTLE);
	button[3] = button_value_get(BUTTON_SRC_PHOTO);
	button[4] = button_value_get(BUTTON_SRC_PLAYBACK);
	button[5] = button_value_get(BUTTON_SRC_MODE_SET1);
	button[6] = button_value_get(BUTTON_SRC_MODE_SET2);
	button[7] = button_value_get(BUTTON_SRC_RETURN_RESERVE2);
	button[8] = button_value_get(BUTTON_SRC_RETURN_TOGGLE);
	button[9] = button_value_get(BUTTON_SRC_RETURN_RESERVE1);
	button[10] = button_value_get(BUTTON_SRC_RETURN);
	//button[11] = button_value_get(BUTTON_SRC_RETURN_LED);
	//button[12] = button_value_get(BUTTON_SRC_POWER_LED2);
	//button[13] = button_value_get(BUTTON_SRC_POWER_LED1);
	button[11] = button_value_get(BUTTON_SRC_POWER);
	button[12] = button_value_get(BUTTON_SRC_VIDEO);
	for(count=0;count<BUTTON_SRC_NUM-1;count++)
	{
		MSG("%d,",button[count]);
	}
	MSG("\r\n");
    tc_printf_tail("DBG button test ");
}


void button_tc_run(void)
{
    button_tc_01();
	button_tc_02();
}

#endif

