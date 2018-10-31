#include "common.h"
#include "stm32f4xx_gpio.h"
#include "watchdog_drv.h"

/*----------------------------------------------------------------------------*/

//初始化独立看门狗
//prer:分频系数:0~7
//rlr:自动重装载值,0~0XFFF.
//分频因子 =4*2^prer.最大值256!
//rlr:重装载值: 低11位有效
//时间计算:Tout=((4*2^prer)*rlr)/32 (ms).

void IWDG_Init(u8 prer)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 
	
	IWDG_SetPrescaler(prer);

	IWDG_SetReload(500);  
	IWDG_ReloadCounter(); 
	
	IWDG_Enable();
}

//喂狗
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}

