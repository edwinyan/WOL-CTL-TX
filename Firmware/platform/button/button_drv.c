#include "common.h"
#include "stm32f4xx_gpio.h"
#include "button_drv.h"

/*----------------------------------------------------------------------------*/

typedef struct{
    GPIO_TypeDef *port;
    u16         pin;
	GPIOPuPd_TypeDef pupd;
}button_config_t;

#if (TX == 1)
STATIC button_config_t button_config_array[BUTTON_SRC_NUM] = {
    {GPIOA, GPIO_Pin_12, GPIO_PuPd_UP},	//pair buttion
		
	{GPIOC, GPIO_Pin_0, GPIO_PuPd_UP},  //shuttle adj2
	{GPIOC, GPIO_Pin_1, GPIO_PuPd_UP},	//shuttle adj1
	{GPIOC, GPIO_Pin_2, GPIO_PuPd_UP},	//shuttle button	
	{GPIOA, GPIO_Pin_1, GPIO_PuPd_UP},	//photo button
	{GPIOA, GPIO_Pin_2, GPIO_PuPd_UP},	//playback

	{GPIOA, GPIO_Pin_4, GPIO_PuPd_UP},	//mode set 1
	{GPIOA, GPIO_Pin_5, GPIO_PuPd_UP},	//mode set 2
	{GPIOB, GPIO_Pin_15, GPIO_PuPd_UP},	//video button

	{GPIOA, GPIO_Pin_8, GPIO_PuPd_UP},	//power button		
	{GPIOB, GPIO_Pin_5, GPIO_PuPd_UP},	//return reserve 2
	{GPIOB, GPIO_Pin_6, GPIO_PuPd_UP},	//return toggle
	{GPIOB, GPIO_Pin_7, GPIO_PuPd_UP},	//return reserve 1
	{GPIOB, GPIO_Pin_8, GPIO_PuPd_UP},	//return button


	
};
#else
STATIC button_config_t button_config_array[BUTTON_SRC_NUM] = {
    {GPIOC, GPIO_Pin_8, GPIO_PuPd_UP},
	{GPIOC, GPIO_Pin_9, GPIO_PuPd_UP},
	{GPIOB, GPIO_Pin_9, GPIO_PuPd_UP},
};
#endif
	
/*----------------------------------------------------------------------------*/
//global functions
void button_drv_init(void)
{
    GPIO_InitTypeDef  gpio_init;
    button_src_enum src;
    
    gpio_init.GPIO_Mode  = GPIO_Mode_IN;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    //gpio_init.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

    for(src = BUTTON_SRC_START; src < BUTTON_SRC_NUM; src++)
    {
        gpio_init.GPIO_Pin = button_config_array[src].pin;
		gpio_init.GPIO_PuPd = button_config_array[src].pupd;
        GPIO_Init(button_config_array[src].port, &gpio_init);
    }
}


u8 button_value_get(button_src_enum src)
{
    ASSERT_D(src < BUTTON_SRC_NUM);
    return GPIO_ReadInputDataBit(button_config_array[src].port, button_config_array[src].pin);
}


/*----------------------------------------------------------------------------*/



