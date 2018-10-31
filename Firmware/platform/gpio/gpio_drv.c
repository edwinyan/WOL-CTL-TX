#include "common.h"
#include "stm32f4xx_gpio.h"
#include "gpio_drv.h"

/*----------------------------------------------------------------------------*/

typedef struct{
    GPIO_TypeDef *port;
    u16         pin;
}gpio_config_t;


STATIC gpio_config_t gpio_config_array[GPIO_SRC_NUM] = {
    {GPIOC, GPIO_Pin_0},    //shuttle adj 2
    {GPIOC, GPIO_Pin_1},	//shuttle adj 1
    {GPIOC, GPIO_Pin_2},	//shuttle
    {GPIOA, GPIO_Pin_1}, 	//photo
    {GPIOA, GPIO_Pin_2}, 	//playback
    {GPIOA, GPIO_Pin_4}, 	//mode set 1
    {GPIOA, GPIO_Pin_5}, 	//mode set 2
    {GPIOB, GPIO_Pin_15}, 	//video

	{GPIOA, GPIO_Pin_8}, 	//power
	{GPIOB, GPIO_Pin_5}, 	//return reserve2
	{GPIOB, GPIO_Pin_6}, 	//return toggle
	{GPIOB, GPIO_Pin_7}, 	//return reserve1
	{GPIOB, GPIO_Pin_8}, 	//return 

};

/*----------------------------------------------------------------------------*/
//global functions
void gpio_drv_init(void)
{
    GPIO_InitTypeDef  gpio_init;
    gpio_src_enum src;
    
    gpio_init.GPIO_Mode  = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

    for(src = GPIO_SRC_START; src < GPIO_SRC_NUM; src++)
    {
        gpio_init.GPIO_Pin = gpio_config_array[src].pin;
        GPIO_Init(gpio_config_array[src].port, &gpio_init);
    }

}
#if 0
//config rx led status output
void tx_gpio_init(void)
{
	GPIO_InitTypeDef  gpio_init;
	u8 counter;

	gpio_init.GPIO_Mode  = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

	 for(counter = 0; counter < 3; counter++)
    {
        gpio_init.GPIO_Pin = gpio_led_array[counter].pin;
        GPIO_Init(gpio_led_array[counter].port, &gpio_init);
    }
}

//config tx led status input
void rx_gpio_init(void)
{
	GPIO_InitTypeDef  gpio_init;
	u8 counter;

	gpio_init.GPIO_Mode  = GPIO_Mode_IN;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd  = GPIO_PuPd_DOWN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

	 for(counter = 0; counter < 3; counter++)
    {
        gpio_init.GPIO_Pin = gpio_led_array[counter].pin;
        GPIO_Init(gpio_led_array[counter].port, &gpio_init);
    }
}
#endif

void gpio_value_set(u8 src)
{
    ASSERT_D(src < GPIO_SRC_NUM);
    GPIO_SetBits(gpio_config_array[src].port, gpio_config_array[src].pin);
}

void gpio_value_reset(u8 src)
{
    ASSERT_D(src < GPIO_SRC_NUM);
    GPIO_ResetBits(gpio_config_array[src].port, gpio_config_array[src].pin);
}

