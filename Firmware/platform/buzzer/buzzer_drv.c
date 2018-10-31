#include "common.h"
#include "stm32f4xx_gpio.h"
#include "buzzer_drv.h"

/*----------------------------------------------------------------------------*/

typedef struct{
    GPIO_TypeDef *port;
    u16         pin;
}buzzer_config_t;


STATIC buzzer_config_t buzzer_config_array[BUZZER_SRC_NUM] = {
    {GPIOA, GPIO_Pin_11},       
};


/*----------------------------------------------------------------------------*/
//global functions
void buzzer_drv_init(void)
{
    GPIO_InitTypeDef  gpio_init;
    buzzer_src_enum src;
    
    gpio_init.GPIO_Mode  = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

    for(src = BUZZER_SRC_START; src < BUZZER_SRC_NUM; src++)
    {
        gpio_init.GPIO_Pin = buzzer_config_array[src].pin;
        GPIO_Init(buzzer_config_array[src].port, &gpio_init);
    }

	buzzer(0);
}


void buzzer(u8 state)
{
	if(state){
		GPIO_SetBits(BUZZER_GPIO,BUZZER_PIN);
	}else{
		GPIO_ResetBits(BUZZER_GPIO,BUZZER_PIN);
	}
}







/*----------------------------------------------------------------------------*/



