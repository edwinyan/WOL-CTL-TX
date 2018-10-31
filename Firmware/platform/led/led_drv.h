#ifndef _LED_DRV_H_
#define _LED_DRV_H_
//#include "binding.h" 

typedef enum{
    LED_SRC_START = 0,
    LED_SRC_R = LED_SRC_START,
    LED_SRC_G,
    LED_SRC_B,

    LED_SRC_NUM
}led_src_enum;

void led_drv_init(void);
void led_on(led_src_enum src);
void led_off(led_src_enum src);


#define LED_R_ON      led_on(LED_SRC_R)
#define LED_G_ON      led_on(LED_SRC_G)
#define LED_B_ON      led_on(LED_SRC_B)

#define LED_R_OFF     led_off(LED_SRC_R)
#define LED_G_OFF     led_off(LED_SRC_G)
#define LED_B_OFF     led_off(LED_SRC_B)
#if (TX==1)
#define LED_W_ON      led_on(LED_SRC_W)
#define LED_W_OFF      led_off(LED_SRC_W)
#endif
#endif
