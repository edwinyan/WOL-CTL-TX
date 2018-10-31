#ifndef _BUZZER_DRV_H_
#define _BUZZER_DRV_H_

typedef enum{
    BUZZER_SRC_START = 0,
    BUZZER_SRC_FRIST = BUZZER_SRC_START,    //·äÃùÆ÷

    BUZZER_SRC_NUM
}buzzer_src_enum;

#define BUZZER_GPIO		GPIOA
#define BUZZER_PIN		GPIO_Pin_11

void buzzer_drv_init(void);
void buzzer(u8 state);

#endif
