#ifndef _PWM_DRV_H_
#define _PWM_DRV_H_

#define TIM3_CH1_GPIO	GPIOA
#define TIM3_CH1_PIN	GPIO_Pin_6
#define TIM3_CH1_PINSOURCE	GPIO_PinSource6

#define TIM3_CH2_GPIO	GPIOA
#define TIM3_CH2_PIN	GPIO_Pin_7
#define TIM3_CH2_PINSOURCE	GPIO_PinSource7

#define TIM3_CH3_GPIO	GPIOB
#define TIM3_CH3_PIN	GPIO_Pin_0
#define TIM3_CH3_PINSOURCE	GPIO_PinSource0

#define TIM3_CH4_GPIO	GPIOB
#define TIM3_CH4_PIN	GPIO_Pin_1
#define TIM3_CH4_PINSOURCE	GPIO_PinSource1

#define TIM5_CH1_GPIO	GPIOA
#define TIM5_CH1_PIN	GPIO_Pin_0
#define TIM5_CH1_PINSOURCE	GPIO_PinSource0

#define PWM_CHANNEL_CAMERA	0		//camera adc channel
#define PWM_CHANNEL_JS_R1	1		//joystick right channel1
#define	PWM_CHANNEL_JS_R2	2		//joystick right channel2
#define PWM_CHANNEL_JS_L1	3		//joystick left channel1
#define	PWM_CHANNEL_JS_L2	4		//joystick left channel2

#define PWM_MEDIUM_VALUE	2048	//“°∏À÷–Œª÷µ


void TIM3_PWM_Init(u32 arr,u32 psc);
void TIM5_PWM_Init(u32 arr,u32 psc);

#endif
