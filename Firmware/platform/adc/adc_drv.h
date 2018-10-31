#ifndef _ADC_DRV_H_
#define _ADC_DRV_H_

#define ADC1_IN0_PIN	GPIO_Pin_0
#define ADC1_IN6_PIN	GPIO_Pin_6
#define ADC1_IN7_PIN	GPIO_Pin_7

#define ADC1_IN8_PIN	GPIO_Pin_0
#define ADC1_IN9_PIN	GPIO_Pin_1

#define ADC_NUM_CHANNELS	5

#define ADC_CHANNEL_CAMERA	0		//camera adc channel
#define ADC_CHANNEL_JS_R1	1		//joystick right channel1
#define	ADC_CHANNEL_JS_R2	2		//joystick right channel2
#define ADC_CHANNEL_JS_L1	3		//joystick left channel1
#define	ADC_CHANNEL_JS_L2	4		//joystick left channel2

void  Adc_Init(void);
u16 adc_getvalue(u8 num);



#endif
