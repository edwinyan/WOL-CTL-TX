#include "common.h"
#include "bsp.h"
#include "bsp_os.h"
#include "uart_drv.h"
#include "adc_drv.h"
#include "string.h"
#include "datalink_drv.h"
#include "button_drv.h"
#include "led_drv.h"
#include "pwm_drv.h"
#include "buzzer_drv.h"

#define ADC_COMPENSATE	(3.304f/3.274f)   //发送端(3.276v)和接收端(3.304v)的adc参考电压不一致，需要做补偿
#define TX_HEAD	0xAA
#define TX_TAIL	0x55
#define MAX_PACKETSIZE 16		//we use 16bytes for TX module send buffer

u8 tx_buf[MAX_PACKETSIZE]={0};

static vu8 stamp=0;
bool connected=FALSE;
static bool connected_state=FALSE;	//是否连接过
extern FIFO_T stFiFo;

void datalink_recv(void)
{
	//u8 data_len=0;
	u8 read;
	static u8 buzzer_count=0;
	//u8 i=0;
	
	if(Fifo_DataLen(&stFiFo))
	{
		//	data_len = Fifo_DataLen(&stFiFo);
		if(Fifo_Read(&stFiFo,&read) == TRUE && read == 'R'){
			if(Fifo_Read(&stFiFo,&read) == TRUE && read == 'S'){
				if(Fifo_Read(&stFiFo,&read) == TRUE && read == 'S'){
					if(Fifo_Read(&stFiFo,&read) == TRUE && read == 'I'){
						if(Fifo_Read(&stFiFo,&read) == TRUE && read == '='){
							//we got a heart beat packet
							//while(data_len --)
							//	Fifo_Read(&stFiFo,&read);
							stamp = 25;
							connected= TRUE;
							if(connected_state == FALSE ){
								//connected_state = TRUE;
								if(buzzer_count < 4){
									buzzer(1);
									buzzer_count++;
								}else{
									connected_state = TRUE;
									buzzer_count=0;
								}
							}
							LED_W_ON;
						}
					}
				}
			}
		}
	}
}

u8 getLB2Values(u8 state)
{
	u8 ret=0;

	if(state == 1)
	{
		if(button_value_get(BUTTON_SRC_SHUTTLE2) == 1){
			ret |= SHUTTLE_ADJ2;
		}else{
			ret &= ~SHUTTLE_ADJ2;
		}
		if(button_value_get(BUTTON_SRC_SHUTTLE1) == 1){
			ret |= SHUTTLE_ADJ1;
		}else{
			ret &= ~SHUTTLE_ADJ1;
		}
		if(button_value_get(BUTTON_SRC_SHUTTLE) == 1){
			ret |= SHUTTLE_BUTTON;
		}else{
			ret &= ~SHUTTLE_BUTTON;
		}
		if(button_value_get(BUTTON_SRC_PHOTO) == 1){
			ret |= PHOTO_BUTTON;
		}else{
			ret &= ~PHOTO_BUTTON;
		}
		if(button_value_get(BUTTON_SRC_PLAYBACK) == 1){
			ret |= PLAYBACK_BUTTON;
		}else{
			ret &= ~PLAYBACK_BUTTON;
		}
		if(button_value_get(BUTTON_SRC_MODE_SET1) == 1){
			ret |= MODE_SET1;
		}else{
			ret &= ~MODE_SET1;
		}
		if(button_value_get(BUTTON_SRC_MODE_SET2) == 1){
			ret |= MODE_SET2;
		}else{
			ret &= ~MODE_SET2;
		}
		if(button_value_get(BUTTON_SRC_VIDEO) == 1){
			ret |= VIDEO_BUTTON;
		}else{
			ret &= ~VIDEO_BUTTON;
		}
	}
	if(state == 0){
		if(button_value_get(BUTTON_SRC_POWER) == 1){
			//ret |= POWER_BUTTON;
			ret &= ~POWER_BUTTON;
		}else{
			//ret &= ~POWER_BUTTON;
			ret |= POWER_BUTTON;
		}
		if(button_value_get(BUTTON_SRC_RETURN_RESERVE2) == 1){
			ret |= RETURN_RESERVE2;
		}else{
			ret &= ~RETURN_RESERVE2;
		}
		if(button_value_get(BUTTON_SRC_RETURN_TOGGLE) == 1){
			ret |= RETURN_TOGGLE;
		}else{
			ret &= ~RETURN_TOGGLE;
		}
		if(button_value_get(BUTTON_SRC_RETURN_RESERVE1) == 1){
			ret |= RETURN_RESERVE1;
		}else{
			ret &= ~RETURN_RESERVE1;
		}
		if(button_value_get(BUTTON_SRC_RETURN) == 1){
			ret |= RETURN_BUTTON;
		}else{
			ret &= ~RETURN_BUTTON;
		}

		ret &= 0x1F;   //we only use bit4-0
	}
	return ret;
}

u8 checkbufferdata(u8 *data,u8 size)
{
	u8 ret;
	u16 sum=0;

	while(size--)
	{
		sum += *(++data);
	}

	ret = sum %256;

	return ret;
}


void packChannels(void)
{
	u32 counter;
	u16 adcTemp[5];
//	u8 *temp;

	static u8 number=0;
	

	adcTemp[ADC_CHANNEL_CAMERA] = (4095 - adc_getvalue(ADC_CHANNEL_CAMERA))*ADC_COMPENSATE;
	adcTemp[ADC_CHANNEL_JS_R1] = (4095- adc_getvalue(ADC_CHANNEL_JS_R1))*ADC_COMPENSATE;
	adcTemp[ADC_CHANNEL_JS_R2] = (4095 - adc_getvalue(ADC_CHANNEL_JS_R2))*ADC_COMPENSATE;
	adcTemp[ADC_CHANNEL_JS_L1] = (4095 - adc_getvalue(ADC_CHANNEL_JS_L1))*ADC_COMPENSATE;
	adcTemp[ADC_CHANNEL_JS_L2] = (4095 - adc_getvalue(ADC_CHANNEL_JS_L2))*ADC_COMPENSATE;

	#if 0
		for(counter=0;counter<5;counter++)
			MSG("%d,", adcTemp[counter]);
		MSG("\r\n");
	#endif

	//adc 输入输出映射
	adcTemp[ADC_CHANNEL_JS_R1] = adcTemp[ADC_CHANNEL_JS_R1]*1.96f - 2000;
	adcTemp[ADC_CHANNEL_JS_R2] = adcTemp[ADC_CHANNEL_JS_R2]*1.96f - 2000;
	adcTemp[ADC_CHANNEL_JS_L1] = adcTemp[ADC_CHANNEL_JS_L1]*1.96f - 2000;
	adcTemp[ADC_CHANNEL_JS_L2] = adcTemp[ADC_CHANNEL_JS_L2]*1.96f - 2000;
	
	tx_buf[0] =TX_HEAD;
	tx_buf[1] = adcTemp[ADC_CHANNEL_CAMERA]>>8;
	tx_buf[2] = adcTemp[ADC_CHANNEL_CAMERA];
	tx_buf[3] = adcTemp[ADC_CHANNEL_JS_R1]>>8;
	tx_buf[4] = adcTemp[ADC_CHANNEL_JS_R1];
	tx_buf[5] = adcTemp[ADC_CHANNEL_JS_R2]>>8;
	tx_buf[6] = adcTemp[ADC_CHANNEL_JS_R2];
	tx_buf[7] = adcTemp[ADC_CHANNEL_JS_L1]>>8;
	tx_buf[8] = adcTemp[ADC_CHANNEL_JS_L1];
	tx_buf[9] = adcTemp[ADC_CHANNEL_JS_L2]>>8;
	tx_buf[10] = adcTemp[ADC_CHANNEL_JS_L2];
	tx_buf[11] = getLB2Values(1);
	tx_buf[12] = getLB2Values(0);
	tx_buf[13] = number++; //发送数据包序号
	tx_buf[14] = checkbufferdata(tx_buf,13);
	tx_buf[MAX_PACKETSIZE-1] = TX_TAIL;
	#if 1
	for(counter=0;counter<MAX_PACKETSIZE;counter++){
		if(tx_buf[counter] == 0x0A)
			tx_buf[counter] = 0xFF - tx_buf[counter];
	}
	//MSG("\r\n");
	#endif
}


void datalink_send(void)
{
	packChannels();
	uart_drv_dbg_msg(tx_buf,MAX_PACKETSIZE);
}

void datalink_state(void)
{
	u8 buzzer_rate=0;	//蜂鸣器频率控制标志位，时基100ms
	
	if(stamp > 0){
		stamp --;
	}else{
		connected = FALSE;
		LED_W_OFF;
	}

	if(connected == FALSE && connected_state == TRUE)
	{
		buzzer_rate++;
		if(buzzer_rate == 20)
		{
			buzzer(1);
			buzzer_rate = 0;
		}
	}
}

