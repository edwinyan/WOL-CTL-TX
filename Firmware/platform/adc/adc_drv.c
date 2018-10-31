#include "common.h"
#include "stm32f4xx_gpio.h"
#include "adc_drv.h"
#include "bsp.h"
#include "bsp_os.h"

#include "uart_drv.h"


__IO u16 Adc_Value[64][ADC_NUM_CHANNELS];
u32 test_flag=0;
u16 adc_avg[ADC_NUM_CHANNELS];
/*----------------------------------------------------------------------------*/
void adc_filter(void)
{
	u8 i;
	u8 j;
	u32 sum[ADC_NUM_CHANNELS]={0};

	for(i=0;i<ADC_NUM_CHANNELS;i++)
	{
		for(j=0;j<64;j++)
		{
			sum[i] += Adc_Value[j][i];
		}
		adc_avg[i] = sum[i]/64;
		sum[i] = 0;
	}
}

void DMA2_Stream0_IRQHandler()
{
	OSIntEnter();
	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0) != RESET){
		//test_flag++;
		adc_filter();
		DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
	}

	OSIntExit();
}




void Adc_DMA_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	while(DMA_GetCmdStatus(DMA2_Stream0)!=DISABLE);

	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Adc_Value;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_NUM_CHANNELS*64;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;/*FIFO的值*/
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;/*单次传输*/
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC , ENABLE);

	MSG("adc dma config complete\r\n");
}

void  Adc_Init(void)
{    
  	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	Adc_DMA_init();
	
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = ADC1_IN0_PIN | ADC1_IN6_PIN | ADC1_IN7_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ADC1_IN8_PIN | ADC1_IN9_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
	ADC_InitStructure.ADC_NbrOfConversion = ADC_NUM_CHANNELS;
	ADC_Init(ADC1, &ADC_InitStructure);

	// Enable and set EXTI1 Interrupt to the lowest priority
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	BSP_IntVectSet(BSP_INT_ID_DMA2_CH0,DMA2_Stream0_IRQHandler);
	BSP_IntEn(BSP_INT_ID_DMA2_CH0);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_56Cycles );    
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_56Cycles );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_56Cycles );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_56Cycles );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 5, ADC_SampleTime_56Cycles );

	
	
    ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE); 
    ADC_DMACmd(ADC1, ENABLE); 
	ADC_Cmd(ADC1, ENABLE);

	ADC_SoftwareStartConv(ADC1);

	MSG("adc init complete\r\n");

}				  



/*----------------------------------------------------------------------------*/
u16 adc_getvalue(u8 num)
{
	return adc_avg[num];
}



