#include "common.h"
#include "adc_drv.h"
#include "adc_tc.h"
#include "tc_common.h"
#include "bsp.h"
#include "bsp_os.h"

//extern __IO u16 Adc_Value[ADC_NUM_CHANNELS];

STATIC void adc_tc_01(void)
{
	#if 0
	OS_ERR      err;
	//u16 adc[5];
	u8 num,i;

    tc_printf_banner("DBG adc test ");
	for(num=0;num<ADC_NUM_CHANNELS;num++)
	{
		for(i=0;i<16;i++)
		{
			adcConverted[i] += Adc_Value[i][num];
		}
		adcConverted[i] = adcConverted[i]/16;
		//
	}
	for(i=0;i<ADC_NUM_CHANNELS;i++)
	{
		MSG("adc%d=%d,",i,(u16)adcConverted[i]);
	}
	MSG("\r\n");
#else
	if(DMA_GetFlagStatus(DMA2_Stream0,DMA_FLAG_TCIF0)!=RESET)/*传输完成*/
	{
#if 0
		MSG("adc1 = %d\r\n",Adc_Value[0]);
		MSG("adc2 = %d\r\n",Adc_Value[1]);
		MSG("adc3 = %d\r\n",Adc_Value[2]);
		MSG("adc4 = %d\r\n",Adc_Value[3]);
		MSG("adc5 = %d\r\n",Adc_Value[4]);
#else
		MSG("adc1 = %d\r\n",adc_getvalue(0));
		MSG("adc2 = %d\r\n",adc_getvalue(1));
		MSG("adc3 = %d\r\n",adc_getvalue(2));
		MSG("adc4 = %d\r\n",adc_getvalue(3));
		MSG("adc5 = %d\r\n",adc_getvalue(4));
#endif
		DMA_ClearFlag(DMA2_Stream0,DMA_FLAG_TCIF0);/*清零*/
	}

#endif
    tc_printf_tail("DBG adc test ");
}


void adc_tc_run(void)
{
    //button_tc_01();
	adc_tc_01();
}



