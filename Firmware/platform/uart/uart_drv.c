#include "common.h"
#include "stm32f4xx_gpio.h"
#include "serial.h"
#include "serial_bsp_stm3240x.h"
#include "serial_line_tty.h"
#include "uart_drv.h"

extern OS_MUTEX	FIFO_MUTEX;

enum{
    UART_PIN_TYPE_TX = 0,
    UART_PIN_TYPE_RX,

    UART_PIN_TYPE_NUM
};

typedef struct{
    char            *uart_name;
    SERIAL_DEV_CFG  *uart_dev_cfg;
    struct  serial_if_cfg uart_if_cfg;
    SERIAL_IF_NBR   uart_if_nbr;    

    GPIO_TypeDef    *port[UART_PIN_TYPE_NUM]; // tx/rx
    u16             pin[UART_PIN_TYPE_NUM];  // tx/rx  
    u16             pin_src[UART_PIN_TYPE_NUM];
    u8              uart_af_nbr;
    u8              uart_enabled;
}uart_drv_t;

uart_drv_t uart_drv_array[UART_SRC_NUM] = {
    //uart1 for debug
    { 
        "USART1", &SerialDevCfg_STM32_USART1, 
        {SERIAL_BAUDRATE_115200, SERIAL_DATABITS_8, SERIAL_STOPBITS_1, SERIAL_PARITY_NONE, SERIAL_FLOW_CTRL_NONE}, 
        0,{GPIOA, GPIOA},{GPIO_Pin_9,GPIO_Pin_10}, {GPIO_PinSource9, GPIO_PinSource10},GPIO_AF_USART1,DEF_FALSE
    },

    //endmark
    { 
        "", NULL, 
        {SERIAL_BAUDRATE_115200, SERIAL_DATABITS_8, SERIAL_STOPBITS_1, SERIAL_PARITY_NONE, SERIAL_FLOW_CTRL_NONE}, 
        0,{GPIOA, GPIOA},{GPIO_Pin_9,GPIO_Pin_10}, {GPIO_PinSource9, GPIO_PinSource10},GPIO_AF_USART1,DEF_FALSE
    },
};



void uart_drv_init(void)
{
    uart_src_enum src;
    SERIAL_ERR     err;
    uart_drv_t     *uart_drv;
    GPIO_InitTypeDef  GPIO_InitStructure;

    //gpio config
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    
    
    //serial init
    Serial_Init(); 
    
    for(src = UART_SRC_START; src < UART_SRC_NUM; src++)
    {
        uart_drv = &uart_drv_array[src];
        
        if(Str_Len(uart_drv->uart_name))
        {
            GPIO_PinAFConfig(uart_drv->port[UART_PIN_TYPE_TX], uart_drv->pin_src[UART_PIN_TYPE_TX], uart_drv->uart_af_nbr);
            GPIO_InitStructure.GPIO_Pin = uart_drv->pin[UART_PIN_TYPE_TX];
            GPIO_Init(uart_drv->port[UART_PIN_TYPE_TX], &GPIO_InitStructure);
            
            GPIO_PinAFConfig(uart_drv->port[UART_PIN_TYPE_RX], uart_drv->pin_src[UART_PIN_TYPE_RX], uart_drv->uart_af_nbr);
            GPIO_InitStructure.GPIO_Pin = uart_drv->pin[UART_PIN_TYPE_RX];
            GPIO_Init(uart_drv->port[UART_PIN_TYPE_RX], &GPIO_InitStructure);

    
            Serial_DevDrvAdd((CPU_CHAR       *)uart_drv->uart_name,     
                     (SERIAL_DEV_CFG *) uart_drv->uart_dev_cfg,
                     (CPU_SIZE_T      ) 52u,
                     (CPU_SIZE_T      ) 52u,
                     (SERIAL_ERR     *)&err);

            ASSERT_R(err == SERIAL_ERR_NONE);
            if(err != SERIAL_ERR_NONE)
            {
                continue;
            }
            uart_drv->uart_if_nbr = Serial_Open((CPU_CHAR      *)uart_drv->uart_name,
                                     (SERIAL_IF_CFG *)&uart_drv->uart_if_cfg,
                                     (SERIAL_ERR    *)&err);
            ASSERT_R(err == SERIAL_ERR_NONE);

            if(err != SERIAL_ERR_NONE)
            {
                continue;
            }
            
            uart_drv->uart_enabled = DEF_TRUE;
        }
        else
        {
            break;
        }
    }
#if 0
    uart_drv = &uart_drv_array[UART_SRC_DBG];
    ASSERT_R(DEF_TRUE == uart_drv->uart_enabled);
    
    Serial_SetLineDrv((SERIAL_IF_NBR        ) uart_drv->uart_if_nbr,
                      (SERIAL_LINE_DRV_API *)&SerialLine_TTY ,
                      (SERIAL_ERR          *)&err);
    
    Serial_Wr((SERIAL_IF_NBR   )uart_drv->uart_if_nbr,
              (void           *)"\n\n",
              (CPU_SIZE_T      )2,
              (CPU_INT32U      )0,
              (SERIAL_ERR     *)&err);
    
    ASSERT_R(err == SERIAL_ERR_NONE);
#endif
}

void uart_drv_dbg_msg(u8 *msg)
{
    SERIAL_ERR  err;
    uart_drv_t *uart_drv = &uart_drv_array[UART_SRC_DBG];
    ASSERT_R(DEF_TRUE == uart_drv->uart_enabled);
    
    Serial_Wr((SERIAL_IF_NBR   )uart_drv->uart_if_nbr,
              (void           *)&msg[0],
              (CPU_SIZE_T      )Str_Len((const CPU_CHAR *)msg),
              (CPU_INT32U      )0,
              (SERIAL_ERR     *)&err);
    ASSERT_R(err == SERIAL_ERR_NONE);
}

u32 uart_drv_dbg_recv(u8 *buf, u32 len)
{
    u32 read_len;
    SERIAL_ERR  err;
    uart_drv_t *uart_drv = &uart_drv_array[UART_SRC_DBG];
    ASSERT_R(DEF_TRUE == uart_drv->uart_enabled);
    
    read_len = Serial_Rd((SERIAL_IF_NBR   )uart_drv->uart_if_nbr,
              (void           *)buf,
              (CPU_SIZE_T      )len,
              (CPU_INT32U      )0,
              (SERIAL_ERR     *)&err);
   
    if(err != SERIAL_ERR_NONE)
    {
        MSG_INFO("%s err: status: 0x%x, len: 0x%x\r\n", __FUNCTION__,err,len);
        //ASSERT_R(err == SERIAL_ERR_NONE);
    }
    
    return read_len;
}

void Fifo_Init(pFIFO_T stFiFo)
{
	stFiFo->r_idx = 0;
	stFiFo->w_idx = 0;
		
	return;
}

bool Fifo_Write(pFIFO_T stFiFo, u8 dat)
{
	bool ret = TRUE;
	OS_ERR		err;

	OSMutexPend (&FIFO_MUTEX,0,OS_OPT_PEND_BLOCKING,0,&err);

	/* Save it to buffer */
	if (((stFiFo->w_idx + 1) % UART_BUFFER_LEN) == stFiFo->r_idx) {//Check if the fifo is full
		/* Adjust read index since buffer is full */
		/* Keep the latest one and drop the oldest one */
		stFiFo->r_idx = (stFiFo->r_idx + 1) % UART_BUFFER_LEN;
		MSG_INFO("%s err, fifo full\r\n", __FUNCTION__);
		ret = FALSE;//cout<< "buffer full\n";
	}
	
	stFiFo->buf[stFiFo->w_idx] = dat; //Write fifo
	stFiFo->w_idx = (stFiFo->w_idx + 1) % UART_BUFFER_LEN;//adjust w_idx
	
	OSMutexPost(&FIFO_MUTEX,OS_OPT_POST_NONE,&err);

	return ret;
}
 
bool Fifo_Read(pFIFO_T stFiFo,u8 *dat)
{
	bool ret = TRUE;
	OS_ERR      err;
	

	OSMutexPend (&FIFO_MUTEX,0,OS_OPT_PEND_BLOCKING,0,&err);

	if ((stFiFo->r_idx == stFiFo->w_idx)){
		MSG_INFO("%s err, fifo empty\r\n", __FUNCTION__);
		ret = FALSE;//cout << "buffer empty\n";
	}else
	{
		*dat = stFiFo->buf[stFiFo->r_idx];
		stFiFo->r_idx = (stFiFo->r_idx + 1) % UART_BUFFER_LEN;
	}
	OSMutexPost(&FIFO_MUTEX,OS_OPT_POST_NONE,&err);

	return ret;

}

u8 Fifo_DataLen(pFIFO_T stFiFo)
{
	u8 len;
	OS_ERR      err;

	OSMutexPend (&FIFO_MUTEX,0,OS_OPT_PEND_BLOCKING,0,&err);
	
	if(stFiFo->r_idx <= stFiFo->w_idx)
	{
		len = stFiFo->w_idx - stFiFo->r_idx;
	}else{
		len = stFiFo->w_idx - stFiFo->r_idx + UART_BUFFER_LEN;
	}

	OSMutexPost(&FIFO_MUTEX,OS_OPT_POST_NONE,&err);
	return len;
}


