#include "common.h"
#include "stm32f4xx_gpio.h"
#include "serial.h"
#include "serial_bsp_stm3240x.h"
#include "serial_line_tty.h"
#include "uart_drv.h"
#include "bsp_os.h"
#include "bsp.h"

extern OS_MUTEX	FIFO_MUTEX;
extern OS_MUTEX	TX_MUTEX;		//uart tx mutex
extern OS_MUTEX	RX_MUTEX;		//uart rx mutex

FIFO_T stFiFo;

enum{
    UART_PIN_TYPE_TX = 0,
    UART_PIN_TYPE_RX,

    UART_PIN_TYPE_NUM
};

typedef struct{
    char            *uart_name;
	USART_TypeDef	*USARTx;
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
        "USART1", USART1,&SerialDevCfg_STM32_USART1, 
        {SERIAL_BAUDRATE_115200, SERIAL_DATABITS_8, SERIAL_STOPBITS_1, SERIAL_PARITY_NONE, SERIAL_FLOW_CTRL_NONE}, 
        0,{GPIOA, GPIOA},{GPIO_Pin_9,GPIO_Pin_10}, {GPIO_PinSource9, GPIO_PinSource10},GPIO_AF_USART1,DEF_FALSE
    },

    //endmark
    { 
        "", NULL, NULL,
        {SERIAL_BAUDRATE_115200, SERIAL_DATABITS_8, SERIAL_STOPBITS_1, SERIAL_PARITY_NONE, SERIAL_FLOW_CTRL_NONE}, 
        0,{GPIOA, GPIOA},{GPIO_Pin_9,GPIO_Pin_10}, {GPIO_PinSource9, GPIO_PinSource10},GPIO_AF_USART1,DEF_FALSE
    },
};

void USART1_IRQHandler(void)
{
	OSIntEnter();
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    //进中断的标志
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		Fifo_Write(&stFiFo,USART_ReceiveData(USART1));
		//USART_SendData(USART1, USART_ReceiveData(USART1));      //接收到的数据重新发送到串口   
	}
	OSIntExit(); 
}

void USART2_IRQHandler(void)
{
	OSIntEnter();
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)    //进中断的标志
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		Fifo_Write(&stFiFo,USART_ReceiveData(USART2));
		//USART_SendData(USART1, USART_ReceiveData(USART1));      //接收到的数据重新发送到串口   
	}
	OSIntExit(); 
}

void USART3_IRQHandler(void)
{
	OSIntEnter();
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)    //进中断的标志
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		Fifo_Write(&stFiFo,USART_ReceiveData(USART3));
		//USART_SendData(USART1, USART_ReceiveData(USART1));      //接收到的数据重新发送到串口   
	}
	OSIntExit(); 
}

void USART4_IRQHandler(void)
{
	OSIntEnter();
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)    //进中断的标志
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);
		Fifo_Write(&stFiFo,USART_ReceiveData(UART4));
		//USART_SendData(USART1, USART_ReceiveData(USART1));      //接收到的数据重新发送到串口   
	}
	OSIntExit(); 
}

void USART5_IRQHandler(void)
{
	OSIntEnter();
	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)    //进中断的标志
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);
		Fifo_Write(&stFiFo,USART_ReceiveData(UART5));
		//USART_SendData(USART1, USART_ReceiveData(USART1));      //接收到的数据重新发送到串口   
	}
	OSIntExit(); 
}

void USART6_IRQHandler(void)
{
	OSIntEnter();
	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)    //进中断的标志
	{
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);
		Fifo_Write(&stFiFo,USART_ReceiveData(USART6));
		//USART_SendData(USART1, USART_ReceiveData(USART1));      //接收到的数据重新发送到串口   
	}
	OSIntExit(); 
}

void USART_IRQInit(USART_TypeDef* USARTx)
{
	if(USARTx == USART1)
	{
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		BSP_IntVectSet(BSP_INT_ID_USART1, USART1_IRQHandler); //设置串口1的中断向量
		BSP_IntEn(BSP_INT_ID_USART1);
	}else if(USARTx == USART2){
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		BSP_IntVectSet(BSP_INT_ID_USART2, USART2_IRQHandler); //设置串口2的中断向量
		BSP_IntEn(BSP_INT_ID_USART2);
	}else if(USARTx == USART3){
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		BSP_IntVectSet(BSP_INT_ID_USART3, USART3_IRQHandler); //设置串口3的中断向量
		BSP_IntEn(BSP_INT_ID_USART3);
	}else if(USARTx == UART4){
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
		BSP_IntVectSet(BSP_INT_ID_USART4, USART4_IRQHandler); //设置串口4的中断向量
		BSP_IntEn(BSP_INT_ID_USART4);
	}else if(USARTx == UART5){
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
		BSP_IntVectSet(BSP_INT_ID_USART5, USART5_IRQHandler); //设置串口5的中断向量
		BSP_IntEn(BSP_INT_ID_USART5);
	}else if(USARTx == USART6){
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
		BSP_IntVectSet(BSP_INT_ID_USART6, USART6_IRQHandler); //设置串口6的中断向量
		BSP_IntEn(BSP_INT_ID_USART6);
	}
	
}

void uart_drv_init(void)
{
    uart_src_enum src;
//    SERIAL_ERR     err;
    uart_drv_t     *uart_drv;
    GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

    //gpio config
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    
    
    //serial init
    //Serial_Init(); 
    Fifo_Init(&stFiFo);
    
    for(src = UART_SRC_START; src < UART_SRC_NUM; src++)
    {
        uart_drv = &uart_drv_array[src];
		//Fifo_Init(&stFiFo);
        
        if(Str_Len(uart_drv->uart_name))
        {
            GPIO_PinAFConfig(uart_drv->port[UART_PIN_TYPE_TX], uart_drv->pin_src[UART_PIN_TYPE_TX], uart_drv->uart_af_nbr);
            GPIO_InitStructure.GPIO_Pin = uart_drv->pin[UART_PIN_TYPE_TX];
            GPIO_Init(uart_drv->port[UART_PIN_TYPE_TX], &GPIO_InitStructure);
            
            GPIO_PinAFConfig(uart_drv->port[UART_PIN_TYPE_RX], uart_drv->pin_src[UART_PIN_TYPE_RX], uart_drv->uart_af_nbr);
            GPIO_InitStructure.GPIO_Pin = uart_drv->pin[UART_PIN_TYPE_RX];
            GPIO_Init(uart_drv->port[UART_PIN_TYPE_RX], &GPIO_InitStructure);

    	#if 0
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
		#else
			USART_InitStructure.USART_BaudRate = uart_drv->uart_if_cfg.Baudrate;//一般设置为9600; 
			USART_InitStructure.USART_WordLength = uart_drv->uart_if_cfg.DataBits;//字长为8位数据格式 
			USART_InitStructure.USART_StopBits = uart_drv->uart_if_cfg.StopBits;//一个停止位 
			USART_InitStructure.USART_Parity = uart_drv->uart_if_cfg.Parity;//无奇偶校验位 
			USART_InitStructure.USART_HardwareFlowControl = uart_drv->uart_if_cfg.FlowCtrl;//无硬件数据流控制 
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式 
			USART_Init(uart_drv->USARTx, &USART_InitStructure); //初始化串口 

			USART_IRQInit(uart_drv->USARTx);

			USART_Cmd(uart_drv->USARTx, ENABLE);                //使能串口

		#endif
        }
        else
        {
            break;
        }
    }
}

#if 0
void uart_drv_dbg_msg(u8 *msg,u32 len)
{
    SERIAL_ERR  err;
	OS_ERR      Err;
	
    uart_drv_t *uart_drv = &uart_drv_array[UART_SRC_DBG];
    ASSERT_R(DEF_TRUE == uart_drv->uart_enabled);

	OSMutexPend (&TX_MUTEX,0,OS_OPT_PEND_BLOCKING,0,&Err);
    Serial_Wr((SERIAL_IF_NBR   )uart_drv->uart_if_nbr,
              (void           *)&msg[0],
              (CPU_SIZE_T      )len,
              (CPU_INT32U      )0,
              (SERIAL_ERR     *)&err);
	OSMutexPost(&TX_MUTEX,OS_OPT_POST_NONE,&Err);
	
    ASSERT_R(err == SERIAL_ERR_NONE);
}

u32 uart_drv_dbg_recv(u8 *buf, u32 len)
{
    u32 read_len;
    SERIAL_ERR  err;
	OS_ERR      Err;
	
    uart_drv_t *uart_drv = &uart_drv_array[UART_SRC_DBG];
    ASSERT_R(DEF_TRUE == uart_drv->uart_enabled);

	OSMutexPend (&RX_MUTEX,0,OS_OPT_PEND_BLOCKING,0,&Err);
    read_len = Serial_Rd((SERIAL_IF_NBR   )uart_drv->uart_if_nbr,
              (void           *)buf,
              (CPU_SIZE_T      )len,
              (CPU_INT32U      )0,
              (SERIAL_ERR     *)&err);
   	OSMutexPost(&RX_MUTEX,OS_OPT_POST_NONE,&Err);
	
    if(err != SERIAL_ERR_NONE)
    {
        MSG_INFO("%s err: status: 0x%x, len: 0x%x\r\n", __FUNCTION__,err,len);
        //ASSERT_R(err == SERIAL_ERR_NONE);
    }
    
    return read_len;
}
#endif
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

void uart_drv_dbg_msg(u8 *msg)
{
	OS_ERR      err;
	
	OSMutexPend (&TX_MUTEX,0,OS_OPT_PEND_BLOCKING,0,&err);
	
	while(*msg)
	{
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);          
    		USART_SendData(USART1, *msg );
		msg ++;
	}

	OSMutexPost(&TX_MUTEX,OS_OPT_POST_NONE,&err);
}

void uart_drv_dbg_send(u8 *msg,u32 len)
{
	OS_ERR      err;
	
	OSMutexPend (&TX_MUTEX,0,OS_OPT_PEND_BLOCKING,0,&err);
	
	while(len--)
	{
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);          
    		USART_SendData(USART1, *msg );
		msg ++;
	}

	OSMutexPost(&TX_MUTEX,OS_OPT_POST_NONE,&err);
}



