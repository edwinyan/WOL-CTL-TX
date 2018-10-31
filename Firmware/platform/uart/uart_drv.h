#ifndef _UART_DRV_H_
#define _UART_DRV_H_

#define UART_BUFFER_LEN 	0xFF	// 1k

typedef enum{
    UART_SRC_START = 0,
    UART_SRC_DBG = UART_SRC_START,
    UART_SRC_GPRS,
    UART_SRC_MAVLINK,

    UART_SRC_NUM
}uart_src_enum;

typedef struct {
  	u8 buf[UART_BUFFER_LEN];        /* Circular buffer */
  	u8 w_idx;            	/* Index of write */
  	u8 r_idx;         		/* Index of read */
 }FIFO_T, *pFIFO_T;

void uart_drv_init(void);
void uart_drv_dbg_msg(u8 *msg);
u32 uart_drv_dbg_recv(u8 *buf, u32 len);

void Fifo_Init(pFIFO_T stFiFo);
bool Fifo_Write(pFIFO_T stFiFo, u8 dat);
bool Fifo_Read(pFIFO_T stFiFo,u8 *dat);
u8 Fifo_DataLen(pFIFO_T stFiFo);

#endif
