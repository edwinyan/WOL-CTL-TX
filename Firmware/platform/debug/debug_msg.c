#include "common.h"
#include "uart_drv.h"

#define MSG_BUF_LEN     (256)
STATIC u8 msg_buf[MSG_BUF_LEN + 4];

void debug_msg_printf(CPU_CHAR *format, ...)
{
    va_list     vArgs;
    va_start(vArgs, format);
    vsprintf((char       *)msg_buf,(char const *)format,vArgs);
    va_end(vArgs);

    uart_drv_dbg_msg(msg_buf);        
}
