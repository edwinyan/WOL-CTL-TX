#ifndef _DEBUG_MSG_H_
#define _DEBUG_MSG_H_

/*----------------------------------------------------------------------------*/
#define MSG_LVL_ERR     0
#define MSG_LVL_WARN    1
#define MSG_LVL_DBG     2
#define MSG_LVL_INFO    3

#define MSG_LVL         MSG_LVL_INFO

#define SYS_ERR         "-E:"
#define SYS_WARN        "-W:"
#define SYS_DBG         "-D:"
#define SYS_INFO        "-I:"
#define SYS_ASSERT      "-A:"

void debug_msg_printf(CPU_CHAR *format, ...);

#define MSG_ERR(x,...)      ((MSG_LVL_ERR <= MSG_LVL)?debug_msg_printf(SYS_ERR x, ##__VA_ARGS__):(void)0)
#define MSG_WARN(x,...)     ((MSG_LVL_WARN <= MSG_LVL)?debug_msg_printf(SYS_WARN x, ##__VA_ARGS__):(void)0)
#define MSG_DBG(x,...)      ((MSG_LVL_DBG <= MSG_LVL)?debug_msg_printf(SYS_DBG x, ##__VA_ARGS__):(void)0)
#define MSG_INFO(x,...)     ((MSG_LVL_INFO <= MSG_LVL)?debug_msg_printf(SYS_INFO x, ##__VA_ARGS__):(void)0)
#define MSG(x,...)          debug_msg_printf(x, ##__VA_ARGS__)

#define ASSERT_R(x)       \
    do \
    { \
       if(!((x))) \
       { \
            debug_msg_printf(SYS_ASSERT "File: %s, Function: %s, Line: %d\r\n",__FILE__, __FUNCTION__, __LINE__); \
            while(1); \
       } \
    }while(0);

#define ASSERT_D(x)     ASSERT_R(x)


#endif

