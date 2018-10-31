#ifndef _BUTTON_DRV_H_
#define _BUTTON_DRV_H_

#if(TX == 1)
typedef enum{
    BUTTON_SRC_START = 0,
    BUTTON_SRC_PAIR = BUTTON_SRC_START,    //对码按键
    BUTTON_SRC_SHUTTLE2,
    BUTTON_SRC_SHUTTLE1,
    BUTTON_SRC_SHUTTLE,
    BUTTON_SRC_PHOTO,
    BUTTON_SRC_PLAYBACK,
    BUTTON_SRC_MODE_SET1,
    BUTTON_SRC_MODE_SET2,
    BUTTON_SRC_VIDEO,
    BUTTON_SRC_POWER,
    BUTTON_SRC_RETURN_RESERVE2,
    BUTTON_SRC_RETURN_TOGGLE,
    BUTTON_SRC_RETURN_RESERVE1,
    BUTTON_SRC_RETURN,
    
    BUTTON_SRC_NUM
}button_src_enum;
#else
typedef enum{
    BUTTON_SRC_START = 0,
    BUTTON_SRC_POWER_CTRL2 = BUTTON_SRC_START,    //对码按键
    BUTTON_SRC_POWER_CTRL1,
	BUTTON_SRC_RETURN_LED,
	
	BUTTON_SRC_NUM
}button_src_enum;

#endif

void button_drv_init(void);
u8 button_value_get(button_src_enum src);

#endif
