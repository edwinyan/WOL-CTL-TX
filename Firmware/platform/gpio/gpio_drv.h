#ifndef _GPIO_DRV_H_
#define _GPIO_DRV_H_

typedef enum{
    GPIO_SRC_START = 0,
	GPIO_SRC_SHUTTLE2 =GPIO_SRC_START,
	GPIO_SRC_SHUTTLE1,
	GPIO_SRC_SHUTTLE,
	GPIO_SRC_PHOTO,
	GPIO_SRC_PLAYBACK,
	GPIO_SRC_MODE_SET1,
	GPIO_SRC_MODE_SET2,
	GPIO_SRC_VIDEO,
	GPIO_SRC_RETURN_RESERVE2,
	GPIO_SRC_RETURN_TOGGLE,
	GPIO_SRC_RETURN_RESERVE1,
	GPIO_SRC_RETURN,
	GPIO_SRC_POWER,
	
	GPIO_SRC_NUM

}gpio_src_enum;

void gpio_drv_init(void);
void gpio_value_set(u8 src);
void gpio_value_reset(u8 src);
//void rx_gpio_init(void);
//void tx_gpio_init(void);



#endif
