#ifndef _WATCHDOG_DRV_H_
#define _WATCHDOG_DRV_H_

#define WATCHDOG_64MS	0
#define WATCHDOG_125MS	1
#define WATCHDOG_250MS	2
#define WATCHDOG_500MS	3
#define WATCHDOG_1S		4
#define WATCHDOG_2S		5
#define WATCHDOG_4S		6
#define WATCHDOG_8S		7

void IWDG_Init(u8 prer);
void IWDG_Feed(void);

#endif
