#ifndef _DATALINK_DRV_H_
#define _DATALINK_DRV_H_

//output status of LB2 controller
#define SHUTTLE_ADJ2		0x01
#define SHUTTLE_ADJ1		0x02
#define	SHUTTLE_BUTTON		0x04
#define PHOTO_BUTTON		0x08
#define	PLAYBACK_BUTTON		0x10
#define MODE_SET1			0x20
#define MODE_SET2			0x40
#define VIDEO_BUTTON		0x80

#define POWER_BUTTON		0x01
#define RETURN_RESERVE2		0x02
#define RETURN_TOGGLE		0x04
#define RETURN_RESERVE1		0x08
#define RETURN_BUTTON		0x10

void datalink_recv(void);
void datalink_send(void);
void datalink_state(void);
u8 checkbufferdata(u8 *data,u8 size);
void packChannels(u8 index);

#endif
