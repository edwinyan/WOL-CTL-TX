#ifndef _RFM_COMMON_H_
#define _RFM_COMMON_H_

#include "binding.h"

struct rfm22_modem_regs {
  u32 bps;
  u8  r_1c, r_1d, r_1e, r_20, r_21, r_22, r_23, r_24, r_25, r_2a, r_6e, r_6f, r_70, r_71, r_72;
};

#define AVAILABLE    0
#define TRANSMIT    1
#define TRANSMITTED  2
#define RECEIVE    3
#define RECEIVED  4

extern volatile u8 RF_Mode;
//extern u8 RF_channel;
u32 getInterval(struct Bind_data *bd);
u8 getChannelCount(struct Bind_data *bd);
u8 getPacketSize(struct Bind_data *bd);
u8 countSetBits(u16 x);
void init_rfm(u8 isbind);
void tx_reset(void);
void rx_reset(void);
void check_module(void);
void setHopChannel(u8 ch);
void tx_packet_async(u8* pkt, u8 size);
void tx_packet(u8* pkt, u8 size);
u8 tx_done(void);
#endif
