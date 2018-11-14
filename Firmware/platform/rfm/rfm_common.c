#include "common.h"
#include "stm32f4xx_gpio.h"
#include "rfm_common.h"
#include "bsp_os.h"
#include "bsp.h"
#include "uart_drv.h"
#include "led_drv.h"
#include "rfm_drv.h"
#include "binding.h"

u32 tx_start = 0;
volatile u8 RF_Mode = 0;
static const u8 pktsizes[8] = {0, 7, 16, 12, 16, 17, 21, 0};
//u8 RF_channel = 0;
//#define TX_TIMING_DEBUG

struct rfm22_modem_regs modem_params[] = {
  { 4800, 0x1a, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x1b, 0x1e, 0x27, 0x52, 0x2c, 0x23, 0x30 }, // 50000 0x00
  { 9600, 0x05, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x20, 0x24, 0x4e, 0xa5, 0x2c, 0x23, 0x30 }, // 25000 0x00
  { 19200, 0x06, 0x40, 0x0a, 0xd0, 0x00, 0x9d, 0x49, 0x00, 0x7b, 0x28, 0x9d, 0x49, 0x2c, 0x23, 0x30 }, // 25000 0x01
  { 57600, 0x05, 0x40, 0x0a, 0x45, 0x01, 0xd7, 0xdc, 0x03, 0xb8, 0x1e, 0x0e, 0xbf, 0x00, 0x23, 0x2e },
  { 125000, 0x8a, 0x40, 0x0a, 0x60, 0x01, 0x55, 0x55, 0x02, 0xad, 0x1e, 0x20, 0x00, 0x00, 0x23, 0xc8 },
};

#define DATARATE_COUNT (sizeof(modem_params) / sizeof(modem_params[0]))

struct rfm22_modem_regs bind_params =
{ 9600, 0x05, 0x40, 0x0a, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x20, 0x24, 0x4e, 0xa5, 0x2c, 0x23, 0x30 };

/*----------------------------------------------------------------------------*/

u8 getPacketSize(struct Bind_data *bd)
{
  return pktsizes[(bd->flags & 0x07)];
}

u8 getChannelCount(struct Bind_data *bd)
{
  return (((bd->flags & 7) / 2) + 1 + (bd->flags & 1)) * 4;
}

u32 getInterval(struct Bind_data *bd)
{
	u32 ret;
	// Sending a x byte packet on bps y takes about (emperical)
	// usec = (x + 15) * 8200000 / baudrate
#define BYTES_AT_BAUD_TO_USEC(bytes,bps) ((u32)((bytes)+15) * 8200000L / (u32)(bps))

	ret = (BYTES_AT_BAUD_TO_USEC(getPacketSize(bd), modem_params[bd->modem_params].bps) + 8000);

	if (bd->flags & TELEMETRY_MASK) {
		ret += (BYTES_AT_BAUD_TO_USEC(TELEMETRY_PACKETSIZE, modem_params[bd->modem_params].bps) + 9000);
	}

	// round up to ms
	ret = ((ret + 999) / 1000) * 1000;
	
	return ret;
}

u8 countSetBits(u16 x)
{
  x  = x - ((x >> 1) & 0x5555);
  x  = (x & 0x3333) + ((x >> 2) & 0x3333);
  x  = x + (x >> 4);
  x &= 0x0F0F;
  return (x * 0x0101) >> 8;
}

static void rfmSetModemRegs(struct rfm22_modem_regs* r)
{
  spiWriteRegister(RFM22B_IFBW,      r->r_1c);
  spiWriteRegister(RFM22B_AFCLPGR,   r->r_1d);
  spiWriteRegister(RFM22B_AFCTIMG,   r->r_1e);
  spiWriteRegister(RFM22B_RXOSR,     r->r_20);
  spiWriteRegister(RFM22B_NCOFF2,    r->r_21);
  spiWriteRegister(RFM22B_NCOFF1,    r->r_22);
  spiWriteRegister(RFM22B_NCOFF0,    r->r_23);
  spiWriteRegister(RFM22B_CRGAIN1,   r->r_24);
  spiWriteRegister(RFM22B_CRGAIN0,   r->r_25);
  spiWriteRegister(RFM22B_AFCLIM,    r->r_2a);
  spiWriteRegister(RFM22B_TXDR1,     r->r_6e);
  spiWriteRegister(RFM22B_TXDR0,     r->r_6f);
  spiWriteRegister(RFM22B_MODCTL1,   r->r_70);
  spiWriteRegister(RFM22B_MODCTL2,   r->r_71);
  spiWriteRegister(RFM22B_FREQDEV,   r->r_72);
}

void rfm_reg_test(void)
{
	MSG("reg test start\r\n");
	spiReadRegister(0x09);
	spiReadRegister(0x0a);
	spiReadRegister(0x0b);
	spiReadRegister(0x0c);
	//spiReadRegister(0x0d);
	//spiReadRegister(0x30);
	//spiReadRegister(0x32);
	//spiReadRegister(0x33);
	//spiReadRegister(0x34);

	spiReadRegister(RFM22B_SYNC3);    // sync word 3
    spiReadRegister(RFM22B_SYNC2);    // sync word 2
    spiReadRegister(RFM22B_SYNC1);    // sync word 1 (not used)
    spiReadRegister(RFM22B_SYNC0);    // sync word 0 (not used)
	MSG("reg test end\r\n");
}

void init_rfm(u8 isbind)
{
  u8 i;
  u8 magic;
  rfmClearIntStatus();
  spiWriteRegister(RFM22B_INTEN2, 0x00);
  rfmSetReadyMode(); // turn on the XTAL and give it time to settle
  BSP_OS_TimeDly(600);
  
  rfmInit(bind_data.flags&DIVERSITY_ENABLED);
  rfmSetStepSize(bind_data.rf_channel_spacing);

  magic = isbind ? BIND_MAGIC : bind_data.rf_magic;
  for (i = 0; i < 4; i++) {
    rfmSetHeader(i, (magic >> 24) & 0xff);
    magic = magic << 8; // advance to next byte
  }

  if (isbind) {
    rfmSetModemRegs(&bind_params);
    rfmSetPower(BINDING_POWER);
    rfmSetCarrierFrequency(BINDING_FREQUENCY);
  } else {
    rfmSetModemRegs(&modem_params[bind_data.modem_params]);
    rfmSetPower(bind_data.rf_power);
    rfmSetCarrierFrequency(bind_data.rf_frequency);
  }

  //rfm_reg_test();
}



void tx_reset(void)
{
  OS_ERR  err;
  tx_start = OSTimeGet(&err);
  RF_Mode = TRANSMIT;
  rfmSetTX();
}

void rx_reset(void)
{
  rfmClearFIFO(bind_data.flags & DIVERSITY_ENABLED);
  rfmClearIntStatus();
  RF_Mode = RECEIVE;
  rfmSetRX();
}

void check_module(void)
{
  if (rfmGetGPIO1() == 0) {
    // detect the locked module and reboot
    MSG("RFM Module Locked\r\n");
    LED_R_ON;
    init_rfm(0);
    rx_reset();
    LED_R_OFF;
  }
 // BSP_OS_TimeDly(10);
}

void setHopChannel(u8 ch)
{
  rfmSetChannel(bind_data.hopchannel[ch]);
}

void tx_packet_async(u8* pkt, u8 size)
{
  //OS_ERR      err;
  rfmSendPacket(pkt, size);
  //MSG("-- 11 --%d\r\n",OSTimeGet(&err));
  rfmClearIntStatus();
 // MSG("-- 12 --%d\r\n",OSTimeGet(&err));
  tx_reset();
 // MSG("-- 13 --%d\r\n",OSTimeGet(&err));
}

void tx_packet(u8* pkt, u8 size)
{
  OS_ERR  err;
  tx_packet_async(pkt, size);
  while ((RF_Mode == TRANSMIT) && ((OSTimeGet(&err) - tx_start) < 100));

  #ifdef TX_TIMING_DEBUG
  if (RF_Mode == TRANSMIT) {
   MSG("TX timeout!\r\n");
  }
  //MSG("TX took:%d\r\n",(OSTimeGet(&err) - tx_start) < 100);
  #endif
}

u8 tx_done(void)
{
  OS_ERR  err;
  if (RF_Mode == TRANSMITTED) {
  	#ifdef TX_TIMING_DEBUG
    MSG("TX success! TX took:%d\r\n",(OSTimeGet(&err) - tx_start));
    #endif
    RF_Mode = AVAILABLE;
    return 1; // success
  } else if ((RF_Mode == TRANSMIT) && ((OSTimeGet(&err) - tx_start) > 100)) {
  	MSG("TX timeout!\r\n");
    RF_Mode = AVAILABLE;
    return 2; // timeout
  }
  return 0;
}


