#ifndef _BINDING_H_
#define _BINDING_H_

// OpenLRSng binding

// Version number in single uint16 [8bit major][4bit][4bit]
// a.b.c == 0xaabc
#define OPENLRSNG_VERSION 0x0388
extern u16 version;


// Factory setting values, modify via the CLI

//####### RADIOLINK RF POWER (beacon is always 100/13/1.3mW) #######
// 7 == 100mW (or 1000mW with M3)
// 6 == 50mW (use this when using booster amp), (800mW with M3)
// 5 == 25mW
// 4 == 13mW
// 3 == 6mW
// 2 == 3mW
// 1 == 1.6mW
// 0 == 1.3mW
#define DEFAULT_RF_POWER 7

#define DEFAULT_CHANNEL_SPACING 5 // 50kHz
#define DEFAULT_HOPLIST 22,10,19,34,49,41
#define DEFAULT_RF_MAGIC 0xDEADFEED

//  0 -- 4800bps, best range
//  1 -- 9600bps, medium range
//  2 -- 19200bps, medium range
#define DEFAULT_DATARATE 2

#define DEFAULT_BAUDRATE 115200

// TX_CONFIG flag masks
#define SW_POWER            0x04 // enable powertoggle via switch (JR dTX)
#define ALT_POWER           0x08
#define MUTE_TX             0x10 // do not beep on telemetry loss
#define MICROPPM            0x20
#define INVERTED_PPMIN      0x40
#define WATCHDOG_USED       0x80 // read only flag, only sent to configurator

// RX_CONFIG flag masks
#define PPM_MAX_8CH         0x01
#define ALWAYS_BIND         0x02
#define SLAVE_MODE          0x04
#define IMMEDIATE_OUTPUT    0x08
#define STATIC_BEACON       0x10
#define INVERTED_PPMOUT      0x40
#define WATCHDOG_USED       0x80 // read only flag, only sent to configurator

// BIND_DATA flag masks
#define TELEMETRY_OFF       0x00
#define TELEMETRY_PASSTHRU  0x08
#define TELEMETRY_FRSKY     0x10 // covers smartport if used with &
#define TELEMETRY_SMARTPORT 0x18
#define TELEMETRY_MASK      0x18
#define CHANNELS_4_4        0x01
#define CHANNELS_8          0x02
#define CHANNELS_8_4        0x03
#define CHANNELS_12         0x04
#define CHANNELS_12_4       0x05
#define CHANNELS_16         0x06
#define DIVERSITY_ENABLED   0x80
#define DEFAULT_FLAGS       (CHANNELS_8 | TELEMETRY_PASSTHRU)

#define RX_FLYTRON8CH 0x01
#define RX_OLRSNG4CH  0x02
#define RX_OLRSNG12CH 0x03
#define RX_DTFUHF10CH 0x04

#define PINMAP_PPM 	  0x20
#define PINMAP_RSSI   0x21
#define PINMAP_SDA    0x22
#define PINMAP_SCL    0x23
#define PINMAP_RXD    0x24
#define PINMAP_TXD    0x25
#define PINMAP_ANALOG 0x26
#define PINMAP_LBEEP  0x27 // packetloss beeper
#define PINMAP_SPKTRM 0x28 // spektrum satellite output
#define PINMAP_SBUS   0x29 // SBUS output
#define PINMAP_SUMD   0x2a // SUMD output
#define PINMAP_LLIND  0x2b // LinkLoss indication (digital output)

#define ANALOG0_PIN 	9
#define ANALOG1_PIN 	10		
#define RSSI_PIN 		0
#define PPM_PIN 		5

#define TX_HEAD	0xAA
#define TX_TAIL	0x55

#define RX_HEAD	0xA5
#define RX_TAIL	0x5A

typedef enum {
    SERIAL_MODE_NONE = 0,
    SERIAL_MODE_SPEKTRUM1024,
    SERIAL_MODE_SPEKTRUM2048,
    SERIAL_MODE_SBUS,
    SERIAL_MODE_SUMD,
    SERIAL_MODE_MULTI,
    SERIAL_MODE_MAX = SERIAL_MODE_MULTI
} serialMode_e;

#define MULTI_OPERATION_TIMEOUT_MS 5000

// helper macro for European PMR channels
#define EU_PMR_CH(x) (445993750L + 12500L * (x)) // valid for ch1-ch16 (Jan 2016  ECC update)

// helper macro for US FRS channels 1-7
#define US_FRS_CH(x) (462537500L + 25000L * (x)) // valid for ch1-ch7

#define DEFAULT_BEACON_FREQUENCY 0 // disable beacon
#define DEFAULT_BEACON_DEADTIME 30 // time to wait until go into beacon mode (30s)
#define DEFAULT_BEACON_INTERVAL 10 // interval between beacon transmits (10s)

#define MIN_DEADTIME 0
#define MAX_DEADTIME 255

#define MIN_INTERVAL 1
#define MAX_INTERVAL 255

#define BINDING_POWER     0x06 // not lowest since may result fail with RFM23BP

#define TELEMETRY_PACKETSIZE 9
#define MAX_PACKETSIZE 16		//we use 16bytes for TX module send buffer

#define BIND_MAGIC (0xDEC1BE15 + (OPENLRSNG_VERSION & 0xfff0))
#define BINDING_VERSION ((OPENLRSNG_VERSION & 0x0ff0)>>4)

// HW frequency limits
#if (RFMTYPE == 868)
#  define MIN_RFM_FREQUENCY 848000000
#  define MAX_RFM_FREQUENCY 888000000
#  define DEFAULT_CARRIER_FREQUENCY 868000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 868000000 // Hz
#elif (RFMTYPE == 915)
#  define MIN_RFM_FREQUENCY 895000000
#  define MAX_RFM_FREQUENCY 935000000
#  define DEFAULT_CARRIER_FREQUENCY 915000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 915000000 // Hz
#else
#  define MIN_RFM_FREQUENCY 413000000
#  define MAX_RFM_FREQUENCY 463000000
#  define DEFAULT_CARRIER_FREQUENCY 435000000  // Hz  (ch 0)
#  define BINDING_FREQUENCY 435000000 // Hz
#endif

#define MAXHOPS      24
#define PPM_CHANNELS 16

//使用最后一个扇区作为flash存储空间
#define FLASH_BASE_ADDR		0x080E0000
#define FLASH_PAGE_SIZE  ((u16)0x400)
#define FLASH_BDWRITE_ADDR (FLASH_BASE_ADDR + (u32)FLASH_PAGE_SIZE * 0) // use the last page
#define FLASH_TXWRITE_ADDR (FLASH_BASE_ADDR + (u32)FLASH_PAGE_SIZE * 1)
#define FLASH_RXWRITE_ADDR (FLASH_BASE_ADDR + (u32)FLASH_PAGE_SIZE * 2)
#define FLASH_FSWRITE_ADDR (FLASH_BASE_ADDR + (u32)FLASH_PAGE_SIZE * 3)

struct TX_config {
  u8  rfm_type;
  u32 max_frequency;
  u32 console_baud_rate;
  u32 flags;
  u8  chmap[16];
};

// 27 bytes
struct RX_config {
  u8  rx_type; // RX type fillled in by RX, do not change
  u8  pinMapping[13];
  u8  flags;
  u8  RSSIpwm; //0-15 inject composite, 16-31 inject quality, 32-47 inject RSSI, 48-63 inject quality & RSSI on two separate channels
  u32 beacon_frequency;
  u8  beacon_deadtime;
  u8  beacon_interval;
  u16 minsync;
  u8  failsafeDelay;
  u8  ppmStopDelay;
  u8  pwmStopDelay;
};

// 18 bytes
struct Bind_data {
  u8 version;
  u32 serial_baudrate;
  u32 rf_frequency;
  u32 rf_magic;
  u8 rf_power;
  u8 rf_channel_spacing;
  u8 hopchannel[MAXHOPS];
  u8 modem_params;
  u8 flags;
};

// 1:tx  0:rx
#define TX 1
//#define TX_CYCLE		20		//ms,发送周期
//#define RX_TIMEOUT		20		//接收超时
//#define HOP_TIMEOUT		



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

#define RETURN_LED			0x01
#define POWER_LED			0x02

struct rxSpecialPinMap {
  u8 output;
  u8 type;
};

extern struct TX_config tx_config;
extern struct RX_config rx_config;
extern struct Bind_data bind_data;
extern const struct rxSpecialPinMap rxSpecialPins[];
extern volatile u16 PPM[PPM_CHANNELS];
extern const u8 rxSpecialPinsSize;

// 0 - no PPM needed, 1=2ch ... 0x0f=16ch
#define TX_CONFIG_GETMINCH() (tx_config.flags >> 28)
#define TX_CONFIG_SETMINCH(x) (tx_config.flags = (tx_config.flags & 0x0fffffff) | (((u32)(x) & 0x0f) << 28))

void rxInitDefaults(void);
void bindInitDefaults(void);
void txInitDefaults(void);
void txWriteEeprom(void);
void rxWriteEeprom(void);
s16 bindReadEeprom(void);
void txReadEeprom(void);
void rxReadEeprom(void);
void failsafeLoad(void);
void printRXconf(void);
//u8 checkbufferdata(u8 *data,u8 size);

#endif
