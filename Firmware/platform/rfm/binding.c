#include "common.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_flash.h"
#include "flash_drv.h"
#include "uart_drv.h"
#include "binding.h"

/*----------------------------------------------------------------------------*/
struct flash_head {
  u32 magic;
  u32 size;
  u32 checksum;
  u32 reserved;
};

u8 default_hop_list[] = {DEFAULT_HOPLIST};

u8 activeProfile = 0;
u8 defaultProfile = 0;

struct TX_config tx_config;
struct RX_config rx_config;
struct Bind_data bind_data;

#define PPM_CHANNELS 16

volatile u16 PPM[PPM_CHANNELS] = {
  512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512,512
};

u16 version = OPENLRSNG_VERSION;

const struct rxSpecialPinMap rxSpecialPins[] = {
  {  0, PINMAP_RSSI},
  {  0, PINMAP_LBEEP},
  {  5, PINMAP_PPM},
  {  8, PINMAP_LLIND},
  {  9, PINMAP_SDA},
  {  9, PINMAP_ANALOG}, // AIN0
  { 10, PINMAP_SCL},
  { 10, PINMAP_ANALOG}, // AIN1
  { 11, PINMAP_RXD},
  { 12, PINMAP_TXD},
  { 12, PINMAP_SPKTRM},
  { 12, PINMAP_SBUS},
  { 12, PINMAP_SUMD},
};

const uint8_t rxSpecialPinsSize = sizeof(rxSpecialPins);

static u32 checksum(u8 *data, u32 size)
{
  u32 ret = 0;

  while (size--) {
    if (ret & 0x80000000) {
      ret = (ret << 1) | 1;
    } else {
      ret = (ret << 1);
    }

    ret ^= *(data++);
  }

  return ret;
}

void rxInitDefaults(void)
{
  u8 i;
  
  rx_config.rx_type = RX_FLYTRON8CH;

  rx_config.pinMapping[RSSI_PIN] = PINMAP_RSSI;

  for (i = 1; i < 9; i++) {
    rx_config.pinMapping[i] = i; // default to PWM out
  }

  
  rx_config.pinMapping[PPM_PIN] = PINMAP_PPM;
  rx_config.pinMapping[9] = PINMAP_ANALOG;
  rx_config.pinMapping[10] = PINMAP_ANALOG;
  rx_config.pinMapping[11] = PINMAP_RXD;
  rx_config.pinMapping[12] = PINMAP_TXD;

  rx_config.flags = ALWAYS_BIND;
  rx_config.RSSIpwm = 255; //rssi injection disabled
  rx_config.beacon_frequency = DEFAULT_BEACON_FREQUENCY;
  rx_config.beacon_deadtime = DEFAULT_BEACON_DEADTIME;
  rx_config.beacon_interval = DEFAULT_BEACON_INTERVAL;
  rx_config.minsync = 3000;
  rx_config.failsafeDelay = 10;
  rx_config.ppmStopDelay = 0;
  rx_config.pwmStopDelay = 0;
}

void bindInitDefaults(void)
{
	u8 default_hop_list[] = {DEFAULT_HOPLIST};
  u8 c;
  bind_data.version = BINDING_VERSION;
  bind_data.serial_baudrate = DEFAULT_BAUDRATE;
  bind_data.rf_power = DEFAULT_RF_POWER;
  bind_data.rf_frequency = DEFAULT_CARRIER_FREQUENCY;
  bind_data.rf_channel_spacing = DEFAULT_CHANNEL_SPACING;

  bind_data.rf_magic = DEFAULT_RF_MAGIC;

  //bind_data.hopcount = sizeof(default_hop_list) / sizeof(default_hop_list[0]);


  for (c = 0; c < MAXHOPS; c++) {
    bind_data.hopchannel[c] = ((c < sizeof(default_hop_list)) ? default_hop_list[c] : 0);
  }

  bind_data.modem_params = 3;
  bind_data.flags = DEFAULT_FLAGS;
}

void txInitDefaults(void)
{
  u8 i;
  tx_config.max_frequency = MAX_RFM_FREQUENCY;
  tx_config.console_baud_rate = DEFAULT_BAUDRATE;
  tx_config.flags = 0x00;
  TX_CONFIG_SETMINCH(5); // 6ch
  for (i = 0; i < 16; i++) {
    tx_config.chmap[i] = i;
  }
}

void txWriteEeprom(void)
{
  FLASH_Status status;  
  struct flash_head head;
  u32 i;

  head.magic = BIND_MAGIC;
  head.size = sizeof(bind_data);
  head.checksum = checksum((u8 *)(&bind_data), sizeof(bind_data));

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR);

  if (FLASH_EraseSector(FLASH_GetFlashSector(FLASH_BDWRITE_ADDR),VoltageRange_3) == FLASH_COMPLETE) {//擦除当前扇区
    for (i = 0; i < sizeof(struct flash_head); i += 4) {
      status = FLASH_ProgramWord(FLASH_BDWRITE_ADDR + i, *(u32 *)((char *)&head + i));

      if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        MSG_ERR("FLASH ERROR on head!!!\r\n");

        while (1);
      }
    }

    for (i = 0; i < sizeof(bind_data); i += 4) {
	  //写入bind_data数据到flash中
      status = FLASH_ProgramWord(FLASH_BDWRITE_ADDR + sizeof(struct flash_head) + i, *(u32 *)((char *)&bind_data + i));

      if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        MSG_ERR("FLASH ERROR on bind!!!\r\n");

        while (1);
      }
    }

	//写入校验头到tx_config数据之前
	status = FLASH_ProgramWord(FLASH_TXWRITE_ADDR,BIND_MAGIC);
	if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        MSG_ERR("FLASH ERROR on bind!!!\r\n");

        while (1);
    }
	for(i=0; i< sizeof(tx_config); i += 4) {
		//写入tx_config数据到flash中
		status = FLASH_ProgramWord(FLASH_TXWRITE_ADDR + 4 + i, *(u32 *)((char *)&tx_config + i));
		if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        MSG_ERR("FLASH ERROR on tx_config!!!\r\n");

        while (1);
      }
	}

  } else {
    MSG_ERR("FLASH ERASE FAILED\r\n");
  }

  FLASH_Lock();
}

void rxWriteEeprom(void)
{
  FLASH_Status status;  
  struct flash_head head;
  u32 i;

  head.magic = BIND_MAGIC;
  head.size = sizeof(bind_data);
  head.checksum = checksum((u8 *)(&bind_data), sizeof(bind_data));

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR);

  if (FLASH_EraseSector(FLASH_GetFlashSector(FLASH_BDWRITE_ADDR),VoltageRange_3) == FLASH_COMPLETE) {//擦除当前扇区
    for (i = 0; i < sizeof(struct flash_head); i += 4) {
		//写入head校验
      status = FLASH_ProgramWord(FLASH_BDWRITE_ADDR + i, *(u32 *)((char *)&head + i));

      if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        MSG_ERR("FLASH ERROR on head!!!\r\n");

        while (1);
      }
    }

    for (i = 0; i < sizeof(bind_data); i += 4) {
	  //写入bind_data数据到flash中
      status = FLASH_ProgramWord(FLASH_BDWRITE_ADDR + sizeof(struct flash_head) + i, *(u32 *)((char *)&bind_data + i));

      if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        MSG_ERR("FLASH ERROR on bind!!!\r\n");

        while (1);
      }
    }

	//写入校验头到rx_config数据之前
	status = FLASH_ProgramWord(FLASH_RXWRITE_ADDR,BIND_MAGIC);
	if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        MSG_ERR("FLASH ERROR on bind!!!\r\n");

        while (1);
    }
	for(i=0; i< sizeof(rx_config); i += 4) {
		//写入rx_config数据到flash中
		status = FLASH_ProgramWord(FLASH_RXWRITE_ADDR + 4 + i, *(u32 *)((char *)&rx_config + i));
		if (status != FLASH_COMPLETE) {
        FLASH_Lock();
        MSG_ERR("FLASH ERROR on tx_config!!!\r\n");

        while (1);
      }
	}
#if 0
	//写入校验头到failsafePPM数据之前
	status = FLASH_ProgramWord(FLASH_FSWRITE_ADDR,0xDEADC0DE);
	if (status != FLASH_COMPLETE) {
	  FLASH_Lock();
	  MSG_ERR("FLASH ERROR on bind!!!\r\n");

	  while (1);
	}
	for(i=0; i< sizeof(PPM); i += 4) {
	  //写入failsafePPM数据到flash中
	  status = FLASH_ProgramWord(FLASH_FSWRITE_ADDR + 4 + i, *(u32 *)((char *)&PPM + i));
	  if (status != FLASH_COMPLETE) {
		  FLASH_Lock();
		  MSG_ERR("FLASH ERROR on tx_config!!!\r\n");

		  while (1);
	  }
	} 
#endif
  } else {
    MSG_ERR("FLASH ERASE FAILED\r\n");
  }

  FLASH_Lock();
}

s16 bindReadEeprom(void)
{
  const struct flash_head *head = (const struct flash_head *)(FLASH_BDWRITE_ADDR);
  const struct Bind_data *temp = (const struct Bind_data *)(FLASH_BDWRITE_ADDR + sizeof(struct flash_head));

  if (head->magic != BIND_MAGIC) {
    MSG_ERR("FLASH MAGIC FAIL\r\n");
    return 0;
  }

  if (head->size != sizeof(bind_data)) {
    MSG_ERR("FLASH SIZE FAIL\r\n");
    return 0;
  }

  if (head->checksum != checksum((u8 *)temp, sizeof(bind_data))) {
    MSG_ERR("FLASH CHECKSUM FAIL\r\n");
    return 0;
  }

  if (temp->version != BINDING_VERSION) {
    MSG_ERR("FLASH VERSION FAIL\r\n");
    return 0;
  }

  memcpy(&bind_data, temp, sizeof(bind_data));

  return 1;
}

void txReadEeprom(void)
{
  u32 *tempb = (u32 *)FLASH_TXWRITE_ADDR;

  if (*tempb != BIND_MAGIC) {
    MSG("TXconf reinit\r\n");
    txInitDefaults();
	bindInitDefaults();
    txWriteEeprom();
  } else {
    const struct tx_config *temp = (const struct tx_config *)(FLASH_TXWRITE_ADDR + 4);
    memcpy(&tx_config, temp, sizeof(tx_config));
    MSG("TXconf loaded\r\n");
  }
}

void rxReadEeprom(void)
{
  u32 *tempb = (u32 *)FLASH_RXWRITE_ADDR;

  if (*tempb != BIND_MAGIC) {
    MSG("RXconf reinit\r\n");
    rxInitDefaults();
    rxWriteEeprom();
  } else {
    const struct rx_config *temp = (const struct rx_config *)(FLASH_RXWRITE_ADDR + 4);
    memcpy(&rx_config, temp, sizeof(rx_config));
    MSG("RXconf loaded\r\n");
  }
}


void printRXconf(void)
{
  uint8_t i;
  MSG("Type: %d\r\n", rx_config.rx_type);

  for (i = 0; i < 13; i++) {
    MSG("pmap%d: %d\r\n", i, rx_config.pinMapping[i]);
  }

  MSG("Flag: %d\r\n", rx_config.flags);
  MSG("rssi: %d\r\n", rx_config.RSSIpwm);
  MSG("Bfre: %d\r\n", rx_config.beacon_frequency);
  MSG("Bdea: %d\r\n", rx_config.beacon_deadtime);
  MSG("Bint: %d\r\n", rx_config.beacon_interval);
  MSG("msyc: %d\r\n", rx_config.minsync);
  MSG("fsfe: %d\r\n", rx_config.failsafeDelay);
  MSG("ppms: %d\r\n", rx_config.ppmStopDelay);
  MSG("pwms: %d\r\n", rx_config.pwmStopDelay);
}


