#include "common.h"
#include "stm32f4xx_gpio.h"
#include "rx.h"
#include "bsp_os.h"
#include "bsp.h"
#include "binding.h"
#include "uart_drv.h"
#include "rfm_common.h"
#include "rfm_drv.h"
#include "led_drv.h"
#include "pwm_drv.h"
#include "gpio_drv.h"
#include "button_drv.h"

#if (TX == 0)

u8 watchdogUsed = 0;

u32 lastPacketTimeMs = 0;
u32 nextBeaconTimeMs;

u16 lastAFCCvalue = 0;
u16 linkQuality = 0;
u8  linkQ;
u8  lastRSSIvalue = 0;
u32 lastRSSITimeMs = 0;
u32 RSSI_timeout = 0;
u32 pktTimeDelta = 0;
u16 RSSI_sum = 0;
u8  RSSI_count = 0;
u8  smoothRSSI = 0;

u32 timeUs = 0;
u32 timeMs= 0;

u8 linkAcquired = 0;
u8 numberOfLostPackets = 0;
static u8 numberOfPacket=0;
static u8 lastNumberOfPacket=0;
u8 fristPacket=0;
u32 linkLossTimeMs;
u32 hopTimeoutSlow = 0;

u32 hopInterval = 0;
u32 hopTimeout = 0;
u8 hopcount;
u8 willhop = 0;

static u8 rf_channel=0;

u16 beaconRSSIavg = 255;

u8 Rx_buf[MAX_PACKETSIZE]; // RX buffer (uplink)

u8 Tx_buf[TELEMETRY_PACKETSIZE]; // TX buffer (downlink)(type plus 8 x data)

const pinDefine_t pinDefine[] = {
  { GPIOC, GPIO_Pin_6, },          // RSSI
  { GPIOC, GPIO_Pin_7, },          // PWM1 ADC12_IN0
};


u32 test_value_rx=1000;

u16 pwm[5]={0};
u8 button_output[13] = {0};

#define ADC_OUTPUT_COMPENSATE	(42.5f/32.5f)
/*----------------------------------------------------------------------------*/

u8 checkIfConnected(u8 pin1, u8 pin2)
{
    u8 ret = 0;
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = pinDefine[pin1].pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(pinDefine[pin1].gpio, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pinDefine[pin2].pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(pinDefine[pin2].gpio, &GPIO_InitStructure);

    GPIO_SetBits(pinDefine[pin1].gpio, pinDefine[pin1].pin);
    BSP_OS_TimeDly(100);

    if (GPIO_ReadInputDataBit(pinDefine[pin2].gpio, pinDefine[pin2].pin))
    {
        GPIO_ResetBits(pinDefine[pin1].gpio, pinDefine[pin1].pin);
        BSP_OS_TimeDly(100);

        if (!GPIO_ReadInputDataBit(pinDefine[pin2].gpio, pinDefine[pin2].pin))
        {
            ret = 1;
        }
    }

    //Return pins to default state
    GPIO_InitStructure.GPIO_Pin = pinDefine[pin1].pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(pinDefine[pin1].gpio, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pinDefine[pin2].pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(pinDefine[pin2].gpio, &GPIO_InitStructure);
    return ret;
}

u8 bindReceive(u32 timeout)
{
  OS_ERR  err;
  u32 start;
  u8 rxc_buf[33];
  u8 len;
  u16 i;

  start = OSTimeGet(&err);
  MSG("bindReceive enter timeout=%d @%d\r\n",timeout,OSTimeGet(&err));
  init_rfm(1);
  rx_reset();
  MSG("Waiting bind\r\n");
  while ((!timeout) || ((OSTimeGet(&err) - start) < timeout)) {
    if (RF_Mode == RECEIVED) {
      MSG("Got pkt\r\n");
      len=rfmGetPacketLength();
      rfmGetPacket(rxc_buf, len);
      switch((char) rxc_buf[0]) {
	      case 'b': { // GET bind_data
	        memcpy(&bind_data, (rxc_buf + 1), sizeof(bind_data));
	        if (bind_data.version == BINDING_VERSION) {
	          MSG("data good\r\n");
	          rxc_buf[0] = 'B';
			  for(i=0;i<10;i++){
	          	tx_packet(rxc_buf, 1); // ACK that we got bound
	          	BSP_OS_TimeDly(10);
			  }
	          LED_G_ON; //signal we got bound on LED:s
	          return 1;
	        }
	      }
	      break;
	      case 'p': // GET rx_config
	      case 'i': { // GET & reset rx_config
	        if (rxc_buf[0] == 'p') {
	          rxc_buf[0] = 'P';
	          timeout = 0;
	        } else {
	          rxInitDefaults();
	          rxc_buf[0] = 'I';
	        }
	        if (watchdogUsed) {
	          rx_config.flags |= WATCHDOG_USED;
	        } else {
	          rx_config.flags &=~ WATCHDOG_USED;
	        }
	        memcpy((rxc_buf + 1), &rx_config, sizeof(rx_config));
	        tx_packet(rxc_buf, sizeof(rx_config) + 1);
	      }
	      break;
	      case 't': { // GET version & outputs/pins info
	        timeout = 0;
	        rxc_buf[0] = 'T';
	        rxc_buf[1] = (version >> 8);
	        rxc_buf[2] = (version & 0xff);
	        rxc_buf[3] = 13;
	        rxc_buf[4] = rxSpecialPinsSize / sizeof(rxSpecialPins[0]);
	        memcpy((rxc_buf + 5), &rxSpecialPins, rxSpecialPinsSize);
	        tx_packet(rxc_buf, rxSpecialPinsSize + 5);
	      }
	      break;
	      case 'u': { // SET save rx_config to eeprom
	        memcpy(&rx_config, (rxc_buf + 1), sizeof(rx_config));
	        rxWriteEeprom();
	        rxc_buf[0] = 'U';
	        tx_packet(rxc_buf, 1);
	      }
	      break;
      }
      rx_reset();
    }
  }
  return 0;
}

u8 beaconGetRSSI(void)
{ 
  u16 rssiSUM = 0;
  LED_G_ON;

  rfmSetChannel(0);
  rfmSetCarrierFrequency(rx_config.beacon_frequency);

  BSP_OS_TimeDly(1);
  rssiSUM += rfmGetRSSI();
  BSP_OS_TimeDly(1);
  rssiSUM += rfmGetRSSI();
  BSP_OS_TimeDly(1);
  rssiSUM += rfmGetRSSI();
  BSP_OS_TimeDly(1);
  rssiSUM += rfmGetRSSI();

  LED_G_OFF;
  return rssiSUM>>2;
}


void handleBeacon(void)
{
  u8 brssi;
  OS_ERR  err;
  if ((rx_config.beacon_frequency) && (nextBeaconTimeMs)) {
    // Listen for RSSI on beacon channel briefly for 'trigger'
    brssi = beaconGetRSSI();
    if (brssi > ((beaconRSSIavg>>2) + 20)) {
      nextBeaconTimeMs = OSTimeGet(&err) + 1000L;
    }
    beaconRSSIavg = (beaconRSSIavg * 3 + brssi * 4) >> 2;
    rfmSetCarrierFrequency(bind_data.rf_frequency);
  }
}

void updateHopChannel(void)
{
//	OS_ERR  err;
  if (willhop == 1) {
    rf_channel++;
    if ((rf_channel == MAXHOPS) || (bind_data.hopchannel[rf_channel] == 0)) {
      rf_channel = 0;
    }
    //handleBeacon();
    setHopChannel(rf_channel);

    willhop = 0;
	//MSG("hop channel,rf=%d,@%d\r\n",rf_channel,OSTimeGet(&err));
  }
}

u8 getLedStatus(void)
{
	u8 ret=0;

	if(button_value_get(BUTTON_SRC_POWER_CTRL1) == 1){
		ret |= POWER_LED;
	}else{
		ret &= ~POWER_LED;
	}
	if(button_value_get(BUTTON_SRC_RETURN_LED) == 1){
		ret |= RETURN_LED;
	}else{
		ret &= ~RETURN_LED;
	}
	return ret;
}

void rx_pack()
{
//	u8 i;
	static u32 number=0;
	
	Tx_buf[0] = RX_HEAD;
	Tx_buf[1] = number++;
	Tx_buf[2] = getLedStatus(); // led status fedback for TX module
	Tx_buf[3] = countSetBits(linkQuality & 0x7fff);     	//link quality
	Tx_buf[4] = lastRSSIvalue;	   	//rssi
	Tx_buf[5] = lastAFCCvalue >> 8;		//afcc
	Tx_buf[6] = lastAFCCvalue&0xff;
	Tx_buf[7] =checkbufferdata(Tx_buf,6);

	Tx_buf[8] = RX_TAIL;
}
void handlePacketTelem(void)
{
//	OS_ERR	err;
	
	rx_pack();
	tx_packet_async(Tx_buf, 9);
	while(!tx_done()) {
	//checkSerial();
	}
	//MSG("handle packet telem finished %d\r\n",OSTimeGet(&err));

}

void unpackChannels(void)
{
//	u8 i;

	button_output[0] =Rx_buf[11]&SHUTTLE_ADJ2;
	button_output[1] =Rx_buf[11]&SHUTTLE_ADJ1;
	button_output[2] =Rx_buf[11]&SHUTTLE_BUTTON;
	button_output[3] =Rx_buf[11]&PHOTO_BUTTON;
	button_output[4] =Rx_buf[11]&PLAYBACK_BUTTON;
	button_output[5] =Rx_buf[11]&MODE_SET1;
	button_output[6] =Rx_buf[11]&MODE_SET2;
	button_output[7] =Rx_buf[11]&VIDEO_BUTTON;

	button_output[8] =Rx_buf[12]&POWER_BUTTON;
	button_output[9] =Rx_buf[12]&RETURN_RESERVE2;
	button_output[10] =Rx_buf[12]&RETURN_TOGGLE;
	button_output[11] =Rx_buf[12]&RETURN_RESERVE1;
	button_output[12] =Rx_buf[12]&RETURN_BUTTON;

	//for(i=0;i<13;i++)
	//	MSG("%d,",button_output[i]);
	//MSG("\r\n");
	
}

void gpioSetOutput(u8 *gpio,u8 size)
{
	u8 i=0;
	while(size --){
		if(*(gpio++)){
			gpio_value_set(i);
		}else{
			gpio_value_reset(i);
		}
		if(i<GPIO_SRC_NUM)
			i++;
	}
}

void handlePacketRX(void)
{
  OS_ERR  err;
//  u8 i;
//  static u32 count;
  
  if (RF_Mode == RECEIVED) {
		u32 timeTemp = OSTimeGet(&err);
		//MSG("receive data enter count=%d @%d\r\n",count++,timeTemp);
		rfmGetPacket(Rx_buf, 16);
		lastAFCCvalue = rfmGetAFCC();
		LED_G_ON;

		lastPacketTimeMs = timeTemp;
		numberOfLostPackets = 0;
		//nextBeaconTimeMs = 0;
		linkQuality <<= 1;
		linkQuality |= 1;
		LED_R_OFF;
		
		if (Rx_buf[0] == TX_HEAD && Rx_buf[15] == TX_TAIL && Rx_buf[14] == checkbufferdata(Rx_buf,13)) {
			pwm[PWM_CHANNEL_CAMERA] = (Rx_buf[1]<<8) + Rx_buf[2];
			pwm[PWM_CHANNEL_JS_R1] = (Rx_buf[3]<<8) + Rx_buf[4];
			pwm[PWM_CHANNEL_JS_R2] = (Rx_buf[5]<<8) + Rx_buf[6];
			pwm[PWM_CHANNEL_JS_L1] = (Rx_buf[7]<<8) + Rx_buf[8];
			pwm[PWM_CHANNEL_JS_L2] = (Rx_buf[9]<<8) + Rx_buf[10];

			//MSG("---%d,%d,%d,%d,%d---\r\n",pwm[PWM_CHANNEL_CAMERA],pwm[PWM_CHANNEL_JS_R1],pwm[PWM_CHANNEL_JS_R2],pwm[PWM_CHANNEL_JS_L1],pwm[PWM_CHANNEL_JS_L2]);
			TIM_SetCompare1(TIM5,pwm[PWM_CHANNEL_CAMERA]);
			TIM_SetCompare1(TIM3,pwm[PWM_CHANNEL_JS_R1]);
			TIM_SetCompare2(TIM3,pwm[PWM_CHANNEL_JS_R2]);
			TIM_SetCompare3(TIM3,pwm[PWM_CHANNEL_JS_L1]);
			TIM_SetCompare4(TIM3,pwm[PWM_CHANNEL_JS_L2]);

			unpackChannels();
			gpioSetOutput(button_output,13);

			#if 0
			numberOfPacket = Rx_buf[13];
			//MSG("---------------------%d---------------------\r\n",numberOfPacket);
			if(fristPacket == 1)
			{
				if(numberOfPacket < lastNumberOfPacket) //超量程，从零开始计数
				{
					if(numberOfPacket+255- lastNumberOfPacket != 1) //有丢包
					{
						numberOfLostPackets += numberOfPacket+255-lastNumberOfPacket;
						//numberOfLostPackets++;
					}
				}else{
					if(numberOfPacket - lastNumberOfPacket != 1) //丢包
					{
						numberOfLostPackets += numberOfPacket - lastNumberOfPacket;
						//numberOfLostPackets++;
					}
				}
			}

			lastNumberOfPacket = numberOfPacket;
			if(fristPacket == 0)
				fristPacket = 1;
			if(numberOfLostPackets){
				linkQuality <<= 1;
				MSG("number of lost packet= %d\r\n",numberOfLostPackets);
			}
			#endif
		}

      if(fristPacket == 0)
		fristPacket= 1;

      if (bind_data.flags & TELEMETRY_MASK) {
        handlePacketTelem();
      }

      willhop = 1;
      rx_reset();
      LED_G_OFF;

	}
}

void checkRSSI(void)
{
  // sample RSSI when packet is in the 'air'
  if ((numberOfLostPackets < 2) && (lastRSSITimeMs != lastPacketTimeMs) && (pktTimeDelta > RSSI_timeout)) {
  	//MSG("----checkRSSI-----\r\n");
    lastRSSITimeMs = lastPacketTimeMs;
    lastRSSIvalue = rfmGetRSSI(); // Read the RSSI value
#if 1
    RSSI_sum += lastRSSIvalue;    // tally up for average
    RSSI_count++;

    if (RSSI_count > 20) {
      RSSI_sum /= RSSI_count;
	  RSSI_sum = constrain(RSSI_sum, 45, 255);
      smoothRSSI = (u8)(((u16)smoothRSSI * 8 + RSSI_sum * 2) / 8);
      //set_RSSI_output();
      RSSI_sum = 0;
      RSSI_count = 0;
    }
#endif
  }
}

void checkLinkState(void)
{
  if ((numberOfLostPackets < hopcount) && (pktTimeDelta > hopTimeout)) {
    // we lost a packet, so hop to next channel
    //MSG("------------------------lost packet------------------------------\r\n");
	//MSG("numberOfLostPackets=%d\r\n",numberOfLostPackets);
	//MSG("pktTimeDelta=%d\r\n",pktTimeDelta);
    linkQuality <<= 1;
    willhop = 1;
    numberOfLostPackets++;
    lastPacketTimeMs += hopInterval;
    willhop = 1;
    LED_R_ON;
  } else if ((numberOfLostPackets == hopcount) && (pktTimeDelta > hopTimeoutSlow)) {
    // hop slowly to allow re-sync with TX
    linkQuality = 0;
    willhop = 1;
    smoothRSSI = 0;
    lastPacketTimeMs = timeMs;
  }

  if (numberOfLostPackets) {
    //handleFailsafe();
   // MSG("need lost packet handler\r\n");
  }
}


extern vu32 systick;
static u32 lost_count=0;
static u32 recovery=0;
static u8 doReturn=0;

void do_return_action(void)
{
	//拉低500ms，再拉高500ms，然后再拉低3s，产生一个返航信号
	gpio_value_reset(GPIO_SRC_RETURN);
	BSP_OS_TimeDly(500);
	gpio_value_set(GPIO_SRC_RETURN);
	BSP_OS_TimeDly(500);
	gpio_value_reset(GPIO_SRC_RETURN);
	BSP_OS_TimeDly(3000);
	gpio_value_set(GPIO_SRC_RETURN);
}

void RXloop(void)
{
	OS_ERR  err;
	
	check_module();

	//handle receive packet
	handlePacketRX();
	

	timeMs = OSTimeGet(&err);

	pktTimeDelta = (timeMs - lastPacketTimeMs);
	//MSG("pktTimeDelta= %d\r\n",pktTimeDelta);
	
	checkRSSI();
#if 1
	if (fristPacket) {
	    // check RC link status after initial 'lock'
	    checkLinkState();
		/*做失控返航保护逻辑*/
		if(linkQuality < 8 && systick > 40)	//开机100s之内不执行返航操作
		{
			lost_count++;
			//1秒内链路质量连续过低，自动返航
			if(lost_count > 500 && doReturn ==0)
			{
				doReturn=1;
				MSG("linkquality too bad,do return action\r\n");
				do_return_action();		//产生返航动作
			}
			
		}else{
			lost_count=0;
			#if 1
			if(doReturn == 1 && linkQuality > 12){
				recovery++;
				if(recovery > 500){
					doReturn=0;
					recovery=0;
					MSG("linkquailty recovery\r\n");
				}
			}else{
				recovery=0;
			}
			#endif
		}
		
		//MSG("-------------\r\n");
    } else if (pktTimeDelta > hopTimeoutSlow) {
	    // Still waiting for first packet, so hop slowly
	    MSG("have not receive packet,so hop slow\r\n");
		linkQuality = 0;
	    lastPacketTimeMs = timeMs;
	    willhop = 1;
    }
#endif

	updateHopChannel();
	//MSG("out loop @%d\r\n",OSTimeGet(&err));

}


void RXsetup(void)
{
	OS_ERR  err;
//	u8 i;
	
	rxReadEeprom();
	bindReadEeprom();

	MSG("OpenLRS RX starting\r\n");
	MSG("RX type %d\r\n",rx_config.rx_type);

	LED_R_ON;

	BSP_OS_TimeDly(100);

	if(checkIfConnected(0,1))
	{
		MSG("force bind\r\n");
		if(bindReceive(0)){
			rxWriteEeprom();
			MSG("save bind data to flash\r\n");
			LED_G_ON;
		}
	}else if(rx_config.flags & ALWAYS_BIND){
		if (bindReceive(2000)) {
			rxWriteEeprom();
			MSG("save bind data to flash\r\n");
			LED_G_ON;
		}
	}

	

	MSG("Entering normal mode\r\n");
	// Count hopchannels as we need it later
	hopcount=0;
	while ((hopcount < MAXHOPS) && (bind_data.hopchannel[hopcount] != 0)) {
		MSG("openLRS hop channel %d\r\n",bind_data.hopchannel[hopcount]);
    	hopcount++;
  	}

	//################### RX SYNC AT STARTUP #################
	init_rfm(0);   // Configure the RFM22B's registers for normal operation
	rf_channel = 0;
  	setHopChannel(rf_channel);
  	rx_reset();

	linkAcquired = 0;
	hopInterval = getInterval(&bind_data)/1000;
	hopTimeout = hopInterval + 1;
	hopTimeoutSlow = hopInterval * hopcount;
	RSSI_timeout = hopInterval - 1;
	MSG("hopInterval=%d\r\n",hopInterval);
	MSG("hopTimeout=%d\r\n",hopTimeout);
	MSG("hopTimeoutSlow=%d\r\n",hopTimeoutSlow);
	MSG("RSSI_timeout=%d\r\n",RSSI_timeout);
	lastPacketTimeMs = OSTimeGet(&err);
	//MSG("lastPacketTimeMs=%d\r\n",lastPacketTimeMs);

}

#endif


