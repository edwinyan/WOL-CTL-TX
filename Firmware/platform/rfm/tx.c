#include "common.h"
#include "stm32f4xx_gpio.h"
#include "tx.h"
#include "uart_drv.h"
#include "rfm_drv.h"
#include "rfm_common.h"
#include "button_drv.h"
#include "buzzer_drv.h"
#include "bsp_os.h"
#include "bsp.h"
#include "led_drv.h"
#include "adc_drv.h"
#include "gpio_drv.h"
#include "datalink_drv.h"

#if (TX == 1)

u32 lastTelemetry = 0;
u8 RSSI_rx = 0;
u8 RSSI_tx = 0;
u32 sampleRSSI = 0;


u16 LinkQuality = 0;
u16 LinkQualityRX = 0;
u32 lastSent = 0;


static u8 Rf_channel=0;

u32 test_value=0;

/*byte0 	head
  byte1-2	camera adc output
  byte3-4	joystick left channel1
  byte5-6	joystick left channel2
  byte7-8	joystick right channel1
  byte9-10	joystick right channel2
  byte11	8 channels button status output
  byte12	5 channels button status output
  byte13	number of send message
  byte14	checksum
  byte15	tail
*/
//u8 tx_buf[MAX_PACKETSIZE];
u8 rx_buf[TELEMETRY_PACKETSIZE];

//#define ADC_COMPENSATE	(3.276f/3.304f)   //发送端(3.276v)和接收端(3.304v)的adc参考电压不一致，需要做补偿
#define ADC_COMPENSATE	(3.304f/3.274f)   //发送端(3.276v)和接收端(3.304v)的adc参考电压不一致，需要做补偿

extern u8 tx_buf[MAX_PACKETSIZE];
/*----------------------------------------------------------------------------*/

void swap(u8 *a, u8 i, u8 j)
{
  u8 c = a[i];
  a[i] = a[j];
  a[j] = c;
}

void isort(u8 *a, u8 n)
{
  u8 i,j;
  for (i = 1; i < n; i++) {
    for (j = i; j > 0 && a[j] < a[j-1]; j--) {
      swap(a, j, j - 1);
    }
  }
}

void bindRandomize(u8 randomChannels)
{
  u8 emergency_counter = 0;
  u8 c;
  u32 t = 0;
  u8 ch;
  u8 i;
  u32 real_frequency;
  OS_ERR  err;

  MSG("entering bindRandomize \r\n");
  
  while (t == 0) {
    t = OSTimeGet(&err);
  }
  srand(t);

  bind_data.rf_magic = 0;
  for (c = 0; c < 4; c++) {
    bind_data.rf_magic = (u32)(bind_data.rf_magic << 8) + (rand() % 255);
  }
  MSG("rf magic = %d\r\n",bind_data.rf_magic);
  if (randomChannels) {
    /*TODO: verify if this works properly */
    for (c = 0; (c < MAXHOPS) && (bind_data.hopchannel[c] != 0); c++) {
again:
      if (emergency_counter++ == 255) {
        bindInitDefaults();
        return;
      }

      ch = (rand() % 50) + 1;

      // don't allow same channel twice
      for (i = 0; i < c; i++) {
        if (bind_data.hopchannel[i] == ch) {
          goto again;
        }
      }

      // don't allow frequencies higher then tx_config.max_frequency
      real_frequency = bind_data.rf_frequency + (u32)ch * (u32)bind_data.rf_channel_spacing * 10000UL;
      if (real_frequency > tx_config.max_frequency) {
        goto again;
      }

      bind_data.hopchannel[c] = ch;
	  MSG("ch=%d\r\n",ch);
    }
  }
}


u8 chooseChannelsPerRSSI(void)
{
  u8 chRSSImax[255];
  u8 picked[20];
  u8 n,i,lowest,lowestRSSI;
  u8 ch;
  u32 start;
  OS_ERR  err;
  u8 rssi,j,k;

  for (n = 0; (n < MAXHOPS) && (bind_data.hopchannel[n] != 0); n++);

  MSG("Entering adaptive channel selection, picking:%d\r\n",n);
  init_rfm(0);
  rx_reset();
  for (ch=1; ch<255; ch++) {
    start = OSTimeGet(&err);
    if ((bind_data.rf_frequency + (u32)ch * (u32)bind_data.rf_channel_spacing * 10000UL) > tx_config.max_frequency) {
      chRSSImax[ch] = 255;
      continue; // do not break so we set all maxes to 255 to block them out
    }
    rfmSetChannel(ch);
	chRSSImax[ch] = 0;
    while ((OSTimeGet(&err) - start) < 100) {
      rssi = rfmGetRSSI();
	  //MSG("ch = %d,rssi=%d\r\n",ch,rssi);
      if (rssi > chRSSImax[ch]) {
        chRSSImax[ch] = rssi;
      }
    }
	
    if (ch & 1) {
      LED_W_OFF;
      LED_R_ON;
    } else {
      LED_W_ON;
      LED_R_OFF;
    }
  }

  for (i = 0; i < n; i++) {
  	lowestRSSI = 255;
	lowest = 1;
    for (ch = 1; ch < 255; ch++) {		
      if (chRSSImax[ch] < lowestRSSI) {
        lowestRSSI = chRSSImax[ch];
        lowest = ch;
      }
    }
	
	//MSG("lowest = %d\r\n",lowest);
    picked[i] = lowest;
    chRSSImax[lowest] = 255;
    if (lowest > 1) {
      chRSSImax[lowest - 1]=255;
    }
    if (lowest > 2) {
      chRSSImax[lowest - 2]=255;
    }
    if (lowest < 254) {
      chRSSImax[lowest + 1]=255;
    }
    if (lowest < 253) {
      chRSSImax[lowest + 2]=255;
    }
  }

  isort(picked, n);

  // this is empirically a decent way to shuffle changes to give decent hops
  for (j = 0; j < (n / 2); j += 2) {
    swap(picked, j, j + n / 2);
  }

  MSG("new pick value:");
  for (k = 0; k < n; k++) {
	MSG("%d,",picked[k]);
    bind_data.hopchannel[k] = picked[k];
  }
  MSG("\r\n");

  return 1;
}

void bindMode(void)
{
  OS_ERR  err;
  u8  tx_buf[sizeof(bind_data) + 1];
  u8  sendBinds = 1;
  u32 prevsend = OSTimeGet(&err);

  tx_buf[0] = 'b';
  memcpy(tx_buf + 1, &bind_data, sizeof(bind_data));

  init_rfm(1);

  //consoleFlush();

  LED_R_OFF;

  while (1) {
    if (sendBinds & (OSTimeGet(&err) - prevsend > 200)) {
      prevsend = OSTimeGet(&err);
      LED_G_ON;
      buzzer(ON);
      tx_packet(tx_buf, sizeof(bind_data) + 1);
      LED_G_OFF;
      buzzer(OFF);
      rx_reset();
      BSP_OS_TimeDly(50);
      if (RF_Mode == RECEIVED) {
        rfmGetPacket(tx_buf, 1);
        if (tx_buf[0] == 'B') {
          sendBinds = 0;
		  MSG("bind success!!!\r\n");
		  break;
        }
      }
    }
	
    if (!button_value_get(BUTTON_SRC_PAIR)) {
      sendBinds = 1;
    }

  }
}


void checkButton(void)
{
	u32 time, loop_time;
	OS_ERR  err;
	u8 bzstate = 1;
    u8 swapProfile = 0;

	if (button_value_get(BUTTON_SRC_PAIR) == 0) {     // Check the button
    	BSP_OS_TimeDly(200);   // wait for 200mS with buzzer ON
    	buzzer(OFF);

    time = OSTimeGet(&err);  //set the current time
    loop_time = time;

    while (OSTimeGet(&err) < time + 4800) {
      if (button_value_get(BUTTON_SRC_PAIR)) {
	  	MSG("go to just bind\r\n");
        goto just_bind;
      }
    }

    // Check the button again, If it is still down reinitialize
    if (0 == button_value_get(BUTTON_SRC_PAIR)) {


      buzzer(bzstate?1:0);
      loop_time =OSTimeGet(&err);

      while (0 == button_value_get(BUTTON_SRC_PAIR)) {     // wait for button to release
        if (loop_time > time + 9800) {
          buzzer(ON);
          swapProfile = 1;
		  MSG("goto swap profile\r\n");
        } else {
          if ((OSTimeGet(&err) - loop_time) > 200) {
            loop_time = OSTimeGet(&err);
            bzstate = !bzstate;
            buzzer(bzstate ? 1 : 0);
          }
        }
      }

      buzzer(OFF);
      if (swapProfile) {	  	
        //setDefaultProfile((defaultProfile + 1) % (TX_PROFILE_COUNT+1));
        //setupProfile();
        txReadEeprom();
        BSP_OS_TimeDly(500);
        return;
      }
      bindRandomize(0);
      chooseChannelsPerRSSI();
      txWriteEeprom();
    }
just_bind:
    // Enter binding mode, automatically after recoding or when pressed for shorter time.
    bindMode();
  }
}

void doBeeps(u8 numBeeps) {
  u8 i;
  for (i = 0; i <= numBeeps; i++) {
    BSP_OS_TimeDly(50);
    buzzer(ON);
    BSP_OS_TimeDly(50);
    buzzer(OFF);
  }
}

#if 1

void TXloop(void)
{
	OS_ERR  err;
	u32 time=0;
	u8 i=0;
	//static u8 number=0;
	//static u8 pre_number=0;
	//u32 timestamp;
	
	check_module();

	
	if (RF_Mode == RECEIVED) {
//		MSG("receive\r\n");
#if 1
		// got telemetry packet
		lastTelemetry = OSTimeGet(&err);
		//MSG("lastTelemetry = %d\r\n",lastTelemetry);
		
		if (!lastTelemetry) {
			lastTelemetry = 1; //fixup rare case of zero
		}
		LinkQuality |= 1;

		rfmGetPacket(rx_buf, TELEMETRY_PACKETSIZE);

		if (rx_buf[0] == RX_HEAD && rx_buf[8] == RX_TAIL && rx_buf[7] == checkbufferdata(rx_buf,6)) 
		{
			//RSSI_rx = rx_buf[1];
			LinkQualityRX = rx_buf[3];
			if(rx_buf[2]&RETURN_LED){
				GPIO_SetBits(GPIOB, GPIO_Pin_9);
			}else{
				GPIO_ResetBits(GPIOB, GPIO_Pin_9);
			}
			if(rx_buf[2]&POWER_LED){
				GPIO_SetBits(GPIOC, GPIO_Pin_9);
			}else{
				GPIO_ResetBits(GPIOC, GPIO_Pin_9);
			}
			#if 0
			for(i=1;i<7;i++){
				MSG("%d,",rx_buf[i]);
			}
			MSG("\r\n");
			#endif
		}
#endif
	}

	time = OSTimeGet(&err);
	//MSG("time = %d\r\n",time);

	if ((sampleRSSI) && ((time - sampleRSSI) >= 3)) {
    	RSSI_tx = rfmGetRSSI();
    	sampleRSSI = 0;
  	}

	// TX发送数据的时间间隔不大于getInterval(&bind_data)，即发送频率不大于1/getInterval(&bind_data)。
  	if ((time - lastSent) >= getInterval(&bind_data)/1000) {
	//if ((time - lastSent) >= TX_CYCLE) {
		//MSG("send use time = %d\r\n",(time - lastSent));
		//timestamp = OSTimeGet(&err);
		lastSent = time;
		//MSG("send config start %d\r\n",OSTimeGet(&err));

		if (lastTelemetry) {
        	if ((time - lastTelemetry) > getInterval(&bind_data)/1000) {
          		// telemetry lost
          		if (!(tx_config.flags & MUTE_TX)) {
            		buzzer(ON);
          		}
         		lastTelemetry = 0;
        	} else {
          		// telemetry link re-established
          		buzzer(OFF);
        	}
      	}
		//MSG("pack start %d\r\n",OSTimeGet(&err));
		packChannels();
		//MSG("pack end %d\r\n",OSTimeGet(&err));
		LED_W_ON;

		// Send the data over RF
		setHopChannel(Rf_channel);
		//MSG("-- 1 --%d\r\n",OSTimeGet(&err));
		tx_packet_async(tx_buf, 16);
		//MSG("------------------tx send pack----------------------\r\n");
		//MSG("-- 2 --%d\r\n",OSTimeGet(&err));
      	//Hop to the next frequency
      	Rf_channel++;
		if ((Rf_channel == MAXHOPS) || (bind_data.hopchannel[Rf_channel] == 0)) {
        	Rf_channel = 0;
      	}
		//MSG("send use time = %d\r\n",(OSTimeGet(&err) - time));
		//MSG("send\r\n");
  	}

	if (tx_done() == 1) {
	    if (bind_data.flags & TELEMETRY_MASK) {
      		LinkQuality <<= 1;
      		rx_reset();
    	}
  	}
	//发送完成
	LED_W_OFF;
	//MSG("LinkQualityRX= %d\r\n",LinkQualityRX);
	//MSG("send  end %d\r\n",OSTimeGet(&err));
}

#endif

void printconf()
{
	MSG("tx_config console_baud_rate=%d\r\n",tx_config.console_baud_rate);
	MSG("tx_config flags=%d\r\n",tx_config.flags);
	MSG("tx_config max_frequency=%d\r\n",tx_config.max_frequency);
	MSG("tx_config rfm_type=%d\r\n",tx_config.rfm_type);
	MSG("bind_data flags=%d\r\n",bind_data.flags);
	MSG("bind_data serial_baudrate=%d\r\n",bind_data.serial_baudrate);
	MSG("bind_data rf_channel_spacing=%d\r\n",bind_data.rf_channel_spacing);
	MSG("bind_data rf_magic=%d\r\n",bind_data.rf_magic);
	MSG("bind_data version=%d\r\n",bind_data.version);
	MSG("bind_data rf_power=%d\r\n",bind_data.rf_power);	
}

void TXsetup(void)
{
	//OS_ERR  err;

	//tx_gpio_init();
	
	txReadEeprom();
	bindReadEeprom();

	printconf();

	LED_R_ON;

	MSG("openLRS TX starting\r\n");
	MSG("openLRS version 0x%x\r\n",bind_data.version);

	BSP_OS_TimeDly(100);

	//check button for bind
	checkButton();

	MSG("Entering normal mode\r\n");
	LED_R_OFF;
	buzzer(OFF);

	txReadEeprom();
	doBeeps(2);

	init_rfm(0);
	setHopChannel(Rf_channel);
	rx_reset();
}


#endif

