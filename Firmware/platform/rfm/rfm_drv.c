#include "common.h"
#include "stm32f4xx_gpio.h"
#include "rfm_drv.h"
#include "uart_drv.h"
#include "spi_drv.h"
#include "bsp_os.h"
#include "bsp.h"
#include "binding.h"
#include "rfm_common.h"

u32 counter=0;

void EXTI2_IRQHandler(void)
{
	OSIntEnter();
	if (RF_Mode == TRANSMIT) {
    	RF_Mode = TRANSMITTED;
  	} else if (RF_Mode == RECEIVE) {
   		RF_Mode = RECEIVED;
  	}

	EXTI_ClearITPendingBit(EXTI_Line2);
	OSIntExit(); 
}



/*----------------------------------------------------------------------------*/
static void setupRFMints(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    GPIO_InitStructure.GPIO_Pin = RFM_INT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(RFM_INT_GPIO, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
	
    // Enable and set EXTI1 Interrupt to the lowest priority
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	BSP_IntVectSet(BSP_INT_ID_EXTI2,EXTI2_IRQHandler);
	BSP_IntEn(BSP_INT_ID_EXTI2);
}

void rfm_preinit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 
	//SPI NSS GPIOB3
	GPIO_InitStructure.GPIO_Pin = RFM_NSS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(RFM_NSS_GPIO, &GPIO_InitStructure);

	SPI3_drv_Init();

	setupRFMints();
}


u8 spiReadRegister(u8 reg)
{
	u8 ret;
    RFM_CS_ON;
	//MSG("read RFM%d(%x)\r\n",which,reg);
	SPI_ReadWriteByte(SPI3,reg);
	ret = SPI_ReadWriteByte(SPI3,0xFF);
	RFM_CS_OFF;
	//MSG("read reg(%x),return %x\r\n",reg,ret);
	return ret;
}

void spiWriteRegister(u8 reg, u8 data)
{  
    RFM_CS_ON;
	SPI_ReadWriteByte(SPI3,(reg&0x7f)+0x80);
	SPI_ReadWriteByte(SPI3,data);
    RFM_CS_OFF;
	//MSG("write reg(%x)=%x finish\r\n",reg,data);
}


void rfmInit(u8 diversity)
{
  spiWriteRegister(RFM22B_INTEN2, 0x00);    // disable interrupts
  spiWriteRegister(RFM22B_INTEN1, 0x00);    // disable interrupts
  spiWriteRegister(RFM22B_XTALCAP, 0x7F);   // XTAL cap = 12.5pF
  spiWriteRegister(RFM22B_MCUCLK, 0x05);    // 2MHz clock

  spiWriteRegister(RFM22B_GPIOCFG2, 0xFD ); // gpio 2 ant. sw, 1 if diversity on else VDD
  spiWriteRegister(RFM22B_PREAMLEN, 0x0A );    // 40 bit preamble, 80 with diversity
  spiWriteRegister(RFM22B_IOPRTCFG, 0x00);    // gpio 0,1,2 NO OTHER FUNCTION.

  spiWriteRegister(RFM22B_GPIOCFG0, 0x12);    // gpio0 TX State
  spiWriteRegister(RFM22B_GPIOCFG1, 0x15);    // gpio1 RX State

  // Packet settings
  spiWriteRegister(RFM22B_DACTL, 0x8c);    // enable packet handler, msb first, enable crc,
  spiWriteRegister(RFM22B_HDRCTL1, 0x0F);    // no broadcast, check header bytes 3,2,1,0
  spiWriteRegister(RFM22B_HDRCTL2, 0x42);    // 4 byte header, 2 byte sync, variable packet size
  spiWriteRegister(RFM22B_PREATH, 0x2A);    // preamble detect = 5 (20bits), rssioff = 2
  spiWriteRegister(RFM22B_SYNC3, 0x2D);    // sync word 3
  spiWriteRegister(RFM22B_SYNC2, 0xD4);    // sync word 2
  spiWriteRegister(RFM22B_SYNC1, 0x00);    // sync word 1 (not used)
  spiWriteRegister(RFM22B_SYNC0, 0x00);    // sync word 0 (not used)
  spiWriteRegister(RFM22B_HDREN3, 0xFF);    // must set all bits
  spiWriteRegister(RFM22B_HDREN2, 0xFF);    // must set all bits
  spiWriteRegister(RFM22B_HDREN1, 0xFF);    // must set all bits
  spiWriteRegister(RFM22B_HDREN0, 0xFF);    // must set all bits

  spiWriteRegister(RFM22B_FREQOFF1, 0x00);    // no offset
  spiWriteRegister(RFM22B_FREQOFF2, 0x00);    // no offset
  spiWriteRegister(RFM22B_FHCH,     0x00);   // set to hop channel 0
}

void rfmClearFIFO(u8 diversity)
{
  //clear FIFO, disable multi-packet, enable diversity if needed
  //requires two write ops, set & clear
  spiWriteRegister(RFM22B_OPMODE2, (diversity ? 0x83 : 0x03) );
  spiWriteRegister(RFM22B_OPMODE2, (diversity ? 0x80 : 0x00) );
}

void rfmClearInterrupts(void)
{
  spiWriteRegister(RFM22B_INTEN1, 0x00);
  spiWriteRegister(RFM22B_INTEN2, 0x00);
}

void rfmClearIntStatus(void)
{
  spiReadRegister(RFM22B_INTSTAT1);
  spiReadRegister(RFM22B_INTSTAT2);
}

void rfmSendPacket(u8* pkt, u8 size)
{
  u8 i;
  spiWriteRegister(RFM22B_PKTLEN, size);   // total tx size
  for (i = 0; i < size; i++) {
    spiWriteRegister(RFM22B_FIFO, pkt[i]);
  }
  spiWriteRegister(RFM22B_INTEN1, RFM22B_PACKET_SENT_INTERRUPT);
}

u16 rfmGetAFCC(void)
{
  return (((u16) spiReadRegister(RFM22B_AFC0) << 2) | ((u16) spiReadRegister(RFM22B_AFC1) >> 6));
}

u8 rfmGetGPIO1(void)
{
  return spiReadRegister(RFM22B_GPIOCFG1);
}

u8 rfmGetRSSI(void)
{
  return spiReadRegister(RFM22B_RSSI);
}

u8 rfmGetPacketLength(void)
{
  return spiReadRegister(RFM22B_RXPLEN);
}

void rfmGetPacket(u8 *buf, u8 size)
{
    while (size--)
    {
        *(buf++) = spiReadRegister(RFM22B_FIFO);
    }
}

void rfmSetTX(void)
{
  spiWriteRegister(RFM22B_OPMODE1, (RFM22B_OPMODE_TX | RFM22B_OPMODE_READY));
  //BSP_OS_TimeDly(1); // allow for PLL & PA ramp-up, ~200us
 // spiReadRegister(RFM22B_OPMODE1);
}

void rfmSetRX(void)
{
  spiWriteRegister(RFM22B_INTEN1, RFM22B_RX_PACKET_RECEIVED_INTERRUPT);
  spiWriteRegister(RFM22B_OPMODE1, (RFM22B_OPMODE_RX | RFM22B_OPMODE_READY));
  //BSP_OS_TimeDly(1);  // allow for PLL ramp-up, ~200us
  //spiReadRegister(RFM22B_OPMODE1);
}

void rfmSetCarrierFrequency(u32 f)
{
  u16 fb, fc, hbsel;
  if (f < 480000000) {
    hbsel = 0;
    fb = f / 10000000 - 24;
    fc = (f - (fb + 24) * 10000000) * 4 / 625;
  } else {
    hbsel = 1;
    fb = f / 20000000 - 24;
    fc = (f - (fb + 24) * 20000000) * 2 / 625;
  }
  spiWriteRegister(RFM22B_BANDSEL, 0x40 + (hbsel ? 0x20 : 0) + (fb & 0x1f));
  spiWriteRegister(RFM22B_CARRFREQ1, (fc >> 8));
  spiWriteRegister(RFM22B_CARRFREQ0, (fc & 0xff));
  BSP_OS_TimeDly(200); // VCO / PLL calibration delay
}

void rfmSetChannel(u8 ch)
{
  spiWriteRegister(RFM22B_FHCH, ch);
}

void rfmSetDirectOut(u8 enable)
{
 static u8 r1 = 0, r2 = 0, r3 = 0;
  if (enable) {
    r1 = spiReadRegister(RFM22B_DACTL);
    r2 = spiReadRegister(RFM22B_MODCTL2);
    r3 = spiReadRegister(RFM22B_FREQDEV);
    // setup for direct output, i.e. beacon tones
    spiWriteRegister(RFM22B_DACTL, 0x00);    //disable packet handling
    spiWriteRegister(RFM22B_MODCTL2, 0x12);    // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
    spiWriteRegister(RFM22B_FREQDEV, 0x02);    // fd (frequency deviation) 2*625Hz == 1.25kHz
  } else {
    // restore previous values
    spiWriteRegister(RFM22B_DACTL, r1);
    spiWriteRegister(RFM22B_MODCTL2, r2);
    spiWriteRegister(RFM22B_FREQDEV, r3); 
  }
}

void rfmSetHeader(u8 iHdr, u8 bHdr)
{
  spiWriteRegister(RFM22B_TXHDR3+iHdr, bHdr);
  spiWriteRegister(RFM22B_CHKHDR3+iHdr, bHdr);
}



void rfmSetPower(u8 power)
{
  spiWriteRegister(RFM22B_TXPOWER, power);
  BSP_OS_TimeDly(25); // PA ramp up/down time
}

void rfmSetReadyMode(void)
{
  spiWriteRegister(RFM22B_OPMODE1, RFM22B_OPMODE_READY);
}

void rfmSetStepSize(u8 sp)
{
  spiWriteRegister(RFM22B_FHS, sp);
}

