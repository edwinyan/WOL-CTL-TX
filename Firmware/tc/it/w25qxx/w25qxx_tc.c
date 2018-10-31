#include "common.h"
#include "w25qxx_drv.h"
#include "w25qxx_tc.h"
#include "tc_common.h"
#include "bsp.h"
#include "bsp_os.h"

const u8 TEXT_Buffer[]={"Explorer STM32F4 SPI TEST"};
//const u8 TEXT_Buffer[] ={"123456789"};

#define SIZE sizeof(TEXT_Buffer)

STATIC void w25qxx_tc_01(void)
{
    u8 datatemp[SIZE];
	u32 FLASH_SIZE;
	u8 i;

    tc_printf_banner("DBG w25qxx test ");
	
    W25QXX_Init();

	if(W25QXX_ReadID()!=W25Q128)
	{
		MSG("w25qxx check failed");

		return ;
	}
	MSG("ID %x \r\n",W25QXX_ReadID());
	#if 1
	FLASH_SIZE=16*1024*1023;	//FLASH´óÐ¡Îª16M

	W25QXX_Write((u8*)TEXT_Buffer,FLASH_SIZE-100,SIZE);	

	W25QXX_Read(datatemp,FLASH_SIZE-100,SIZE);

	for(i=0;i<SIZE;i++){
		MSG("%c",datatemp[i]);
	}
#endif
	MSG("\r\n");
    tc_printf_tail("DBG w25qxx test ");
}





void w25qxx_tc_run(void)
{
    w25qxx_tc_01();
}



