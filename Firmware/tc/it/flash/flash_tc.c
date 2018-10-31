#include "common.h"
#include "flash_tc.h"
#include "bsp.h"
#include "bsp_os.h"
#include "tc_common.h"
#include "flash_drv.h"

const u8 Test_Buffer[]={"STM32 FLASH TEST"};
#define TEXT_LENTH sizeof(Test_Buffer)
#define SIZE TEXT_LENTH

STATIC void flash_tc_01(void)
{
    u8 datatemp[SIZE];
	u8 i;

    tc_printf_banner("flash test ");
    
    FLASH_Write(FLASH_TEST_ADDR,(u32*)Test_Buffer,SIZE);

	FLASH_Read(FLASH_TEST_ADDR,(u32*)datatemp,SIZE);

	for(i=0;i<SIZE;i++){
		MSG("%c",datatemp[i]);
	}
	MSG("\r\n");
    tc_printf_tail("flash test ");
}


void flash_tc_run(void)
{
    flash_tc_01();
}

