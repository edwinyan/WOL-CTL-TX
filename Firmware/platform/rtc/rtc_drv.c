#include "common.h"
#include "rtc_drv.h"

void rtc_drv_init(void)
{

}


u32 rtc_drv_get_datetime(void)
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    RTC_GetDate(RTC_Format_BIN, &date);
    RTC_GetTime(RTC_Format_BIN, &time);

    return ((u32)date.RTC_Year + 2000 - 1980)<<25 |
	((u32)date.RTC_Month)<<21 |
	((u32)date.RTC_Date)<<16 |
	((u32)time.RTC_Hours)<<11 |
	((u32)time.RTC_Minutes)<<5 |
	((u32)time.RTC_Seconds)>>1;
}

