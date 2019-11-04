#include "Private_RTC.h"
#include <Arduino.h>
#include "Memory.h"
#include "public.h"

/*RCT object*/
RTClock Date(RTCSEL_LSE); 
UTCTimeStruct RTCTime;

date Private_RTC;

bool LowBalFlag = false;

void date::Update_RTC(unsigned char *buffer)
{
  RTCTime.year = Type_Conv.Dec_To_Hex(buffer[0]) * 100 + Type_Conv.Dec_To_Hex(buffer[1]);
  RTCTime.month = Type_Conv.Dec_To_Hex(buffer[2]);
  RTCTime.day = Type_Conv.Dec_To_Hex(buffer[3]);
  RTCTime.hour = Type_Conv.Dec_To_Hex(buffer[4]);
  RTCTime.minutes = Type_Conv.Dec_To_Hex(buffer[5]);
  RTCTime.seconds = Type_Conv.Dec_To_Hex(buffer[6]);

  Serial.print("year: "); Serial.println(RTCTime.year);
  Serial.print("month: "); Serial.println(RTCTime.month);
  Serial.print("day: "); Serial.println(RTCTime.day);
  Serial.print("hour: "); Serial.println(RTCTime.hour);
  Serial.print("minutes: "); Serial.println(RTCTime.minutes);
  Serial.print("seconds: "); Serial.println(RTCTime.seconds);

  bkp_enable_writes();
  delay(100);
  UTCTime CurrentSec = osal_ConvertUTCSecs(&RTCTime);
  Date.setTime(CurrentSec);
  delay(100);
  bkp_disable_writes();

  CurrentSec = Date.getTime();
  osal_ConvertUTCTime(&RTCTime, CurrentSec);

  buffer[0] = Type_Conv.Hex_To_Dec(RTCTime.year / 100);
  buffer[1] = Type_Conv.Hex_To_Dec(RTCTime.year % 1000);
  buffer[2] = Type_Conv.Hex_To_Dec(RTCTime.month);
  buffer[3] = Type_Conv.Hex_To_Dec(RTCTime.day);
  buffer[4] = Type_Conv.Hex_To_Dec(RTCTime.hour);
  buffer[5] = Type_Conv.Hex_To_Dec(RTCTime.minutes);
  buffer[6] = Type_Conv.Hex_To_Dec(RTCTime.seconds);

  for (unsigned char i = 0; i < 7; i++){
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println("Update RTC SUCCESS... <Update_RTC>");
}

void date::Get_RTC(unsigned char *buffer)
{
  UTCTime CurrentSec = 0;
  CurrentSec = Date.getTime();
  osal_ConvertUTCTime(&RTCTime, CurrentSec);

  buffer[0] = Type_Conv.Hex_To_Dec(RTCTime.year / 100);
  buffer[1] = Type_Conv.Hex_To_Dec(RTCTime.year % 1000);
  buffer[2] = Type_Conv.Hex_To_Dec(RTCTime.month);
  buffer[3] = Type_Conv.Hex_To_Dec(RTCTime.day);
  buffer[4] = Type_Conv.Hex_To_Dec(RTCTime.hour);
  buffer[5] = Type_Conv.Hex_To_Dec(RTCTime.minutes);
  buffer[6] = Type_Conv.Hex_To_Dec(RTCTime.seconds);
}


/*
 *brief   : RTC alarm interrupt wake-up device
 *para    : None
 *return  : None
 */
void RTC_Interrupt(void)
{
  //Keep this as short as possible. Possibly avoid using function calls
  rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);
  nvic_sys_reset();
}

/*
 *brief   : Initalize setting alarm clock
 *para    : None
 *return  : None
 */
void date::Init_Set_Alarm(void)
{
  bkp_enable_writes();
  delay(100);
  time_t Alarm_Time = 0;
  Alarm_Time = Date.getTime();
  Alarm_Time += 180;
  Date.createAlarm(RTC_Interrupt, Alarm_Time);
  delay(100);
  bkp_disable_writes();
}

/*
 *brief   : Read alarm clock number from EEPROM and set alarm clock
 *para    : None
 *return  : None
 */
void date::Set_Alarm(void)
{
  UTCTime CurrentSec = osal_ConvertUTCSecs(&RTCTime);

  bkp_enable_writes();
  delay(10);
  Date.setTime(CurrentSec);

  unsigned long int alarm = Date.getTime(); //Get current time.

  if (WorkParameter_Info.Verify_Collect_Time_Flag())
  {
    unsigned int Time_temp = WorkParameter_Info.Read_Collect_Time();

    if (Time_temp > 43200 || Time_temp < 15) //max: 12 hours; min: 15seconds.
      Time_temp = 180;

    if (LowBalFlag) //如果电压过低，强制休眠20分钟
    {
      if (Time_temp > 1200) //如果设置的休眠时间大于20分钟，那么采用设置的时间
        alarm += Time_temp;
      else
        alarm += 1200;
    }
    else
      alarm += Time_temp;
  }
  else
  {
    if (LowBalFlag) 
      alarm += 1200;  //如果电压过低，强制休眠20分钟
    else
    alarm += 180;    
  }

  Date.createAlarm(RTC_Interrupt, alarm);
  delay(10);
  bkp_disable_writes();
}
