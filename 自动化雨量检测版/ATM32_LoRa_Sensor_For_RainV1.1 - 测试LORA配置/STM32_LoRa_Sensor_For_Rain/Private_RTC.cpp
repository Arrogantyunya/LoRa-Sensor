#include "Private_RTC.h"
#include <Arduino.h>
#include "Memory.h"
#include "public.h"
#include "private_sensor.h"

/*RCT object*/
RTClock Date(RTCSEL_LSE); 
UTCTimeStruct RTCTime;

date Private_RTC;

void date::Update_RTC(unsigned char *buffer)
{
  RTCTime.year = Type_Conv.Dec_To_Hex(buffer[0]) * 100 + Type_Conv.Dec_To_Hex(buffer[1]);
  RTCTime.month = Type_Conv.Dec_To_Hex(buffer[2]);
  RTCTime.day = Type_Conv.Dec_To_Hex(buffer[3]);
  RTCTime.hour = Type_Conv.Dec_To_Hex(buffer[4]);
  RTCTime.minutes = Type_Conv.Dec_To_Hex(buffer[5]);
  RTCTime.seconds = Type_Conv.Dec_To_Hex(buffer[6]);

  Serial1.print("year: "); Serial1.println(RTCTime.year);
  Serial1.print("month: "); Serial1.println(RTCTime.month);
  Serial1.print("day: "); Serial1.println(RTCTime.day);
  Serial1.print("hour: "); Serial1.println(RTCTime.hour);
  Serial1.print("minutes: "); Serial1.println(RTCTime.minutes);
  Serial1.print("seconds: "); Serial1.println(RTCTime.seconds);

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
    Serial1.print(buffer[i], HEX);
    Serial1.print(" ");
  }
  Serial1.println();

  Serial1.println("Update RTC SUCCESS... <Update_RTC>");
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
  g_Send_Air_SensorData_Flag = true;
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

  if (WorkParameter_Info.Verify_Collect_Time_Flag()){
    unsigned int Time_temp = WorkParameter_Info.Read_Collect_Time();

    if (Time_temp > 43200 || Time_temp < 30) //max: 12 hours; min: 30seconds.
      Time_temp = 180;

    alarm += Time_temp;
  }
  else
    alarm += 180;

  Date.createAlarm(RTC_Interrupt, alarm);
  delay(10);
  bkp_disable_writes();
}
