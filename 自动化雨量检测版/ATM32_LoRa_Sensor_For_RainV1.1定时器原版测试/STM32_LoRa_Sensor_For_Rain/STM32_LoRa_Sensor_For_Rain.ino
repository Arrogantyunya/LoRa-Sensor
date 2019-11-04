                                 
#include <Arduino.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include <libmaple/iwdg.h>
#include <RTClock.h>
#include "RS485.h"
#include "LoRa.h"
#include "Memory.h"
#include "Command_Analysis.h"
#include "fun_periph.h"
//#include "Private_RTC.h"
#include "private_sensor.h"
#include "receipt.h"

void Request_Access_Network(void);
void Data_Communication_with_Gateway(bool Is_Priority_Rain_Flag);
void Sleep(void);
void Key_Reset_LoRa_Parameter(void);
void Timer2_Interrupt(void);

unsigned char g_SN_Code[9] = {0x00}; //Defualt SN code is 0.

unsigned long Cyc_Send_Data_Num = 0;

/*----------Function statement----------*/
void Sleep(void);
/*--------------------------------------*/

/****************Set Up*******************/
void setup() 
{
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  iwdg_init(IWDG_PRE_256, 1000);  //6.5ms * 1000 = 6500ms.

  Serial.begin(9600);         //DEBUG Serial baud
  LoRa_MHL9LF.BaudRate(9600);    //LoRa SoftwareSerial baud
  RS485_Serial.begin(9600);  //ModBus SoftwareSerial baud
  bkp_init(); //Initialize backup register.

  USB_ON; //Turn on the USB enable
  delay(10);

  iwdg_feed();

  Some_Peripheral.Peripheral_GPIO_Config();   
  LoRa_MHL9LF.LoRa_GPIO_Config();   
  RS485_GPIO_Config();  //RS485 GPIO configuration
  EEPROM_Operation.EEPROM_GPIO_Config();
  LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
  delay(1000);
  LoRa_Command_Analysis.Receive_LoRa_Cmd();

  iwdg_feed();

#if SOFT_HARD_VERSION
  Vertion.Save_Software_version(0x00, 0x01);
  Vertion.Save_hardware_version(0x00, 0x01);
#endif

  Key_Reset_LoRa_Parameter();

  //LoRa_Para_Config.Clear_LoRa_Config_Flag();
  //Initialize LoRa parameter.
  if (LoRa_Para_Config.Verify_LoRa_Config_Flag() == false){
    LoRa_MHL9LF.Rewrite_GroupID();
    LoRa_MHL9LF.Rewrite_ID();
    LoRa_MHL9LF.Parameter_Init();
    LoRa_MHL9LF.IsReset(true);
    LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
    LoRa_Command_Analysis.Receive_LoRa_Cmd();
    LoRa_Para_Config.Save_LoRa_Config_Flag();
  }

  iwdg_feed();

  SN.Clear_SN_Access_Network_Flag();
  Request_Access_Network();

  while (SN.Self_Check(g_SN_Code) == false){
    iwdg_feed();
    LED_SELF_CHECK_ERROR;
    Serial.println("Verify SN code ERROR, try to Retrieving SN code...");
    Message_Receipt.Request_Device_SN_and_Channel();
    delay(1000);
    iwdg_feed();
    LoRa_Command_Analysis.Receive_LoRa_Cmd();
    iwdg_feed();
    delay(3000);
    iwdg_feed();
  }
  Serial.println("SN self_check success...");
  LED_RUNNING;

  Timer_Init();
  g_Send_Air_SensorData_Flag = true;
}

void loop() 
{
  iwdg_feed();

  private_sensor.Detect_Rain();
  
  if (g_Current_Rain_Status_Flag != g_Last_Rain_Status_Flag){
    Serial.println("Send real-time data of sensor...");
    Data_Communication_with_Gateway();
  }

  if (g_Send_Air_SensorData_Flag == true){
    g_Send_Air_SensorData_Flag = false;
    Serial.println("Send cyclic data of sensor...");
    Data_Communication_with_Gateway();
    Timer2.setCount(0);//设置当前计时器计数
    Timer2.resume();//在不影响其配置的情况下恢复暂停的计时器
  }
}

/*
 @brief   : 检测是否已经注册到服务器成功，如果没有注册，则配置相关参数为默认参数，然后注册到服务器。
            没有注册成功，红灯1每隔500ms闪烁。
            Checks whether registration with the server was successful, and if not, 
            configures the relevant parameters as default parameters and registers with the server.
            Failing registration, red light flashes every 500ms.
 @para    : None
 @return  : None
 */
void Request_Access_Network(void)
{
  if (SN.Verify_SN_Access_Network_Flag() == false){
    g_Access_Network_Flag = false;
    iwdg_feed();
    if (SN.Save_SN_Code(g_SN_Code) && SN.Save_BKP_SN_Code(g_SN_Code))
      Serial.println("Init SN success...");

    if(Control_Info.Clear_Area_Number() && Control_Info.Clear_Group_Number())
      Serial.println("Already Clear area number and grouop number...");

    unsigned char Default_WorkGroup[5] = {0x01, 0x00, 0x00, 0x00, 0x00};
    if(Control_Info.Save_Group_Number(Default_WorkGroup))
      Serial.println("Save gourp number success...");

   LED_NO_REGISTER;
  }
  while (SN.Verify_SN_Access_Network_Flag() == false){
    iwdg_feed();
    if (digitalRead(K1) == LOW){
      iwdg_feed();
      delay(5000);
      iwdg_feed();
      if (digitalRead(K1) == LOW){
        Message_Receipt.Report_General_Parameter();

        while (digitalRead(K1) == LOW)
          iwdg_feed();
      }
    }
    LoRa_Command_Analysis.Receive_LoRa_Cmd();
    iwdg_feed();
  } 
}

/*
 *brief   : Send sensor data to gateway and then received some parameters from gateway.
 *para    : None
 *return  : None
 */
void Data_Communication_with_Gateway(void)
{
  unsigned char Get_Para_num = 0;
  unsigned long wait_time = millis();

  //If it don't receive a message from the gateway.
  do {
      iwdg_feed();
      if (g_Get_Para_Flag == false){ 
        Message_Receipt.Send_Sensor_Data(); //Send sensor data to gateway again.
        Get_Para_num++;
      }else{
        g_Last_Rain_Status_Flag = g_Current_Rain_Status_Flag;
        g_Get_Para_Flag = false;
        break; //If receive acquisition parameter, break loop.
      }
      while ((millis() < (wait_time + 10000)) && g_Get_Para_Flag == false)  //waiting to receive acquisition parameter.
        LoRa_Command_Analysis.Receive_LoRa_Cmd(); //waiting to receive acquisition parameter.
    
      wait_time = millis();

  }while (Get_Para_num < 3);

  if (Get_Para_num == 3) //If it don't receive message three times.
    Serial.println("No parameter were received !!!");
}

/*
 *brief   : This device goes sleep
 *para    : None
 *library : <Enerlib.h> <Arduino.h>
 */
void Sleep(void)
{
  Serial.println("Enter Sleep>>>>>>");
  Some_Peripheral.Stop_LED();
  PWR_485_OFF;
  LORA_PWR_OFF;
  USB_OFF;

  PWR_WakeUpPinCmd(ENABLE);//使能唤醒引脚，默认PA0
  PWR_ClearFlag(PWR_FLAG_WU);
  PWR_EnterSTANDBYMode();//进入待机
}

void Key_Reset_LoRa_Parameter(void)
{
  if (digitalRead(K1) == LOW){
    delay(100);
    if (digitalRead(K1) == LOW){
      //Some_Peripheral.Key_Buzz(600);
      iwdg_feed();
      delay(5000);
      iwdg_feed();
      if (digitalRead(K2) == LOW){
        delay(100);
        if (digitalRead(K2) == LOW){
          //Some_Peripheral.Key_Buzz(600);
          LoRa_Para_Config.Clear_LoRa_Config_Flag();
          Serial.println("Clear LoRa configuration flag SUCCESS... <Key_Reset_LoRa_Parameter>");
          iwdg_feed();
        }
      }
    }
  }
}

void Timer_Init(void)
{
  Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
  Timer2.setPeriod(1000000L); // in microseconds，1S
  Timer2.setCompare1(1);   // overflow might be small
  Timer2.attachCompare1Interrupt(Timer2_Interrupt);   
  Timer2.setCount(0);
  //Timer2.pause(); 
}

void Timer2_Interrupt(void)
{
  Cyc_Send_Data_Num++;
  if (Cyc_Send_Data_Num >= WorkParameter_Info.Read_Collect_Time()){
    Timer2.pause();
    Cyc_Send_Data_Num = 0;
    g_Send_Air_SensorData_Flag = true;
  }
}