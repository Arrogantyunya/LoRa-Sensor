                                 
#include <Arduino.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include <RTClock.h>
#include "RS485.h"
#include "LoRa.h"
#include "Memory.h"
#include "Command_Analysis.h"
#include "fun_periph.h"
#include "Private_RTC.h"
#include "private_sensor.h"
#include "receipt.h"

unsigned char g_SN_Code[9] = {0x00}; //Defualt SN code is 0.

/*----------Function statement----------*/
void Sleep(void);
/*--------------------------------------*/

/****************Set Up*******************/
void setup() 
{
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  Serial.begin(9600);         //DEBUG Serial baud
  LoRa_MHL9LF.BaudRate(9600);    //LoRa SoftwareSerial baud
  RS485_Serial.begin(9600);  //ModBus SoftwareSerial baud
  bkp_init(); //Initialize backup register.
  delay(10);

  Some_Peripheral.Peripheral_GPIO_Config();   
  LoRa_MHL9LF.LoRa_GPIO_Config();   
  RS485_GPIO_Config();  //RS485 GPIO configuration
  EEPROM_Operation.EEPROM_GPIO_Config();
  LoRa_MHL9LF.Mode(AT);
  delay(1000);
  LoRa_Command_Analysis.Receive_LoRa_Cmd();
  LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
  delay(10);

  USB_ON; //Turn on the USB enable.

  delay(5000);

  //LoRa_MHL9LF.Parameter_Init();

#if SOFT_HARD_VERSION
  Vertion.Save_Software_version(0x00, 0x01);
  Vertion.Save_hardware_version(0x00, 0x01);
#endif

  SN.Clear_SN_Access_Network_Flag();

  Request_Access_Network();

  while (SN.Self_Check(g_SN_Code) == false){
    LED_SELF_CHECK_ERROR;
    Serial.println("Verify SN code ERROR, try to Retrieving SN code...");
    Message_Receipt.Request_Device_SN_and_Channel();
    LoRa_Command_Analysis.Receive_LoRa_Cmd();
    delay(3000);
  }
  Serial.println("SN self_check success...");
  LED_RUNNING;

  Data_Communication_with_Gateway();

  Private_RTC.Set_Alarm();
}

void loop() 
{
  Sleep();
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

    if (SN.Save_SN_Code(g_SN_Code) && SN.Save_BKP_SN_Code(g_SN_Code))
      Serial.println("Write SN success...");

    if(Control_Info.Clear_Area_Number() && Control_Info.Clear_Group_Number())
      Serial.println("Already Clear area number and grouop number...");

    unsigned char Default_WorkGroup[5] = {0x01, 0x00, 0x00, 0x00, 0x00};
    if(Control_Info.Save_Group_Number(Default_WorkGroup))
      Serial.println("Save gourp number success...");

   LED_NO_REGISTER;
  }
  while (SN.Verify_SN_Access_Network_Flag() == false){

    if (digitalRead(K1) == LOW){
      delay(5000);
      if (digitalRead(K1) == LOW){
        Message_Receipt.Report_General_Parameter();

        while (digitalRead(K1) == LOW);
      }
    }
    LoRa_Command_Analysis.Receive_LoRa_Cmd();
  } 
  g_Access_Network_Flag = true;
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

  Message_Receipt.Send_Sensor_Data();

  do {
      while ((millis() < (wait_time + 10000)) && g_Get_Para_Flag == false)  //waiting to receive acquisition parameter.
        LoRa_Command_Analysis.Receive_LoRa_Cmd(); //waiting to receive acquisition parameter.
      
      if (g_Get_Para_Flag == false){ //If it don't receive a message from the gateway.
        GREEN_ON;
        Message_Receipt.Send_Sensor_Data(); //Send sensor data to gateway again.
        GREEN_OFF;
        Get_Para_num++;
        Serial.println(Get_Para_num);
      }
      else
        break; //If receive acquisition parameter, break loop.

      wait_time = millis();

  }while (Get_Para_num < 3);

  if (Get_Para_num == 3) //If it don't receive message three times.
    Serial.println("Not get para cmd !!!");
}

/*
 *brief   : This device goes sleep
 *para    : None
 *library : <Enerlib.h> <Arduino.h>
 */
void Sleep(void)
{
  Serial.println("Enter Sleep>>>>>>");
  PWR_485_OFF;
  RED_OFF;
  GREEN_OFF;
  USB_OFF;
  LORA_PWR_OFF;

  PWR_WakeUpPinCmd(ENABLE);//使能唤醒引脚，默认PA0
  PWR_ClearFlag(PWR_FLAG_WU);
  PWR_EnterSTANDBYMode();//进入待机
}


