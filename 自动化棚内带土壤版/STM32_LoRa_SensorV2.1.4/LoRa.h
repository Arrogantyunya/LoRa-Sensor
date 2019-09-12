
#ifndef _LoRa_H
#define _LoRa_H

#include <Arduino.h>
#include "Memory.h"
#include "User_CRC8.h"

#define LoRa_Serial  Serial1


/*LoRa module pin macro definition*/
#define LoRa_M0       PA4 
#define LoRa_M1       PA5 
#define LoRa_AUX_Wake PA0 
#define LoRa_PWR      PA8

/*LoRa operation macro definition*/
#define LORA_Sleep_Mode       (LoRa_SetMode(Sleep_Mode))              
#define LORA_Genel_Mode     (LoRa_SetMode(General_Mode))            
#define LORA_WakeUp_Mode      (LoRa_SetMode(WakeUp_Mode))             
#define LORA_Low_Power_Mode   (LoRa_SetMode(Low_Power_Mode))    
#define LORA_PWR_ON           (digitalWrite(LoRa_PWR, HIGH))          //Turn on LoRa power
#define LORA_PWR_OFF          (digitalWrite(LoRa_PWR, LOW))           //Turn off LoRa power      

/*---------LoRa parameter setting----------------------*/
enum LoRa_mode //LoRa operation mode
{ 
  General_Mode,   
  WakeUp_Mode,    
  Low_Power_Mode, 
  Sleep_Mode      
};

struct LoRa_Working_Parameter
{
unsigned int local_ID           = 0x0000; //Local ID
unsigned int gateway_ID         = 0xAA03; //gateway ID（Just fill in one）
unsigned char local_channel     = 0x18;   //Local channel
unsigned char gateway_channel   = 0x18;   //Gateway channel
unsigned char LoRa_Serial_Speed = 0x1A;   //Air transmission rate
unsigned char AwakeTime         = 0xC4;   //AwakeTime
unsigned char Receive_LoRa_Flag = 0;      //Determine whether data is received from the upper computer
unsigned char RTC_Awake_Flag    = 0;      //RTC Interrupt Awake flag
}LoRa_para;
/*-------------------------------------------------------*/

/*
 *brief   : Configurate LoRa GPIO mode
 *para    : None
 *return  : None
 */
void LoRa_GPIO_config(void)
{
  pinMode(LoRa_M0, OUTPUT);
  pinMode(LoRa_M1, OUTPUT);
  //pinMode(LoRa_AUX_Wake, INPUT_PULLDOWN);
  pinMode(LoRa_PWR, OUTPUT);
  delay(10);
}

/*
 *brief   : Set LoRa four mode
 *para    : General_Mode  WakeUp_Mode  Low_Power_Mode   Sleep_Mode
 *return  : None
 */
void LoRa_SetMode(LoRa_mode mode)
{
  switch(mode)
  {
    case   General_Mode:  digitalWrite(LoRa_M0, LOW);   digitalWrite(LoRa_M1, LOW);     break;
    case    WakeUp_Mode:  digitalWrite(LoRa_M0, HIGH);  digitalWrite(LoRa_M1, LOW);     break;
    case Low_Power_Mode:  digitalWrite(LoRa_M0, LOW);   digitalWrite(LoRa_M1, HIGH);    break;
    case     Sleep_Mode:  digitalWrite(LoRa_M0, HIGH);  digitalWrite(LoRa_M1, HIGH);    break;

    default:              digitalWrite(LoRa_M0, HIGH);  digitalWrite(LoRa_M1, HIGH);    break; //Default sleep mode
  }
}

/*
 *brief   : Set LoRa work parameter
 *para    : None
 *return  : true or false
 */
bool Set_LoRa_Para(void)
{
  unsigned char Send_LoRa_parameter[6] = {0xC0, highByte(LoRa_para.local_ID), lowByte(LoRa_para.local_ID), LoRa_para.LoRa_Serial_Speed, LoRa_para.local_channel, LoRa_para.AwakeTime};
  unsigned char Back_Work_Para[6] = {0};
  unsigned char EP_LoRa_para[6] = {0};
  unsigned char Inquire_Para_Cmd[3] = {0xC1, 0xC1, 0xC1};
  unsigned char config_num = 0;
  unsigned char i = 0;
  bool Set_Para_Flag = false;
  
  LORA_Sleep_Mode;
  delay(100);

  if (Verify_LORA_OPERATION_FLAG() == true && Verify_BKP_LORA_OPERATION_FLAG() == true) //If the LoRa parameter already exists in the EEPROM.
  {
    if(LoRa_Para_Self_Check(EP_LoRa_para))
    {
      //Read the LoRa pamameters of the device.
      Send_LoRa_parameter[1] = EP_LoRa_para[3];
      Send_LoRa_parameter[2] = EP_LoRa_para[4];
      Send_LoRa_parameter[4] = EP_LoRa_para[5];

      do{

        LoRa_Serial.write(Inquire_Para_Cmd, 3);
        delay(200);

        while (LoRa_Serial.available() > 0)
        {
          Back_Work_Para[i++] = LoRa_Serial.read();
          if (i >= 6)
            i = 0;
        }

        if (i == 6)
        {
          for (i = 0; i < 6; i++)
          {
            if (Back_Work_Para[i] != Send_LoRa_parameter[i])
            {
              Set_Para_Flag = true;
              break;
            }
          }

          if (Set_Para_Flag == false) //Already save parameter ,don't save again.
          {
            Serial.println("Already save parameter ,don't save again...");
            LORA_WakeUp_Mode;
            delay(100);
            return true;
          }
        }

        i = 0;
        config_num++;

      } while (config_num < 3 && Set_Para_Flag == false);
      config_num = 0;
    }
    else
    {
      Clear_LoRa_Flag();
      Clear_BKP_LoRa_Flag();
      LORA_PWR_OFF;
      delay(500);
      nvic_sys_reset();
    }
  }
  else
    Serial.println("No LoRa parameter in EEPROM...");

  for (i = 0; i < 6; i++)
  {
    Serial.print(Send_LoRa_parameter[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  i = 0;


  LoRa_Serial.write(Send_LoRa_parameter, 6); //configurate LoRa parameter
  delay(100);
  
  while (LoRa_Serial.available() > 0)
  {
    Back_Work_Para[i++] = LoRa_Serial.read();
    if (i >= 6)
      i = 0;
  }
  i = 0;

  if (Back_Work_Para[1] == Send_LoRa_parameter[1] && Back_Work_Para[2] == Send_LoRa_parameter[2] && Back_Work_Para[3] == Send_LoRa_parameter[3] && Back_Work_Para[4] == Send_LoRa_parameter[4] && Back_Work_Para[5] == Send_LoRa_parameter[5])
  {
    LORA_WakeUp_Mode;
    delay(100);
    return true;
  } 
  else
  {
    //If set failed, configure LoRa parameter 2 times again.
    do{

      LoRa_Serial.write(Send_LoRa_parameter, 6);//configurate LoRa parameter
      delay(100);

      while (LoRa_Serial.available() > 0)
      {
        Back_Work_Para[i++] = LoRa_Serial.read();
        if (i >= 6)
          i = 0;
      }
      i = 0;

      if (Back_Work_Para[1] == Send_LoRa_parameter[1] && Back_Work_Para[2] == Send_LoRa_parameter[2] && Back_Work_Para[3] == Send_LoRa_parameter[3] && Back_Work_Para[4] == Send_LoRa_parameter[4] && Back_Work_Para[5] == Send_LoRa_parameter[5])
      {
        LORA_WakeUp_Mode;
        delay(100);
        return true;
      }
      else
        config_num++;

    }while (config_num < 3);


    //If the configuration LoRa parameter fails, clear the LoRa configuration flag bit.
    Clear_LoRa_Flag();
    Clear_BKP_LoRa_Flag();
    LORA_WakeUp_Mode;
    delay(100);
    return false;
  }
}


#endif
