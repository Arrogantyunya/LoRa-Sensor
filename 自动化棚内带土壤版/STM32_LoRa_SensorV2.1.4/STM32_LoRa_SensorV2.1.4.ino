                                 
#include <Arduino.h>
#include "MODBUS_RTU_CRC16.h" 
#include "BCD_CON.h"
#include "User_CRC8.h"
#include <libmaple/nvic.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include <SoftWire.h>
#include <RTClock.h>
#include "User_Clock.h"
#include "RS485.h"
#include "LoRa.h"
#include "SHT1x.h"
#include "i2c.h"
#include "i2c_MAX44009.h"

//#define SN_SETTING
#define LORA_SETTING
//#define DEBUG
//#define TEST_DEBUG

#define   PR_3000_ECTH_N01    1

#define LUX_UV

#define USBEN                 PA15

#define RED                   PB3
#define GREEN                 PB4

SHT1x sht10(PB15, PB14); //SDA, SCL

#define BAT_V_PIN             PA6
#define ADC_RATE              0.8057
#define VBAT_DIVIDER_RATIO    6

#define RS485_Serial          Serial3

#define USB_ON                (digitalWrite(USBEN, LOW))
#define USB_OFF               (digitalWrite(USBEN, HIGH))

#define RED_ON                (digitalWrite(RED, HIGH))
#define RED_OFF               (digitalWrite(RED, LOW))
#define GREEN_ON              (digitalWrite(GREEN, HIGH))
#define GREEN_OFF             (digitalWrite(GREEN, LOW))

RTClock InRtc (RTCSEL_LSE); 
UTCTimeStruct RtcTime;

SoftwareI2C CJMCU6750;
MAX44009 max44009;

#define I2C_ADDR 0x38 //UV sensor

//Integration Time
#define IT_1_2 0x0 //1/2T
#define IT_1   0x1 //1T
#define IT_2   0x2 //2T
#define IT_4   0x3 //4T

#define GATEWAY_ID_H          0xAA    
#define GATEWAY_ID_L          0x03
#define GATEWAY_CH            0x18

#define DEVICE_TYPE_H         0x00
#define DEVICE_TYPE_L         0x01

enum Receipt_Status{
  FactoryMode = 0, AskMasterRejoinOk, AskMasterRejoinErr, AssignIdOk, AssignIdErr, SetParamsOk, SetParamsErr
};

struct SENSOR_DATA{
  float             g_Temp;
  unsigned char     g_Temp_Flag;
  float             g_Humi;
  unsigned long int g_Lux;
  unsigned int      g_Solid_Temp;
  unsigned char     g_Solid_Temp_Flag;
  float             g_Solid_Humi;
  unsigned int      g_Salt;
  unsigned int      g_Cond;
  unsigned int      g_UV;
}Sensor_Data;

unsigned char g_SN_Code[9] = {0x00, 0x01, 0x20, 0x18, 0x11, 0x26, 0x15, 0x00, 0x07};  //DEBUG
unsigned char g_SN_Data[9] = {0};

unsigned char g_LoRa_Para[6] = {0};

unsigned char g_Receive_cmd[128] = {0};
unsigned char g_len = 0;
unsigned char g_Receive_Lora_Para_Num = 0;
unsigned char g_End_num = 0;
unsigned char g_Get_Para_num = 0;
bool          g_Receive_end_flag = false;
bool          g_End_num_flag = false;
bool          g_Get_Para_Flag = false;

/*----------Function statement----------*/
void GPIO_Config(void);
unsigned int Get_Voltage(void);
void RED_Display(unsigned int d, unsigned int cycle);
void Wait_LoRa_papa_Configuration(void);
void Receive_LoRa_Cmd(void);
void Sleep(void);
void LoRa_Receive_Data_Analysis(void);
void Read_Solid_Humi_and_Temp(float *humi, unsigned int *temp, unsigned char *temp_flag, unsigned char addr);
void Read_Salt_and_Cond(unsigned int *salt, unsigned int *cond, unsigned char addr);
void LoRa_Message_Receipt(unsigned char status, unsigned char times);
void LoRa_Send_Sensor_Data(void);
void Request_Get_LoRa_parameter(void);
unsigned char BCD_Chg_Dat(unsigned char dat);
unsigned char Dat_Chg_BCD(unsigned char dat);
void LoRa_AUX_Handler();
void RTC_Interrupt();
/*--------------------------------------*/


/****************Set Up*******************/
void setup() 
{
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  Serial.begin(9600);         //DEBUG Serial baud
  LoRa_Serial.begin(9600);    //LoRa SoftwareSerial baud
  RS485_Serial.begin(9600);  //ModBus SoftwareSerial baud
  delay(10);

  GPIO_Config();        
  LoRa_GPIO_config();   //LoRa GPIO configuration
  RS485_GPIO_Config();  //RS485 GPIO configuration
  EEPROM_WP_Init();     //Write enable pin initialization
  I2C_INIT();           //I2C pin initialization
  delay(10);

  USB_ON; //Turn on the USB enable.
  PWR_485_ON;
  LORA_PWR_ON; //Turn on LoRa module power.
  delay(300);  

#ifdef DEBUG
  delay(5000);
#else
  delay(1000); //Waiting for LoRa power supply to stabilize.
#endif

  bkp_init(); //Initialize backup register.
  delay(10);

  RED_OFF;  //Turn off the red light.
  GREEN_OFF;  //Turn off the green light.

  //Test write SN
#ifdef TEST_DEBUG
  Clear_SN_Flag();
  Clear_BKP_SN_Flag();

  if (Write_SN(g_SN_Code) && Write_BKP_SN(g_SN_Code))
    Serial.println("Set SN success...");
  else
  {
    while (1)
    {
      Serial.println("Set SN failed, check up EEPROM !!!");
      delay(3000);
    }
  }
#endif

#ifdef SN_SETTING
  Wait_Reiceive_SN_Code();
#endif

  if(SN_Self_Check(g_SN_Data) == true)
    Serial.println("SN self_check success...");
  else
    RED_Display(500, 0);

#ifdef TEST_DEBUG
  Clear_LoRa_Flag();
  Clear_BKP_LoRa_Flag();
#endif

  //Initialization configuration generic LoRa parameters
  if (Set_LoRa_Para())
    Serial.println("Initialization configuration LoRa parameter success...");
  else
  {
    Serial.println("Initialization configuration LoRa parameter error!");
    LORA_PWR_OFF;
    delay(500);
    nvic_sys_reset();
  }

#ifdef LORA_SETTING
  Application_for_LoRa_parameters();
#endif

  GREEN_ON;
  LoRa_Send_Sensor_Data(); //Sned sensor data to gateway.
  GREEN_OFF;

  RED_ON;
  Data_Communication_with_Gateway();
  RED_OFF;

  Set_Alarm();
}

void loop() 
{
  Sleep();
}

/*
 *brief   : Configurate some GPIO mode 
 *para    : None
 *return  : None
 */
void GPIO_Config(void)
{
  pinMode(USBEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BAT_V_PIN, INPUT_ANALOG);
  delay(10);
}

/*
 *brief   : Initalize setting alarm clock
 *para    : None
 *return  : None
 */
void Init_Set_Alarm(void)
{
  bkp_enable_writes();
  delay(100);
  time_t Alarm_Time = 0;
  Alarm_Time = InRtc.getTime();
  Alarm_Time += 180;
  InRtc.createAlarm(RTC_Interrupt, Alarm_Time);
  delay(100);
  bkp_disable_writes();
}

/*
 *brief   : Read alarm clock number from EEPROM and set alarm clock
 *para    : None
 *return  : None
 */
void Set_Alarm(void)
{
  UTCTime CurrentSec = osal_ConvertUTCSecs(&RtcTime);

  bkp_enable_writes();
  delay(10);
  InRtc.setTime(CurrentSec);

  unsigned long int alarm = InRtc.getTime(); //Get current time.

  if (Verify_COLLECT_TIME_FLAG())
  {
    unsigned int Time_temp = Read_Collect_Time();

    if (Time_temp > 1800 || Time_temp <= 0) //half hours
      Time_temp = 180;

    alarm += Time_temp;
  }
  else
    alarm += 180;

  InRtc.createAlarm(RTC_Interrupt, alarm);
  delay(10);
  bkp_disable_writes();
}

/*
 *brief   : Waiting to receive SN code
 *para    : None
 *return  : None
 */
void Wait_Reiceive_SN_Code(void)
{
  unsigned char SN_Buffer[9] = {0};
  unsigned char Length = 0;

  while (Verify_SN_OPERATION_FLAG() == false || Verify_BKP_SN_OPERATION_FLAG() == false) //If you haven't written SN.
  {
    while (Serial.available() > 0)
    {
      if (Length >= 9)
        Length = 0;

      SN_Buffer[Length++] = Serial.read();
      delay(10);
    }

    if (Length == 9)
    {
      Length = 0;
      if(Write_SN(SN_Buffer) == true && Write_BKP_SN(SN_Buffer) == true) // If the write SN fails
        ;
      else
      {
        Clear_SN_Flag();
        Clear_BKP_SN_Flag();
        RED_Display(200, 10); //Red light flashes for several seconds,indicating that SN writes failed and needs to be rewrite.
      }
    }
    else
      Length = 0;
  }
}

/*
 *brief   : Apply for LoRa parameter with the gateway
 *para    : None
 *return  : None
 */
void Application_for_LoRa_parameters(void)
{
  if(Verify_LORA_OPERATION_FLAG() == false || Verify_BKP_LORA_OPERATION_FLAG() == false) 
  {
    Serial.println("# Request get LoRa parameter...");
    Request_Get_LoRa_parameter();
    Serial.println("# Waiting receive LoRa parameter...");
    Wait_LoRa_papa_Configuration();
  }
}

/*
 *brief   : Send sensor data to gateway and then received some parameters from gateway.
 *para    : None
 *return  : None
 */
void Data_Communication_with_Gateway(void)
{
  g_Get_Para_Flag = false; //waiting to receive acquisition parameter.
  g_Receive_end_flag = false;
  unsigned long wait_time = millis();

  do {

      while ((millis() < (wait_time + 10000)) && g_Get_Para_Flag == false)  //waiting to receive acquisition parameter.
        Receive_LoRa_Cmd(); //waiting to receive acquisition parameter.
      
      if (g_Get_Para_Flag == false) //If it don't receive a message from the gateway.
      {
        GREEN_ON;
        LoRa_Send_Sensor_Data(); //Send sensor data to gateway again.
        GREEN_OFF;
        g_Get_Para_num++;
      }
      else
        break; //If receive acquisition parameter, break loop.

      wait_time = millis();

  }while (g_Get_Para_num < 3);

  if (g_Get_Para_num == 3) //If it don't receive message three times.
    Serial.println("Not get para cmd !!!");
}

/*
 *brief   : Collect battery Voltage
 *para    : None
 *return  : None
 */
unsigned int Get_Voltage(void)
{
  unsigned int BAT_V_Buffer[9] = {0};
  unsigned int Bat_Temp;

  for (unsigned char i = 0; i < 9; i++)
  {
    BAT_V_Buffer[i] = analogRead(BAT_V_PIN) * ADC_RATE * VBAT_DIVIDER_RATIO;
    delay(5);
  }
  
  for (unsigned char i = 0; i < 9; i++)
  {
    for (unsigned char j = 0; j < 8; j++)
    {
      if (BAT_V_Buffer[j] > BAT_V_Buffer[j + 1])
      {
        Bat_Temp = BAT_V_Buffer[j + 1];
        BAT_V_Buffer[j + 1] = BAT_V_Buffer[j];
        BAT_V_Buffer[j] = Bat_Temp;
      }
    }
  }
  return (BAT_V_Buffer[4]);
}

/*
 *brief   : Controlling the frequency and times of LED
 *para    : frequency(ms), times(times = 0, halt)
 *return  : None
 */
void RED_Display(unsigned int d, unsigned int cycle)
{
  if (cycle != 0)
  {
    for (unsigned char i = 0; i < cycle; i++)
    {
      RED_ON;
      delay(d);
      RED_OFF;
      delay(d);
    }
  }
  else
  {
    while (1)
    {
      RED_ON;
      delay(d);
      RED_OFF;
      delay(d);
    }
  }
}

/*
 *brief   : Waitting receive LoRa parameter form LoRa gateway
 *para    : None
 *return  : None
 */
void Wait_LoRa_papa_Configuration(void)
{
  while (Verify_LORA_OPERATION_FLAG() == false || Verify_BKP_LORA_OPERATION_FLAG() == false)
  {
    if (g_Receive_Lora_Para_Num > 100)
    {
      Serial.println("Request get LoRa parameter...");
      g_Receive_Lora_Para_Num = 0;
      Request_Get_LoRa_parameter();
    }
    
    g_Receive_Lora_Para_Num++;
    delay(100);

    while (LoRa_Serial.available() > 0 && g_Receive_end_flag == false)
    {
      g_Receive_cmd[g_len++] = LoRa_Serial.read();
      if (g_len >= 128)
        g_len = 0;

      Serial.print(g_Receive_cmd[g_len - 1], HEX);
      Serial.print(" ");
      delay(5);

      //Verify frame end 0D 0A 0D 0A 0D 0A
      if (g_End_num_flag == false)
      {
        if (g_Receive_cmd[g_len - 1] == 0x0D)
          g_End_num_flag = true;
      }

      if (g_End_num_flag == true) //Get first frame end.
      {
        switch (g_End_num)
        {
          case 0 : g_Receive_cmd[g_len - 1] == 0x0A ? g_End_num += 1 : g_End_num = 0; break;
          case 1 : g_Receive_cmd[g_len - 1] == 0x0D ? g_End_num += 1 : g_End_num = 0; break;
          case 2 : g_Receive_cmd[g_len - 1] == 0x0A ? g_End_num += 1 : g_End_num = 0; break;
          case 3 : g_Receive_cmd[g_len - 1] == 0x0D ? g_End_num += 1 : g_End_num = 0; break;
          case 4 : g_Receive_cmd[g_len - 1] == 0x0A ? g_End_num += 1 : g_End_num = 0; break;
        }
      }

      if (g_End_num == 5)
      {
        g_End_num = 0;
        g_Receive_end_flag = true;
        g_End_num_flag = false;
      }
    }

    if (g_Receive_end_flag == true)
    {
      g_Receive_end_flag = false;

      unsigned char SN_Temp[9];
      for (unsigned char i = 0; i < 9; i++)
        SN_Temp[i] = g_Receive_cmd[i + 9];

      if (Verify_SN(SN_Temp) == true && Verify_BKP_SN(SN_Temp) == true)
      {
        Serial.println();
        Serial.println("# Parsing execution command...");
        LoRa_Receive_Data_Analysis();
        g_len = 0;
      }
      else                  
      {
        Serial.println("Verify_SN failed !!!");
        g_Receive_end_flag = false;
        g_len = 0;
      }
    }
    else
    {
      g_End_num_flag = false;
      g_Receive_end_flag = false;
      g_len = 0;
    }
  }
}

/*
 *brief   : Receive LoRa commomd from gateway
 *para    : None
 *return  : None
 */
void Receive_LoRa_Cmd(void)
{
  while (LoRa_Serial.available() > 0 && g_Receive_end_flag == false)
  {
    g_Receive_cmd[g_len++] = LoRa_Serial.read();
    if (g_len >= 128)
      g_len = 0;

    Serial.print(g_Receive_cmd[g_len - 1], HEX);
    Serial.print(" ");
    delay(5);
    
    //验证帧尾 0D 0A 0D 0A 0D 0A
    if (g_End_num_flag == false)
    {
      if (g_Receive_cmd[g_len - 1] == 0x0D)
        g_End_num_flag = true;
    }

    if (g_End_num_flag == true) //Get first frame end.
    {
      switch (g_End_num)
      {
        case 0 : g_Receive_cmd[g_len - 1] == 0x0A ? g_End_num += 1 : g_End_num = 0; break;
        case 1 : g_Receive_cmd[g_len - 1] == 0x0D ? g_End_num += 1 : g_End_num = 0; break;
        case 2 : g_Receive_cmd[g_len - 1] == 0x0A ? g_End_num += 1 : g_End_num = 0; break;
        case 3 : g_Receive_cmd[g_len - 1] == 0x0D ? g_End_num += 1 : g_End_num = 0; break;
        case 4 : g_Receive_cmd[g_len - 1] == 0x0A ? g_End_num += 1 : g_End_num = 0; break;
      }
    }

    if (g_End_num == 5)
    {
      g_End_num = 0;
      g_End_num_flag = false;
      g_Receive_end_flag = true;
    }
  }

  if (g_Receive_end_flag == true)
  {
    Serial.println("# Parsing execution command...");
    g_Receive_end_flag = false;
    LoRa_Receive_Data_Analysis();
    g_len = 0;
  } 
  else
  {
    g_Receive_end_flag = false;
    g_End_num_flag = false;
    g_len = 0;
  }
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

/*
 *brief   : Verify that the received device type is consistant with local one
 *para    : device type high byte, device type low byte 
 *return  : true or false
 */
bool Verify_Device_Type(unsigned char type_h, unsigned char type_l)
{
  if (type_h == DEVICE_TYPE_H && type_l == DEVICE_TYPE_L)
    return true;
  else
    return false;
}

/*
 *brief   : Clear saved LoRa fig and turn off LoRa module power and restart.
 *para    : None
 *return  : None
 */
void Set_LoRa_Restart(void)
{
  Clear_LoRa_Flag();
  Clear_BKP_LoRa_Flag();
  LORA_PWR_OFF;
  delay(500);
  nvic_sys_reset();  
}

/*
 *brief   : Receive LoRa commond form upper computer,such as LoRa paramter, collection cycle and so on
 *para    : None
 *return  : None
 */
void LoRa_Receive_Data_Analysis(void)
{ 
  unsigned char Receive_crc8 = 0;

  //Request to re-request the LoRa parameter commomd
  if (g_Receive_cmd[0] == 0xFE && g_Receive_cmd[1] == 0xA0 && g_Receive_cmd[2] == 0x11)
  {
    Receive_crc8 = GetCrc8(&g_Receive_cmd[4], 24);

    if (Receive_crc8 == g_Receive_cmd[g_len - 7])
    {
      if (Verify_Device_Type(g_Receive_cmd[7], g_Receive_cmd[8]) == true)
      {
        if (Verify_Receive_SN_Code(&g_Receive_cmd[9]) == true)
        {
          if (g_Receive_cmd[18] == 0x55)
          {
            LoRa_Message_Receipt(AskMasterRejoinOk, 2);
            Set_LoRa_Restart();
          }
        }
        else
          Serial.println("<SN code error> section: (Reset inquire assign LoRa parameter)");
      }
      else
        Serial.println("<Device type error> section: (Reset inquire assign LoRa parameter)");
    }
    else
      Serial.println("<Check Code Error> section: (Reset inquire assign LoRa parameter)");
  }

  else if (g_Receive_cmd[0] == 0xFE && g_Receive_cmd[1] == 0xA0 && g_Receive_cmd[2] == 0x12) // Set LoRa parameter commond
  {
      Receive_crc8 = GetCrc8(&g_Receive_cmd[4], 20);

      if (Receive_crc8 == g_Receive_cmd[g_len - 7])
      {
        if (Verify_Device_Type(g_Receive_cmd[7], g_Receive_cmd[8]) == true)
        {
          if (Verify_Receive_SN_Code(&g_Receive_cmd[9]) == true)
          {
            unsigned char LoRa_para[6];
            for (unsigned char i = 0; i < 6; i++)
              LoRa_para[i] = g_Receive_cmd[18 + i];

            if (Save_LoRa_Para(LoRa_para) == true && Save_BKP_LoRa_Para(LoRa_para) == true)
              Serial.println("# Save LoRa Parameter success...");
            else
            {
              Serial.println("# Save LoRa Parameter failed...");
              LoRa_Message_Receipt(AssignIdErr, 1);
              Set_LoRa_Restart();
            }

            if (Set_LoRa_Para() == true)
            {
              LoRa_Message_Receipt(AssignIdOk, 2); //Message Receipt
              Serial.println("# Set LoRa Parameter success...");
            }
            else
            {
              Serial.println("# Set LoRa Parameter Error!");
              LoRa_Message_Receipt(AssignIdErr, 1);
              Set_LoRa_Restart();
            }
          }
          else
            Serial.println("<SN code error> section: (Reset inquire assign LoRa parameter)");
        }
        else
          Serial.println("Verify Device type error !!!");
      }
      else
        Serial.println("<Check Code Error> section: (Set LoRa parameter)");
  }
  
  else if (g_Receive_cmd[0] == 0xFE && g_Receive_cmd[1] == 0xB0 && g_Receive_cmd[2] == 0x11) // Set collect interval
  {
    Receive_crc8 = GetCrc8(&g_Receive_cmd[4], 13);

    if (Receive_crc8 == g_Receive_cmd[g_len - 7])
    {
      unsigned int alarm = (g_Receive_cmd[8] << 8) | g_Receive_cmd[9];

      if(Save_Collect_Time(alarm) == true)
      {
        Serial.println("Save collect time success...");
        LoRa_Message_Receipt(SetParamsOk, 2);
      }
      else
        LoRa_Message_Receipt(SetParamsErr, 1);

      g_Get_Para_Flag = true;
    }
    else
      Serial.println("<Check Code Error> section: (Set collect interval)");
  }
  else
    Serial.println("<Commond Error>  function: (LoRa_Receive_Data_Analysis) ");
}

/*
 *brief   : Initialize UV and Illumination sensors.
 *para    : None
 *return  : None
 */
void CJMCU6750_Init(void)
{
  CJMCU6750.begin(PB13, PB12);

  CJMCU6750.beginTransmission(I2C_ADDR);
  CJMCU6750.write((IT_1 << 2) | 0x02);
  CJMCU6750.endTransmission();
  delay(500);

  if (max44009.initialize())  //Lux
    Serial.println("Sensor MAX44009 found...");
  else
    Serial.println("Light Sensor missing !!!");   
}

/*
 *brief   : Read Lux and UV from sensor (I2C)
 *para    : Lux, UV
 *return  : None
 */
void Read_Lux_and_UV(unsigned long *lux, unsigned int *uv)
{
  unsigned char msb = 0, lsb = 0;
  unsigned long mLux_value = 0;

  #ifdef LUX_UV
  CJMCU6750.requestFrom(I2C_ADDR + 1, 1); //MSB
  delay(1);

  if (CJMCU6750.available())
    msb = CJMCU6750.read();

  CJMCU6750.requestFrom(I2C_ADDR + 0, 1); //LSB
  delay(1);

  if (CJMCU6750.available())
    lsb = CJMCU6750.read();

  *uv = (msb << 8) | lsb;
  *uv *= 0.005625;
  Serial.print("UV : "); //output in steps (16bit)
  Serial.println(*uv);
  #endif

  max44009.getMeasurement(mLux_value);

  *lux = mLux_value / 1000L;
  Serial.print("light intensity value:");
  Serial.print(*lux);
  Serial.println(" lux");
}

/*
 *brief   : According to the ID address and register(ModBus) address of the soil sensor, read temperature and humidity
 *para    : humidity, temperature, address
 *return  : None
 */
void Read_Solid_Humi_and_Temp(float *humi, unsigned int *temp, unsigned char *temp_flag, unsigned char addr)
{
  #if PR_3000_ECTH_N01
  unsigned char Send_Cmd[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00};
  #else
  unsigned char Send_Cmd[8] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00};
  #endif
  unsigned char Receive_Data[9] = {0};
  unsigned char Length = 0;
  unsigned int Send_CRC16 = 0;
  unsigned int Receive_CRC16 = 0;
  unsigned int Verify_CRC16 = 0;
  float hum = 65535.0;
  unsigned int humi_temp = 65535, tem_temp  = 65535;
  unsigned char temperature_flag = 0;

  Send_Cmd[0] = addr; //Get sensor address
  
  Send_CRC16 = N_CRC16(Send_Cmd, 6);
  
  Send_Cmd[6] = Send_CRC16 >> 8;
  Send_Cmd[7] = Send_CRC16 & 0xFF;
  
  RS485_Send_Enalbe();
  RS485_Serial.write(Send_Cmd, 8);
  RS485_Receive_Enable();
  delay(100);
  
  while (RS485_Serial.available() > 0)
  {
    Receive_Data[Length++] = RS485_Serial.read();
    delay(5);

    if (Length >= 9)
      Length = 0;
  }

  Verify_CRC16 = N_CRC16(Receive_Data, 7);

  Receive_CRC16 = Receive_Data[7] << 8 | Receive_Data[8];

  if (Receive_CRC16 == Verify_CRC16)
  {
    humi_temp = Receive_Data[3] << 8 | Receive_Data[4];
    #if PR_3000_ECTH_N01
      hum = (float)humi_temp / 100.0;
    #else
      hum = (float)humi_temp / 10.0;
    #endif

    tem_temp  = Receive_Data[5] << 8 | Receive_Data[6];
    temperature_flag = ((tem_temp >> 15) & 0x01);
  }

  *humi = hum;
  *temp = tem_temp;
  *temp_flag = temperature_flag;
}

/*
 *brief   : According to the ID address and register(ModBus) address of the soil sensor, read salt and conductivity
 *para    : salt, conductivity, address
 *return  : None
 */
void Read_Salt_and_Cond(unsigned int *salt, unsigned int *cond, unsigned char addr)
{
  //unsigned char Send_Cmd[8] = {0x01, 0x03, 0x00, 0x14, 0x00, 0x02, 0x00, 0x00};
  unsigned char Send_Cmd[8] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00};
  unsigned char Receive_Data[9] = {0};
  unsigned char Length = 0;
  unsigned int Send_CRC16 = 0;
  unsigned int Receive_CRC16 = 0;
  unsigned int Verify_CRC16 = 0;
  unsigned int salt_temp = 65535;
  unsigned int cond_temp = 65535;

  Send_Cmd[0] = addr; //Get sensor address
  
  Send_CRC16 = N_CRC16(Send_Cmd, 6);
  
  Send_Cmd[6] = Send_CRC16 >> 8;
  Send_Cmd[7] = Send_CRC16 & 0xFF;
  
  RS485_Send_Enalbe();
  RS485_Serial.write(Send_Cmd, 8);
  RS485_Receive_Enable();
  delay(100);
  
  while (RS485_Serial.available() > 0)
  {
    Receive_Data[Length++] = RS485_Serial.read();
    delay(5);

    if (Length >= 9)
      Length = 0;
  }

  Verify_CRC16 = N_CRC16(Receive_Data, 7);

  Receive_CRC16 = Receive_Data[7] << 8 | Receive_Data[8];

  if (Receive_CRC16 == Verify_CRC16)
  {
    salt_temp = Receive_Data[5] << 8 | Receive_Data[6];
    cond_temp = Receive_Data[3] << 8 | Receive_Data[4];
  }

  *salt = salt_temp;
  *cond = cond_temp;
}

/*
 *brief   : Receipt to gateway native status message
 *para    : current status, receipt times
 *return  : None
 */
void LoRa_Message_Receipt(unsigned char status, unsigned char times)
{
  unsigned char Receipt_data[128] = {0};
  unsigned char Receipt_Index = 0;
  unsigned char HiByte, LoByte, flag;

  if (LoRa_Para_Self_Check(g_LoRa_Para) == false)
    Set_LoRa_Restart();

  if (SN_Self_Check(g_SN_Code) == false)
  {
  
  }

  Receipt_data[Receipt_Index++] = g_LoRa_Para[0];
  Receipt_data[Receipt_Index++] = g_LoRa_Para[1]; 
  Receipt_data[Receipt_Index++] = g_LoRa_Para[2]; 

  Receipt_data[Receipt_Index++] = 0xFE; //Frame head

  Receipt_data[Receipt_Index++] = 0xE0;
  Receipt_data[Receipt_Index++] = 0x12;

  Receipt_data[Receipt_Index++] = 0x14; // Data length

  Receipt_data[Receipt_Index++] = g_LoRa_Para[3];//AsksendID
  Receipt_data[Receipt_Index++] = g_LoRa_Para[4];
  Receipt_data[Receipt_Index++] = g_LoRa_Para[5];//Ask send channel.

  Receipt_data[Receipt_Index++] = DEVICE_TYPE_H;
  Receipt_data[Receipt_Index++] = DEVICE_TYPE_L;

  for (unsigned char i = 0; i < 9; i++)
    Receipt_data[Receipt_Index++] = g_SN_Code[i];

  Receipt_data[Receipt_Index++] = 0xFF; //Ignore road

  Receipt_data[Receipt_Index++] = Read_Software_Version_H();
  Receipt_data[Receipt_Index++] = Read_Software_Version_L();
  Receipt_data[Receipt_Index++] = Read_Hardware_Version_H();
  Receipt_data[Receipt_Index++] = Read_Hardware_Version_L();

  Receipt_data[Receipt_Index++] = status;

  Receipt_data[Receipt_Index++] = GetCrc8(&Receipt_data[7], 20);

  for (unsigned char i = 0; i < 6; i++)
    i % 2 == 0 ? Receipt_data[Receipt_Index++] = 0x0D : Receipt_data[Receipt_Index++] = 0x0A;

  Serial.println("Send Receipt...");

  for (unsigned char i = 0; i < Receipt_Index; i++)
  {
    Serial.print(Receipt_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  for (unsigned char i = 0; i < times; i++)
  {
    LoRa_Serial.write(Receipt_data, Receipt_Index);
    delay(500);
  }
}

/*
 *brief   : Send 11-factor sensor data to gateway 
 *para    : None
 *return  : None
 */
void LoRa_Send_Sensor_Data(void)
{
  unsigned char HiByte, LoByte, flag;
  unsigned char NumOfDot = 0;
  unsigned char Data_BCD[4] = {0};//把转好的BCD数据存入这个数组
  char weathertr[20] = {0};
  unsigned char SendBuf[128] = {0};
  unsigned char Sindex = 0;
  bool Temperature_Change_Flag = false;
  float Soli_Temp_value = 0.0;

  if (LoRa_Para_Self_Check(g_LoRa_Para) == false)
    Set_LoRa_Restart();

  CJMCU6750_Init();

  Sensor_Data.g_Temp = sht10.readTemperatureC();
  delay(10);
  Sensor_Data.g_Humi = sht10.readHumidity();
  delay(10);

  Read_Solid_Humi_and_Temp(&Sensor_Data.g_Solid_Humi, &Sensor_Data.g_Solid_Temp, &Sensor_Data.g_Solid_Temp_Flag, 0x01);
  delay(10);
  Read_Salt_and_Cond(&Sensor_Data.g_Salt, &Sensor_Data.g_Cond, 0x01);
  delay(10);
  Read_Lux_and_UV(&Sensor_Data.g_Lux, &Sensor_Data.g_UV);
  delay(10);

  Serial.print("temperature: ");
  Serial.println(Sensor_Data.g_Temp);
  Serial.print("humility: ");
  Serial.println(Sensor_Data.g_Humi);
  Serial.print("Solid temperature: ");
  Serial.println(Sensor_Data.g_Solid_Temp / 100);
  Serial.print("Solid humility: ");
  Serial.println(Sensor_Data.g_Solid_Humi);
  Serial.print("Solid salt: ");
  Serial.println(Sensor_Data.g_Salt);
  Serial.print("Solid cond: ");
  Serial.println(Sensor_Data.g_Cond);
  
  SendBuf[Sindex++] = g_LoRa_Para[0];
  SendBuf[Sindex++] = g_LoRa_Para[1];
  SendBuf[Sindex++] = g_LoRa_Para[2];

  SendBuf[Sindex++] = 0xFE; //帧头
  SendBuf[Sindex++] = 0xD0; //帧ID
  SendBuf[Sindex++] = 0x01;

  SendBuf[Sindex++] = 0x37; //数据长度

  SendBuf[Sindex++] = g_LoRa_Para[3];
  SendBuf[Sindex++] = g_LoRa_Para[4];
  SendBuf[Sindex++] = g_LoRa_Para[5];

  SendBuf[Sindex++] = Read_Software_Version_H(); //设备软件版本 V1.0 --- 0010
  SendBuf[Sindex++] = Read_Software_Version_L(); 
  SendBuf[Sindex++] = Read_Hardware_Version_H(); //设备硬件版本 V1.0 --- 0010
  SendBuf[Sindex++] = Read_Hardware_Version_L();

  SendBuf[Sindex++] = 0x00; //传感器类型码
  SendBuf[Sindex++] = 0x01;

  unsigned int BatVol = Get_Voltage();
  SendBuf[Sindex++] = highByte(BatVol);
  SendBuf[Sindex++] = lowByte(BatVol);

  //Air Temperature
  NumOfDot = 2;
  if (Sensor_Data.g_Temp > 150)
  {
    SendBuf[Sindex++] = 0xFF;
    SendBuf[Sindex++] = 0xFF;
  }
  else
  {
    if (Sensor_Data.g_Temp < 0)
    {
      Sensor_Data.g_Temp *= -1;
      Temperature_Change_Flag = true;
    }

    PackBCD((char *)Data_BCD, Sensor_Data.g_Temp, 4, NumOfDot);
    SendBuf[Sindex++] = Data_BCD[0];
    SendBuf[Sindex++] = Data_BCD[1];
  }

  if (Temperature_Change_Flag == true)
    SendBuf[Sindex++] = 0xF0 | NumOfDot;//最高位是1，表示负的数值

  else if (Temperature_Change_Flag == false)
    SendBuf[Sindex++] = 0xE0 | NumOfDot;//最高位是0，表示正的数值//10

  //Air Humidity
  memset(Data_BCD, 0x00, sizeof(Data_BCD));//清零Data_BCD数组
  NumOfDot = 2;
  if ((int)Sensor_Data.g_Humi > 100 || (int)Sensor_Data.g_Humi < 0)
  {
    SendBuf[Sindex++] = 0xFF;
    SendBuf[Sindex++] = 0xFF;
  }
  else
  {
    PackBCD((char *)Data_BCD, Sensor_Data.g_Humi, 4, NumOfDot);//把大气湿度转换成BCD码
    SendBuf[Sindex++] = Data_BCD[0];
    SendBuf[Sindex++] = Data_BCD[1];
  }
  SendBuf[Sindex++] = 0xE0 | NumOfDot;

  //Air light
  NumOfDot = 0;
  memset(Data_BCD, 0x00, sizeof(Data_BCD));
  memset(weathertr, 0x00, sizeof(weathertr));

  if (Sensor_Data.g_Lux > 200000)
  {
    SendBuf[Sindex++] = 0xFF;
    SendBuf[Sindex++] = 0xFF;
    SendBuf[Sindex++] = 0xFF;
    SendBuf[Sindex++] = 0xFF;
  }
  else
  {
    sprintf(weathertr, "%08ld", Sensor_Data.g_Lux);
    //该函数是将一个ASCII码字符串转换成BCD码
    ASC2BCD(Data_BCD, weathertr, strlen(weathertr));//读取BCD数据数组、ASCII码字符串、该字符串的长度
    //Serial.println(weathertr);
    SendBuf[Sindex++] = Data_BCD[0];
    SendBuf[Sindex++] = Data_BCD[1];
    SendBuf[Sindex++] = Data_BCD[2];
    SendBuf[Sindex++] = Data_BCD[3];
  }
  SendBuf[Sindex++] = 0xE0 | NumOfDot;

  //Air Pressure
  SendBuf[Sindex++] = 0xFF;
  SendBuf[Sindex++] = 0xFF;
  SendBuf[Sindex++] = 0xFF;
  SendBuf[Sindex++] = 0xFF;
  SendBuf[Sindex++] = 0xE1;

  //UV
  NumOfDot = 0;
  memset(Data_BCD, 0x00, sizeof(Data_BCD));

  if (Sensor_Data.g_UV > 100)
  {
    SendBuf[Sindex++] = 0xFF;
    SendBuf[Sindex++] = 0xFF;
  }
  else
  {
    PackBCD((char *)Data_BCD, Sensor_Data.g_UV, 4, NumOfDot);
    SendBuf[Sindex++] = Data_BCD[0];
    SendBuf[Sindex++] = Data_BCD[1];
  }
  SendBuf[Sindex++] = 0xE0 | NumOfDot;

  //CO2
  SendBuf[Sindex++] = 0xFF;
  SendBuf[Sindex++] = 0xFF;
  SendBuf[Sindex++] = 0xE0;

  //TVOC
  SendBuf[Sindex++] = 0xFF;
  SendBuf[Sindex++] = 0xFF;
  SendBuf[Sindex++] = 0xE0;

  //Solid temperature
  NumOfDot = 2;
  if (Sensor_Data.g_Solid_Temp >= 65535)
  {
    SendBuf[Sindex++] = 0xFF;
    SendBuf[Sindex++] = 0xFF;
  }
  else
  {
    if (Sensor_Data.g_Solid_Temp_Flag == 1)
    {
      #if PR_3000_ECTH_N01
        Soli_Temp_value = (float)(65536 - Sensor_Data.g_Solid_Temp)  / 100;
      #else
        Soli_Temp_value = (float)(65536 - Sensor_Data.g_Solid_Temp)  / 10;
      #endif
    }
    else
    {
      #if PR_3000_ECTH_N01
        Soli_Temp_value = (float)Sensor_Data.g_Solid_Temp / 100;
      #else
        Soli_Temp_value = (float)Sensor_Data.g_Solid_Temp / 10;
      #endif
    }

    PackBCD((char *)Data_BCD, Soli_Temp_value, 4, NumOfDot);
    SendBuf[Sindex++] = Data_BCD[0];
    SendBuf[Sindex++] = Data_BCD[1];
  }

  if (Sensor_Data.g_Solid_Temp_Flag == 1)
    SendBuf[Sindex++] = 0xF0 | NumOfDot;//最高位是1，表示负的数值
  else
    SendBuf[Sindex++] = 0xE0 | NumOfDot;//最高位是0，表示正的数值//10

  //Solid humidity
  memset(Data_BCD, 0x00, sizeof(Data_BCD));//清零Data_BCD数组
  NumOfDot = 2;
  
  if ((int)(Sensor_Data.g_Solid_Humi) > 100)
  {
    SendBuf[Sindex++] = 0xFF;
    SendBuf[Sindex++] = 0xFF;
  }
  else
  {
    PackBCD((char *)Data_BCD, Sensor_Data.g_Solid_Humi, 4, NumOfDot);//把大气湿度转换成BCD码
    SendBuf[Sindex++] = Data_BCD[0];
    SendBuf[Sindex++] = Data_BCD[1];
  }
  SendBuf[Sindex++] = 0xE0 | NumOfDot;

  //Solid Cond
  NumOfDot = 0;
  memset(Data_BCD, 0x00, sizeof(Data_BCD));

  if (Sensor_Data.g_Cond >= 65535)
  {
    SendBuf[Sindex++] = 0xFF;
    SendBuf[Sindex++] = 0xFF;
  }
  else
  {
    PackBCD((char *)Data_BCD, Sensor_Data.g_Cond, 4, NumOfDot);

    SendBuf[Sindex++] = Data_BCD[0];
    SendBuf[Sindex++] = Data_BCD[1];
  }
  SendBuf[Sindex++] = 0xE0 | NumOfDot;

  //Solid Salt
  NumOfDot = 0;
  memset(Data_BCD, 0x00, sizeof(Data_BCD));

  if (Sensor_Data.g_Salt >= 65535)
  {
    SendBuf[Sindex++] = 0xFF;
    SendBuf[Sindex++] = 0xFF;
  }
  else
  {
    PackBCD((char *)Data_BCD, Sensor_Data.g_Salt, 4, NumOfDot);
    SendBuf[Sindex++] = Data_BCD[0];
    SendBuf[Sindex++] = Data_BCD[1];
  }
  SendBuf[Sindex++] = 0xE0 | NumOfDot;

  UTCTime CurrentSec = 0;
  CurrentSec = InRtc.getTime();
  osal_ConvertUTCTime(&RtcTime, CurrentSec);

  ToBCD(RtcTime.year, &HiByte, &LoByte, &flag);

  SendBuf[Sindex++] = HiByte;
  SendBuf[Sindex++] = LoByte;
  SendBuf[Sindex++] = ByteTOBcd(RtcTime.month);
  SendBuf[Sindex++] = ByteTOBcd(RtcTime.day);
  SendBuf[Sindex++] = ByteTOBcd(RtcTime.hour);
  SendBuf[Sindex++] = ByteTOBcd(RtcTime.minutes);
  SendBuf[Sindex++] = ByteTOBcd(RtcTime.seconds);

  unsigned char crc8 = GetCrc8(&SendBuf[7], Sindex - 7);
  SendBuf[Sindex++] = crc8;

  for (unsigned char i = 0; i < 6; i++)
    i % 2 == 0 ? SendBuf[Sindex++] = 0x0D : SendBuf[Sindex++] = 0x0A;

  for (unsigned char i = 0; i < Sindex; i++)
  {
    Serial.print(SendBuf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  unsigned char End_Frame[9] = {g_LoRa_Para[0], g_LoRa_Para[1], g_LoRa_Para[2], 0x0D, 0x0A, 0x0D, 0x0A, 0x0D, 0x0A};
  LoRa_Serial.write(End_Frame, 9);
  delay(500);

  LoRa_Serial.write(&SendBuf[0], 30);
  delay(200);

  LoRa_Serial.write(&g_LoRa_Para[0], 3);
  LoRa_Serial.write(&SendBuf[0] + 30, Sindex - 30);
  delay(500);
}

/*
 *brief   : Request gateway to get native LoRa parameters.
 *para    : None
 *return  : None
 */
void Request_Get_LoRa_parameter(void)
{
  unsigned char Request_cmd[64] = {0};
  unsigned char SN_Dat[9] = {0};
  unsigned char Request_len = 0;
  unsigned char SN_Verify1, SN_Verify2;

  //Gateway's LoRa ID and Channel.
  Request_cmd[Request_len++] = GATEWAY_ID_H;
  Request_cmd[Request_len++] = GATEWAY_ID_L;
  Request_cmd[Request_len++] = GATEWAY_CH;

  Request_cmd[Request_len++] = 0xFE; //Frame head.

  Request_cmd[Request_len++] = 0xE0; //Frame type.
  Request_cmd[Request_len++] = 0x11;

  Request_cmd[Request_len++] = 0X12; //data length.

  //Native LoRa ID and channel.
  Request_cmd[Request_len++] = highByte(LoRa_para.local_ID);
  Request_cmd[Request_len++] = lowByte(LoRa_para.local_ID);
  Request_cmd[Request_len++] = LoRa_para.gateway_channel;

  Request_cmd[Request_len++] = 0X00;
  Request_cmd[Request_len++] = 0X01;

  //Native SN code.
  for (unsigned char i = 0; i < 9; i++)
    Request_cmd[Request_len++] = g_SN_Data[i];

  Request_cmd[Request_len++] = Read_Software_Version_H();
  Request_cmd[Request_len++] = Read_Software_Version_L();
  Request_cmd[Request_len++] = Read_Hardware_Version_H();
  Request_cmd[Request_len++] = Read_Hardware_Version_L();

  Request_cmd[Request_len++] = GetCrc8(&Request_cmd[7], 18); //CRC8

  //Frame end.
  for (unsigned char i = 0; i < 6; i++)
    i % 2 == 0 ? Request_cmd[Request_len++] = 0x0D : Request_cmd[Request_len++] = 0x0A;

  // for (unsigned char i = 0; i < Request_len; i++)
  // {
  //   Serial.print(Request_cmd[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
  // delay(100);

  LoRa_Serial.write(Request_cmd, Request_len); //LoRa send frame to gateway.
  delay(500);
  Request_len = 0;
}

unsigned char BCD_Chg_Dat(unsigned char dat)
{
    unsigned char dat1, dat2;
    dat1 = dat / 16;
    dat2 = dat % 16;
    dat2 = dat2 + dat1 * 10;
    return dat2;
}

unsigned char Dat_Chg_BCD(unsigned char dat)
{
    unsigned char dat1, dat2;
    dat1 = dat / 10;
    dat2 = dat % 10;
    dat2 = dat2 + dat1 * 16;
    return dat2;
}

/*
 *brief   : Awake device and then receive cmmond or data from upper LoRa
 *para    : None
 *return  : None
 */
void LoRa_AUX_Handler(void)
{
  //nvic_sys_reset();
  rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);
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
