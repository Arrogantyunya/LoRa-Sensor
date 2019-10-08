
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
#include "Private_RTC.h"
#include "private_sensor.h"
#include "receipt.h"

void Request_Access_Network(void);
//void Data_Communication_with_Gateway(bool Is_Priority_Rain_Flag);
void Data_Communication_with_Gateway();

void Sleep(void);
void Key_Reset_LoRa_Parameter(void);

unsigned char g_SN_Code[9] = { 0x00 }; //Defualt SN code is 0.

/*----------Function statement----------*/
void Sleep(void);
/*--------------------------------------*/

/****************Set Up*******************/
void setup()
{
	afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);


	iwdg_init(IWDG_PRE_256, 1000);  //6.5ms * 1000 = 6500ms.

	Serial1.begin(9600);         //DEBUG Serial1 baud
	LoRa_MHL9LF.BaudRate(9600);    //LoRa SoftwareSerial baud
	RS485_Serial.begin(9600);  //ModBus SoftwareSerial baud
	bkp_init(); //Initialize backup register.

	iwdg_feed();

	Some_Peripheral.Peripheral_GPIO_Config();//设置蜂鸣器，LED，功能按钮等引脚 
	LoRa_MHL9LF.LoRa_GPIO_Config();//配置LoRa模块相关引脚
	RS485_GPIO_Config();  //RS485 GPIO configuration
	EEPROM_Operation.EEPROM_GPIO_Config();//设置EEPROM读写引脚
	LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);//配置LoRa模式。
	delay(1000);
	LoRa_Command_Analysis.Receive_LoRa_Cmd();//从网关接收LoRa数据（网关 ---> 本机），接受的指令有通用指令和本设备私有指令。
	//Serial1.println("开机初始化");

	iwdg_feed();

#if SOFT_HARD_VERSION
	Vertion.Save_Software_version(0x00, 0x01);
	Vertion.Save_hardware_version(0x00, 0x01);
#endif

	Key_Reset_LoRa_Parameter();//按键清除LORA参数，就是把LORA的设置的标志位重新清除

	//Initialize LoRa parameter.LORA参数初始化
	if (LoRa_Para_Config.Verify_LoRa_Config_Flag() == false)//读取LORA设置的标志位，为false则没有设置过，则进入LORA的参数设置
	{
		LoRa_MHL9LF.Rewrite_GroupID();//重写入组ID
		LoRa_MHL9LF.Rewrite_ID();//重写入ID
		LoRa_MHL9LF.Parameter_Init();//初始化LoRa配置。
		LoRa_MHL9LF.IsReset(true);//决定LoRa是否重启
		LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);//配置LoRa模式。
		LoRa_Command_Analysis.Receive_LoRa_Cmd();//从网关接收LoRa数据（网关 ---> 本机），接受的指令有通用指令和本设备私有指令。
		LoRa_Para_Config.Save_LoRa_Config_Flag();//存储LORA设置的标志位
	}

	iwdg_feed();

	SN.Clear_SN_Access_Network_Flag(); //清除注册到服务器标志位
	Request_Access_Network();//检测是否已经注册到服务器成功，如果没有注册，则配置相关参数为默认参数

	while (SN.Self_Check(g_SN_Code) == false) 
	{
		iwdg_feed();
		LED_SELF_CHECK_ERROR;
		Serial1.println("Verify SN code ERROR, try to Retrieving SN code...");
		Message_Receipt.Request_Device_SN_and_Channel();
		delay(1000);
		iwdg_feed();
		LoRa_Command_Analysis.Receive_LoRa_Cmd();
		iwdg_feed();
		delay(3000);
		iwdg_feed();
	}
	Serial1.println("SN self_check success...");
	LED_RUNNING;

	Private_RTC.Set_Alarm();
}

void loop()
{
	iwdg_feed();

	private_sensor.Detect_Rain();

	//这里是得到电平的反转（雨量传感器的检测到晴天或雨天），然后开始发送新的实时数据
	if (g_Current_Rain_Status_Flag != g_Last_Rain_Status_Flag)
	{
		Serial1.println("Send real-time data of sensor...");//发送实时传感器数据
		Data_Communication_with_Gateway();
	}

	if (g_Send_Air_SensorData_Flag == true) 
	{
		g_Send_Air_SensorData_Flag = false;
		Serial1.println("Send cyclic data of sensor...");//发送传感器的周期数据
		Data_Communication_with_Gateway();
		Private_RTC.Set_Alarm();
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
	if (SN.Verify_SN_Access_Network_Flag() == false)//验证是否已经注册到服务器，0x55表示已经注册。
	{
		g_Access_Network_Flag = false;
		iwdg_feed();
		if (SN.Save_SN_Code(g_SN_Code) && SN.Save_BKP_SN_Code(g_SN_Code))
			Serial1.println("Init SN success...");

		if (Control_Info.Clear_Area_Number() && Control_Info.Clear_Group_Number())
			Serial1.println("Already Clear area number and grouop number...");

		unsigned char Default_WorkGroup[5] = { 0x01, 0x00, 0x00, 0x00, 0x00 };
		if (Control_Info.Save_Group_Number(Default_WorkGroup))
			Serial1.println("Save gourp number success...");

		LED_NO_REGISTER;
	}
	while (SN.Verify_SN_Access_Network_Flag() == false) 
	{
		iwdg_feed();
		if (digitalRead(K1) == LOW) 
		{
			iwdg_feed();
			delay(5000);
			iwdg_feed();
			if (digitalRead(K1) == LOW) 
			{
				Message_Receipt.Report_General_Parameter();//E011

				while (digitalRead(K1) == LOW)
					iwdg_feed();
			}
		}
		LoRa_Command_Analysis.Receive_LoRa_Cmd();
		iwdg_feed();
	}
	g_Access_Network_Flag = true;
}

/*
 *brief   : Send sensor data to gateway and then received some parameters from gateway.
			向网关发送传感器数据，然后从网关接收一些参数。
 *para    : None
 *return  : None
 */
void Data_Communication_with_Gateway(void)//与网关的数据通信
{
	unsigned char Get_Para_num = 0;
	unsigned long wait_time = millis();

	//If it don't receive a message from the gateway.
	//如果它没有收到来自网关的消息。
	do {
		iwdg_feed();
		if (g_Get_Para_Flag == false) 
		{
			Message_Receipt.Send_Sensor_Data(); //Send sensor data to gateway again.再次发送传感器数据到网关。
			Get_Para_num++;
		}
		else
		{
			g_Last_Rain_Status_Flag = g_Current_Rain_Status_Flag;//最后一场雨状态标志 = 现时雨量状况标志
			g_Get_Para_Flag = false;
			break; //If receive acquisition parameter, break loop.如果接收到采集参数，断开循环。
		}
		while ((millis() < (wait_time + 10000)) && g_Get_Para_Flag == false)  //等待接收采集参数。
			LoRa_Command_Analysis.Receive_LoRa_Cmd(); //waiting to receive acquisition parameter.等待接收采集参数。

		wait_time = millis();

	} while (Get_Para_num < 3);

	if (Get_Para_num == 3)
	{	//If it don't receive message three times.如果3次没有收到信息。
		Serial1.println("No parameter were received !!!");
		Serial1.println("没有收到服务器任何参数！！！");
		
		//在这里进行LORA的切换，切换为组播对所有的卷膜机发送关棚指令
		//---------------------切换组播发送-------------------
		
		LoRa_MHL9LF.Parameter_ReInit();//初始化LoRa配置。
		LoRa_MHL9LF.IsReset(true);//决定LoRa是否重启
		LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);//配置LoRa模式。
		LoRa_Command_Analysis.Receive_LoRa_Cmd();//从网关接收LoRa数据（网关 ---> 本机），接受的指令有通用指令和本设备私有指令。

		iwdg_feed();

		unsigned char text1[9] = { 0x61, 0x0C,0x32,0x50,0x17, 0x00,0x00,0x00, 0x66 };
		unsigned char text2[9] = { 0x61, 0x00,0x00,0x00,0x71, 0x00,0x00,0x00, 0x77 };
		LoRa_Serial.write(text1, 9);
		Serial1.println("发送text1完成");
		delay(1500);
		LoRa_Serial.write(text2, 9);
		Serial1.println("发送text2完成");
		delay(1500);
		
		//---------------------切换正常发送-------------------
		LoRa_MHL9LF.Parameter_Init();//初始化LoRa配置。
		LoRa_MHL9LF.IsReset(true);//决定LoRa是否重启
		LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);//配置LoRa模式。
		LoRa_Command_Analysis.Receive_LoRa_Cmd();//从网关接收LoRa数据（网关 ---> 本机），接受的指令有通用指令和本设备私有指令。
	}
}

/*
 *brief   : This device goes sleep这个设备休眠了
 *para    : None
 *library : <Enerlib.h> <Arduino.h>
 */
void Sleep(void)
{
	Serial1.println("Enter Sleep>>>>>>");
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
	if (digitalRead(K1) == LOW) {
		delay(100);
		if (digitalRead(K1) == LOW) {
			//Some_Peripheral.Key_Buzz(600);
			Serial1.println("请按下按键2");
			iwdg_feed();
			delay(5000);
			iwdg_feed();
			if (digitalRead(K2) == LOW) {
				delay(100);
				if (digitalRead(K2) == LOW) {
					//Some_Peripheral.Key_Buzz(600);
					LoRa_Para_Config.Clear_LoRa_Config_Flag();
					Serial1.println("Clear LoRa configuration flag SUCCESS... <Key_Reset_LoRa_Parameter>");
					iwdg_feed();
				}
			}
		}
	}
}
