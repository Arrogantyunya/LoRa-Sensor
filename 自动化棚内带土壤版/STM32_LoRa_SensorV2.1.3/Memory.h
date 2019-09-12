
#ifndef _Memory_H
#define _Memory_H

#include "AT24CXX.h"
#include "User_CRC8.h"
//#include <libmaple/bkp.h>

//EEPROM
#define SN_OPERATION_FLAG_ADDR          3
#define SN_BASE_ADDR                    6
#define SN_END_ADDR                     14
#define SN_VERIFY_ADDR                  17

#define SN_BKP_OPERATION_FLAG_ADDR      20   
#define SN_BKP_BASE_ADDR                22
#define SN_BKP_END_ADDR                 30
#define SN_BKP_VERIFY_ADDR              33

//EEPROM
#define LORA_OPERATION_FLAG_ADDR        36
#define LORA_BASE_ADDR                  39
#define LORA_END_ADDR                   47
#define LORA_VERIFY_ADDR                50

#define LORA_BKP_OPERATION_FLAG_ADDR    53
#define LORA_BKP_BASE_ADDR              56
#define LORA_BKP_END_ADDR               64
#define LORA_BKP_VERIFY_ADDR            67

//EEPROM     
#define COLLECT_TIME_FLAG_ADDR          70                          
#define COLLECT_TIME_HIGH_ADDR          71
#define COLLECT_TIME_LOW_ADDR           72

#define COLLECT_BKP_TIME_FLAG_ADDR      75                        
#define COLLECT_BKP_TIME_HIGH_ADDR      76
#define COLLECT_BKP_TIME_LOW_ADDR       77

//EEPROM
#define SOFTWARE_VERSION_BASE_ADDR      80
#define SOFTWARE_VERSION_END_ADDR       81
#define HARDWARE_VERSION_BASE_ADDR      82
#define HARDWARE_VERSION_END_ADDR       83

#define EEPROM1024_MAX_ADDR             255 //2KB = 2 * 1024bit = 2048 / 8 byte = 256byte.

#define EEPROM_WP                       PB5

enum Status_Back{
    STORE_FAILL = 0, OPERATION_OK, RESET_AND_OPERATION_OK
};

/*
 *brief     : Initialize the EEPROM read-write enable pin
 *para      : None
 *return    : None
 */
void EEPROM_WP_Init(void)
{
    pinMode(EEPROM_WP, OUTPUT);
    delay(10);
}

/*
 *brief     : Enable writting
 *para      : None
 *return    : None
 */
void EEPROM_Write_Enable(void)
{
    digitalWrite(EEPROM_WP, LOW);
    delay(10);
}

/*
 *brief     : Enable reading
 *para      : None
 *return    : None
 */
void EEPROM_Write_Disable(void)
{
    delay(10);
    digitalWrite(EEPROM_WP, HIGH);
}

/*
 *brief     : Verify that the SN is written
 *para      : None
 *return    : true or false
 */
bool Verify_SN_OPERATION_FLAG(void)
{
    if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 *brief     : Verify that the backup SN is written
 *para      : None
 *return    : true or false
 */
bool Verify_BKP_SN_OPERATION_FLAG(void)
{
    if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 *brief     : Write SN ID to EEPROM
 *para      : array
 *return    : true or false
 */
bool Write_SN(unsigned char *dat)
{
    unsigned char SN_dat[9];
    unsigned char SN_verify_temp;

    if (Verify_SN_OPERATION_FLAG() == true) //This device has been configured with SN
        return false;

    for (unsigned char i = 0; i < 9; i++) //Get SN data
        SN_dat[i] = dat[i];

    //Write SN data to EEPROM1024 and backup
    EEPROM_Write_Enable();

    for (unsigned char i = 0; i < 9; i++)
        AT24CXX_WriteOneByte(SN_BASE_ADDR + i, SN_dat[i]);

    //Write SN CRC8 to EEPROM10241024
    SN_verify_temp = GetCrc8(&SN_dat[0], 9);
    AT24CXX_WriteOneByte(SN_VERIFY_ADDR, SN_verify_temp);

    //Verify that SN is writing successfully.
    if ((AT24CXX_ReadOneByte(SN_VERIFY_ADDR) != SN_verify_temp))
    {
        EEPROM_Write_Disable();
        return false;
    }

    for (unsigned char i = 0; i < 9; i++)
    {
        if ((AT24CXX_ReadOneByte(SN_BASE_ADDR + i) != SN_dat[i]))
        {
            EEPROM_Write_Disable();
            return false;
        }
    }

    AT24CXX_WriteOneByte(SN_OPERATION_FLAG_ADDR, 0x55);
    EEPROM_Write_Disable();

    return true;
}

/*
 *brief     : Write backup SN ID to EEPROM
 *para      : array
 *return    : true or false
 */
bool Write_BKP_SN(unsigned char *dat)
{
    unsigned char SN_dat[9];
    unsigned char SN_verify_temp;

    if (Verify_BKP_SN_OPERATION_FLAG() == true) //This device has been configured with SN
        return false;

    for (unsigned char i = 0; i < 9; i++) //Get SN data
        SN_dat[i] = dat[i];

    //Write SN data to EEPROM1024 and backup
    EEPROM_Write_Enable();

    for (unsigned char i = 0; i < 9; i++)
        AT24CXX_WriteOneByte(SN_BKP_BASE_ADDR + i, SN_dat[i]);

    //Write SN CRC8 to EEPROM10241024
    SN_verify_temp = GetCrc8(&SN_dat[0], 9);
    AT24CXX_WriteOneByte(SN_BKP_VERIFY_ADDR, SN_verify_temp);

    //Verify that SN is writing successfully.
    if ((AT24CXX_ReadOneByte(SN_BKP_VERIFY_ADDR) != SN_verify_temp))
    {
        EEPROM_Write_Disable();
        return false;
    }

    for (unsigned char i = 0; i < 9; i++)
    {
        if ((AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + i) != SN_dat[i]))
        {
            EEPROM_Write_Disable();
            return false;
        }
    }

    AT24CXX_WriteOneByte(SN_BKP_OPERATION_FLAG_ADDR, 0x55);
    EEPROM_Write_Disable();

    return true;
}

/*
 *brief     : Read SN code form EEPROM
 *para      : SN code array
 *return    : true or false
 */
bool Read_SN(unsigned char *dat)
{
    unsigned char SN_dat[9] = {0};
    unsigned char SN_Verify = 0, SN_Verify_Temp;

    for (unsigned char i = 0; i < 9; i++)
        SN_dat[i] = AT24CXX_ReadOneByte(i + SN_BASE_ADDR);

    SN_Verify = AT24CXX_ReadOneByte(SN_VERIFY_ADDR);
    SN_Verify_Temp = GetCrc8(SN_dat, 9);

    if (SN_Verify != SN_Verify_Temp)
        return false;
    else
    {
        for (unsigned char i = 0; i < 9; i++)
            dat[i] = SN_dat[i];
    }

    return true;
}

/*
 *brief     : Read backup SN code form EEPROM
 *para      : SN code array
 *return    : true or false
 */
bool Read_BKP_SN(unsigned char *dat)
{
    unsigned char SN_dat[9] = {0};
    unsigned char SN_Verify = 0, SN_Verify_Temp;

    for (unsigned char i = 0; i < 9; i++)
        SN_dat[i] = AT24CXX_ReadOneByte(i + SN_BKP_BASE_ADDR);

    SN_Verify = AT24CXX_ReadOneByte(SN_BKP_VERIFY_ADDR);
    SN_Verify_Temp = GetCrc8(&SN_dat[0], 9);

    if (SN_Verify != SN_Verify_Temp)
        return false;
    else
    {
        for (unsigned char i = 0; i < 9; i++)
            dat[i] = SN_dat[i];
    }

    return true;
}

/*
 *brief     : Clear saved SN code flag
 *para      : None
 *return    : None
 */
void Clear_SN_Flag(void)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SN_OPERATION_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();
}

/*
 *brief     : Clear saved backup SN code flag
 *para      : None
 *return    : None
 */
void Clear_BKP_SN_Flag(void)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SN_BKP_OPERATION_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();    
}

/*
 *brief     : Check that two SN code are in good condition
 *para      : SN code (array)
 *return    : true or false
 */
bool SN_Self_Check(unsigned char *dat)
{
    bool Bool_Value;

    if (Read_SN(dat) == true && Read_BKP_SN(dat) == true)
    {
        Serial.println("Read SN and read backup SN success...");
        Bool_Value = true;
    }
    else if (Read_SN(dat) == true)
    {
        Serial.println("Read SN success but read backup SN failed...");
        Clear_BKP_SN_Flag();
        Write_BKP_SN(dat) == true ? Bool_Value = true : Bool_Value = false;
    }
    else if (Read_BKP_SN(dat) == true)
    {
        Serial.println("Read backup SN success but read SN failed...");
        Clear_SN_Flag();
        Write_SN(dat) == true ? Bool_Value = true : Bool_Value = false;
    }
    else
    {
        Serial.println("Read all SN failed !!!");
        Clear_SN_Flag();
        Clear_BKP_SN_Flag();
        Bool_Value = false;
    }

    return Bool_Value;
}

/*
 *brief     : Verify that the received SN code is equal to the SN code
 *para      : SN code array
 *return    : true or false
 */
bool Verify_SN(unsigned char *SN_data)
{
    for (unsigned char i = 0; i < 9; i++)
    {
        if (SN_data[i] != AT24CXX_ReadOneByte(SN_BASE_ADDR + i))
            return false;
    }
    return true;
}

/*
 *brief     : Verify that the received SN code is equal to the backup SN code
 *para      : SN code array
 *return    : true or false
 */
bool Verify_BKP_SN(unsigned char *SN_data)
{
    for (unsigned char i = 0; i < 9; i++)
    {
        if (SN_data[i] != AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + i))
            return false;
    }
    return true;
}

bool Verify_Receive_SN_Code(unsigned char *SN_code)
{

  for (unsigned char i = 0; i < 9; i++)
  {
    if ((SN_code[i] != AT24CXX_ReadOneByte(SN_BASE_ADDR + i)) && (SN_code[i] != 0x55))
        return false;
  }
  return true;
}

/*
 *brief     : Save LoRa parameter to EEPROM
 *para      : LoRa parameter array
 *return    : true or false
 */
bool Save_LoRa_Para(unsigned char *lora_data)
{
    unsigned char LoRa_Para[6];
    unsigned char LoRa_Verify;

    for (unsigned char i = 0; i < 6; i++)
        LoRa_Para[i] = lora_data[i];

    LoRa_Verify = GetCrc8(&LoRa_Para[0], 6);

    EEPROM_Write_Enable();
    for (unsigned char i = 0; i < 6; i++)
        AT24CXX_WriteOneByte(LORA_BASE_ADDR + i, LoRa_Para[i]);

    AT24CXX_WriteOneByte(LORA_VERIFY_ADDR, LoRa_Verify);

    for (unsigned char i = 0; i < 6; i++)
    {
        if (AT24CXX_ReadOneByte(LORA_BASE_ADDR + i) != LoRa_Para[i])
        {
            EEPROM_Write_Disable();
            return false;
        }
    }

    if (AT24CXX_ReadOneByte(LORA_VERIFY_ADDR) != LoRa_Verify)
    {
        EEPROM_Write_Disable();
        return false;
    }
    else
    {
        AT24CXX_WriteOneByte(LORA_OPERATION_FLAG_ADDR, 0x55);
        EEPROM_Write_Disable();
        return true;
    }
}

/*
 *brief     : Save LoRa backup parameter to EEPROM
 *para      : LoRa parameter array
 *return    : true or false
 */
bool Save_BKP_LoRa_Para(unsigned char *lora_data)
{
    unsigned char LoRa_Para[6];
    unsigned char LoRa_Verify;

    for (unsigned char i = 0; i < 6; i++)
        LoRa_Para[i] = lora_data[i];

    LoRa_Verify = GetCrc8(&LoRa_Para[0], 6);

    EEPROM_Write_Enable();

    for (unsigned char i = 0; i < 6; i++)
        AT24CXX_WriteOneByte(LORA_BKP_BASE_ADDR + i, LoRa_Para[i]);

    AT24CXX_WriteOneByte(LORA_BKP_VERIFY_ADDR, LoRa_Verify);

    for (unsigned char i = 0; i < 6; i++)
    {
        if (AT24CXX_ReadOneByte(LORA_BKP_BASE_ADDR + i) != LoRa_Para[i])
        {
            EEPROM_Write_Disable();
            return false;
        }
    }

    if (AT24CXX_ReadOneByte(LORA_BKP_VERIFY_ADDR) != LoRa_Verify)
    {
        EEPROM_Write_Disable();
        return false;
    }
    else
    {
        AT24CXX_WriteOneByte(LORA_BKP_OPERATION_FLAG_ADDR, 0x55);
        EEPROM_Write_Disable();
        return true;
    }
}

/*
 *brief     : Read LoRa parameter from EEPROM
 *para      : LoRa parameter array
 *return    : true or false
 */
bool Read_LoRa_Para(unsigned char *dat)
{
    unsigned char LoRa_Para[6];
    unsigned char LoRa_Verify, LoRa_Verify_Temp;

    for (unsigned char i = 0; i < 6; i++)
        LoRa_Para[i] = AT24CXX_ReadOneByte(LORA_BASE_ADDR + i);
    
    LoRa_Verify = AT24CXX_ReadOneByte(LORA_VERIFY_ADDR);
    LoRa_Verify_Temp =  GetCrc8(&LoRa_Para[0], 6);

    if (LoRa_Verify != LoRa_Verify_Temp)
        return false;
    else
    {
        for (unsigned char i = 0; i < 6; i++)
            dat[i] = LoRa_Para[i];

        return true;
    }
}

/*
 *brief     : Read backup LoRa parameter from EEPROM
 *para      : LoRa parameter array
 *return    : true or false
 */
bool Read_BKP_LoRa_Para(unsigned char *dat)
{
    unsigned char LoRa_Para[6];
    unsigned char LoRa_Verify, LoRa_Verify_Temp;

    for (unsigned char i = 0; i < 6; i++)
        LoRa_Para[i] = AT24CXX_ReadOneByte(LORA_BKP_BASE_ADDR + i);

    LoRa_Verify = AT24CXX_ReadOneByte(LORA_BKP_VERIFY_ADDR);
    LoRa_Verify_Temp =  GetCrc8(&LoRa_Para[0], 6);

    if (LoRa_Verify != LoRa_Verify_Temp)
        return false;
    else
    {
        for (unsigned char i = 0; i < 6; i++)
            dat[i] = LoRa_Para[i];

        return true;
    }
}

/*
 *brief     : Clear LoRa configuration flag bit.
 *para      : None
 *return    : None
 */
void Clear_LoRa_Flag(void)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(LORA_OPERATION_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();
}

/*
 *brief     : Clear backup LoRa configuration flag bit.
 *para      : None
 *return    : None
 */
void Clear_BKP_LoRa_Flag(void)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(LORA_BKP_OPERATION_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();
}

/*
 *brief     : LoRa parameter self-check 
 *para      : LoRa parameter array
 *return    : true or false
 */
bool LoRa_Para_Self_Check(unsigned char *LoRa_Dat)
{
    unsigned char Bool_Value;

    if (Read_LoRa_Para(LoRa_Dat) && Read_BKP_LoRa_Para(LoRa_Dat))
    {
        Serial.println("Read LoRa para and LoRa backup para success...");
        Bool_Value = true;
    }
    else if (Read_LoRa_Para(LoRa_Dat))
    {
        Serial.println("Read LoRa para success but read LoRa buckup para failed...");
        Clear_BKP_LoRa_Flag();
        Save_BKP_LoRa_Para(LoRa_Dat) == true ? Bool_Value = true : Bool_Value = false;   
    }
    else if (Read_BKP_LoRa_Para(LoRa_Dat))
    {
        Serial.println("Read backup LoRa para success but read LoRa para failed...");
        Clear_LoRa_Flag();
        Save_LoRa_Para(LoRa_Dat) == true ? Bool_Value = true : Bool_Value = false;
    }
    else
    {
        Serial.println("Read LoRa para and read LoRa backup para failed !!!");
        Clear_LoRa_Flag();
        Clear_BKP_LoRa_Flag();
        Bool_Value = false;
    }

    return Bool_Value;
}

/*
 *brief     : Verify that the LoRa is written
 *para      : None
 *return    : true or false
 */
bool Verify_LORA_OPERATION_FLAG(void)
{
    if (AT24CXX_ReadOneByte(LORA_OPERATION_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 *brief     : Verify that the LoRa is written
 *para      : None
 *return    : true or false
 */
bool Verify_BKP_LORA_OPERATION_FLAG(void)
{
    if (AT24CXX_ReadOneByte(LORA_BKP_OPERATION_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 *brief     : Save Received collect time from gateway to EEPROM. If it is the same as that already stored, it is not saved
 *para      : collect time (S)
 *return    : true or false
 */
bool Save_Collect_Time(unsigned int time)
{
    unsigned int Time = time;
    unsigned int Time_Temp;

    if (Time > 43200)
        return false;

    Time_Temp = AT24CXX_ReadOneByte(COLLECT_TIME_HIGH_ADDR) << 8 | AT24CXX_ReadOneByte(COLLECT_TIME_LOW_ADDR);

    if (Time_Temp == Time)
    {
        Serial.println("collect data already stored...");
        return true;
    }
    else
    {
        EEPROM_Write_Enable();        
        AT24CXX_WriteOneByte(COLLECT_TIME_HIGH_ADDR, Time >> 8);
        AT24CXX_WriteOneByte(COLLECT_TIME_LOW_ADDR, Time & 0xFF);
        AT24CXX_WriteOneByte(COLLECT_TIME_FLAG_ADDR, 0x55);
        EEPROM_Write_Disable();
        return true;
    }
}

/*
 *brief     : Read collect time from EEPROM
 *para      : None
 *return    : collect time (S)
 */
unsigned int Read_Collect_Time(void)
{
    unsigned int Time;
    Time = ((AT24CXX_ReadOneByte(COLLECT_TIME_HIGH_ADDR) << 8) | (AT24CXX_ReadOneByte(COLLECT_TIME_LOW_ADDR)));

    return Time;
}

/*
 *brief     : Verify that the collect time has been stored.
 *para      : None.
 *return    : true or false.
 */
bool Verify_COLLECT_TIME_FLAG(void)
{
    if (AT24CXX_ReadOneByte(COLLECT_TIME_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 *brief     : Save the software verstion of this device to EEPROM
 *para      : Software version high byte, Software version low byte
 *return    : None
 */
void Save_Software_Version(unsigned char number_high, unsigned char number_low)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SOFTWARE_VERSION_BASE_ADDR, number_high);
    AT24CXX_WriteOneByte(SOFTWARE_VERSION_END_ADDR, number_low);
    EEPROM_Write_Disable();
}

/*
 *brief     : Save the Hardware verstion of this device to EEPROM
 *para      : Hardware version high byte, Hardware version low byte
 *return    : None
 */
void Save_Hardware_Version(unsigned char number_high, unsigned char number_low)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(HARDWARE_VERSION_BASE_ADDR, number_high);
    AT24CXX_WriteOneByte(HARDWARE_VERSION_END_ADDR, number_low);
    EEPROM_Write_Disable();
}

/*
 *brief     : Read software vertion high byte from EEPROM
 *para      : None
 *return    : Software vertion high byte
 */
unsigned char Read_Software_Version_H(void)
{
    return (AT24CXX_ReadOneByte(SOFTWARE_VERSION_BASE_ADDR));
}

/*
 *brief     : Read software vertion low byte from EEPROM
 *para      : None
 *return    : Software vertion low byte
 */
unsigned char Read_Software_Version_L(void)
{
    return (AT24CXX_ReadOneByte(SOFTWARE_VERSION_END_ADDR));
}

/*
 *brief     : Read hardware vertion high byte from EEPROM
 *para      : None
 *return    : Hardware vertion high byte
 */
unsigned char Read_Hardware_Version_H(void)
{
    return (AT24CXX_ReadOneByte(HARDWARE_VERSION_BASE_ADDR));
}

/*
 *brief     : Read hardware vertion low byte from EEPROM
 *para      : None
 *return    : Hardware vertion low byte
 */
unsigned char Read_Hardware_Version_L(void)
{
    return (AT24CXX_ReadOneByte(HARDWARE_VERSION_END_ADDR));
}


#endif
