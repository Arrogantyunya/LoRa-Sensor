#ifndef _AT24CXX_H
#define _AT24CXX_H


#include "MyI2C.h"

#define AT24C01     127
#define AT24C02     255
#define AT24C04     511
#define AT24C08     1023
#define AT24C16     2047
#define AT24C32     4095
#define AT24C64     8191
#define AT24C128    16383
#define AT24C256    32767

#define EE_TYPE     AT24C02

unsigned char AT24CXX_ReadOneByte(unsigned int addr)
{
    unsigned char temp = 0;

    I2C_Start();

    if (EE_TYPE > AT24C16)
    {
        I2C_Send_Byte(0xA0);
        I2C_Wait_Ack();
        I2C_Send_Byte(addr >> 8);
    }
    else
    {
        I2C_Send_Byte(0xA0 + ((addr / 256) << 1)); //器件地址 + 数据地址
    }

    I2C_Wait_Ack();
    I2C_Send_Byte(addr % 256);

    I2C_Wait_Ack();

    I2C_Start();
    I2C_Send_Byte(0xA1);
    I2C_Wait_Ack();

    temp = I2C_Read_Byte(0); //0 代表主机非应答，也就是只读一个数据，然后非应答，结束通信
    I2C_NAck();
    I2C_Stop();

    return temp;
}

void AT24CXX_WriteOneByte(unsigned int addr, unsigned char dt)
{
    I2C_Start();

    if (EE_TYPE > AT24C16)
    {
        I2C_Send_Byte(0xA0);
        I2C_Wait_Ack();
        I2C_Send_Byte(addr >> 8);
    }
    else
    {
        I2C_Send_Byte(0xA0 + ((addr / 256) << 1));
    }

    I2C_Wait_Ack();
    I2C_Send_Byte(addr % 256);

    I2C_Wait_Ack();

    I2C_Send_Byte(dt);
    I2C_Wait_Ack();
    I2C_Stop();

    delay(10);
}

#endif
