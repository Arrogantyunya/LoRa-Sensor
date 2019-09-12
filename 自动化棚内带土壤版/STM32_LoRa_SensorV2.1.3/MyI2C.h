
#ifndef _I2C_H
#define _I2C_H

#include <Arduino.h>

#define SDA     PB7
#define SCL     PB6

#define I2C_SDA_H       digitalWrite(SDA, HIGH)
#define I2C_SDA_L       digitalWrite(SDA, LOW)
#define I2C_SCL_H       digitalWrite(SCL, HIGH)
#define I2C_SCL_L       digitalWrite(SCL, LOW)

void I2C_INIT(void)
{
    pinMode(SCL, OUTPUT);
    pinMode(SDA, OUTPUT);
    delay(10);

    I2C_SCL_H;
    I2C_SDA_H;
    delay(10);
}

void I2C_SDA_OUT(void)
{
    pinMode(SDA, OUTPUT);
}

void I2C_SDA_IN(void)
{
    pinMode(SDA, INPUT_PULLUP);
}

void I2C_Start(void)
{
    I2C_SDA_OUT();

    I2C_SDA_H;
    I2C_SCL_H;
    delayMicroseconds(5);
    I2C_SDA_L;
    delayMicroseconds(6);
    I2C_SCL_L;
}

void I2C_Stop(void)
{
    I2C_SDA_OUT();
    
    I2C_SCL_L;
    I2C_SDA_L;
    I2C_SCL_H;
    delayMicroseconds(6);
    I2C_SDA_H;
    delayMicroseconds(6);
}

// 主机数据线拉低应答
void I2C_Ack(void)
{
    I2C_SCL_L;
    I2C_SDA_OUT();
    I2C_SDA_L;
    delayMicroseconds(2);
    I2C_SCL_H;
    delayMicroseconds(5);
    I2C_SCL_L;
}

//主机数据线拉高非应答
void I2C_NAck(void)
{
    I2C_SCL_L;
    I2C_SDA_OUT();
    I2C_SDA_H;
    delayMicroseconds(2);
    I2C_SCL_H;
    delayMicroseconds(5);
    I2C_SCL_L;
}

//等待从机应答， 从机返回1接收应答失败，返回0接收应答成功

unsigned char I2C_Wait_Ack(void)
{
    unsigned char tempTime = 0;

    I2C_SDA_IN();
    I2C_SDA_H;
    delayMicroseconds(1);
    I2C_SCL_H;
    delayMicroseconds(1);
    while (digitalRead(SDA))
    {
        tempTime++;
        if (tempTime > 250) //等待从机返回0失败
        {
            I2C_Stop();
            return 1;
        }
    }

    I2C_SCL_L;

    return 0;
}

//I2C 写一个字节
void I2C_Send_Byte(unsigned char txd)
{
    unsigned char i = 0;

    I2C_SDA_OUT();
    I2C_SCL_L; //拉低时钟线，允许数据线上电平变化

    for (i = 0; i < 8; i++)
    {
        if ((txd & 0x80) > 0) //从一个字节的高位开始传送
        {
            I2C_SDA_H;
        }
        else
        {
            I2C_SDA_L;
        }
        txd <<= 1;
        I2C_SCL_H; //时钟线拉高，这时数据线电平不能变化，让从机读取线上的电平
        delayMicroseconds(2);
        I2C_SCL_L;
        delayMicroseconds(2);
    }
}

//I2C读一个字节
unsigned char I2C_Read_Byte(unsigned char ack)
{
    unsigned char i = 0, receive = 0;

    I2C_SDA_IN();

    for (i = 0; i < 8; i++)
    {
        I2C_SCL_L;
        delayMicroseconds(2);
        I2C_SCL_H; //拉高时钟线，去读从机回过来的数据
        receive <<= 1;

        if (digitalRead(SDA))
        {
            receive++;
        }
        delayMicroseconds(1);
    }

    if (ack == 0)
    {
        I2C_NAck();
    }
    else
    {
        I2C_Ack();
    }

    return receive;
}



#endif

