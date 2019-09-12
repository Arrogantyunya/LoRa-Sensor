#ifndef _RS485_H
#define _RS485_H

#include <Arduino.h>

#define RS485_PWR       PB8 //power
#define RS485_DE        PB9 //R/W enalbe pin

#define PWR_485_ON      (digitalWrite(RS485_PWR, HIGH))
#define PWR_485_OFF     (digitalWrite(RS485_DE, LOW))

/*
 *brief   : Configurate RS485 module Pin
 *para    : None
 *return  : None
 */
void RS485_GPIO_Config(void)
{
  pinMode(RS485_PWR, OUTPUT);
  pinMode(RS485_DE, OUTPUT);
  delay(10);
}

/*
 *brief   : Enable RS485 writting
 *para    : None
 *return  : None
 */
void RS485_Send_Enalbe(void)
{
  digitalWrite(RS485_DE, HIGH);
  delay(100);
}

/*
 *brief   : Enable RS485 reading
 *para    : None
 *return  : None
 */
void RS485_Receive_Enable(void)
{
  delay(25);
  digitalWrite(RS485_DE, LOW);
}

#endif