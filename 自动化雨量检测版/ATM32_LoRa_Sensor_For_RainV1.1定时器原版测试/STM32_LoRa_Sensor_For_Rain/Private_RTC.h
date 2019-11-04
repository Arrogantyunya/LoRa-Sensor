#ifndef _PRIVATE_RTC_H
#define _PRIVATE_RTC_H

#include "User_Clock.h"
#include <RTClock.h>

class date{
public:
    void Update_RTC(unsigned char *buffer);
    void Get_RTC(unsigned char *buffer);
private:
    void Init_Set_Alarm(void);
    void Set_Alarm(void);
};

extern date Private_RTC;

#endif