/******************************************************************************
  Filename:       User_Clock.h
  Revised:        $Date: 2011-02-02 14:21:10 -0800 (Wed, 02 Feb 2011) $
  Revision:       $Revision: 24962 $

  Description:    OSAL Clock definition and manipulation functions.


  Copyright 2008-2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
******************************************************************************/

#ifndef USER_CLOCK_H
#define USER_CLOCK_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
/*********************************************************************
 * MACROS
 */

#define U8  unsigned char   
#define U16  unsigned short  
#define U32  unsigned long 
  

#define	IsLeapYear(yr)	(!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

// number of seconds since 0 hrs, 0 minutes, 0 seconds, on the
// 1st of January 2000 UTC
	//0小时后的秒数，0分，0秒
	//2000年1月1日协调世界时

typedef U32 UTCTime;

// To be used with
typedef struct
{
  U8 seconds;  // 0-59
  U8 minutes;  // 0-59
  U8 hour;     // 0-23
  U8 week;
  U8 day;      // 0-30
  U8 month;    // 0-11
  U16 year;    // 2000+
} UTCTimeStruct;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */

  /*
   * Updates the OSAL clock and Timers from the MAC 320us timer tick.
   * 更新操作系统时钟和定时器从MAC 320us定时器滴答。
   */
  extern void osalTimeUpdate( void );

  /*
   * Set the new time.  This will only set the seconds portion of time and doesn't change the factional second counter.
   *设定新的时间。这将只设置秒的时间部分，而不改变派系秒计数器。
   *     newTime - number of seconds since 0 hrs, 0 minutes,0 seconds, on the 1st of January 2000 UTC
   *新时间- 2000协调世界时1月1日0小时后的秒数，0分，0秒
   */
  extern void osal_setClock( UTCTime newTime );

  /*
   * Gets the current time.  This will only return the seconds portion of time and doesn't include the factional second counter.
   * 获取当前时间。这将只返回秒的时间部分，不包括派系秒计数器。
   * returns: number of seconds since 0 hrs, 0 minutes,0 seconds, on the 1st of January 2000 UTC
   * 返回:自协调世界时2000年1月1日起0小时、0分、0秒后的秒数
   */
  extern UTCTime osal_getClock( void );

  /*
   * Converts UTCTime to UTCTimeStruct
   * 将UTCTime转换为UTCTimeStruct
   *
   * secTime - number of seconds since 0 hrs, 0 minutes,0 seconds, on the 1st of January 2000 UTC
   * tm - pointer to breakdown struct
   */
  extern void osal_ConvertUTCTime( UTCTimeStruct *tm, UTCTime secTime );

  /*
   * Converts UTCTimeStruct to UTCTime (seconds since 00:00:00 01/01/2000)
   * 将UTCTimeStruct转换为UTCTime(从00:00:00 01/01/2000开始的秒)
   *
   * tm - pointer to UTC time struct
   */
  extern UTCTime osal_ConvertUTCSecs( UTCTimeStruct *tm );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_CLOCK_H */
