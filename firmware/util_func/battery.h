/******************************* battery.h *******************************
 *           #######
 *           ##  ##
 *           #  ##    ####   #####    #####  ##  ##   #####
 *             ##    ##  ##  ##  ##  ##      ##  ##  ##
 *            ##  #  ######  ##  ##   ####   ##  ##   ####
 *           ##  ##  ##      ##  ##      ##   #####      ##
 *          #######   ####   ##  ##  #####       ##  #####
 *                                           #####
 *          Z-Wave, the wireless language.
 *
 *              Copyright (c) 2006
 *              Zensys A/S
 *              Denmark
 *
 *              All Rights Reserved
 *
 *    This source file is subject to the terms and conditions of the
 *    Zensys Software License Agreement which restricts the manner
 *    in which it may be used.
 *
 *---------------------------------------------------------------------------
 *
 * Description: 
 *
 * Author:   Jonas Roum-Møller
 *
 * Last Changed By:  $Author: iza $
 * Revision:         $Revision: 21357 $
 * Last Changed:     $Date: 2011-09-23 15:18:44 +0200 (fr, 23 sep 2011) $
 *
 ****************************************************************************/
#ifndef _BATTERY_H_
#define _BATTERY_H_
/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

/****************************************************************************/
/*                              EXTERNAL DEFINED FUNCTIONS/DATA             */
/****************************************************************************/

/* PowerDownTimeout determines the number of seconds */
/* the sensor is kept alive between powerdowns.      */
/* The default is one second, which is probably      */
/* too little if you are routing in your network.    */
#define DEFAULT_POWERDOWNTIMEOUT    4

/* KeepAliveTimeout determines the number of seconds */
/* the sensor is kept alive when the button is       */
/* activated for more than KEEPALIVEACTIVATEPERIOD   */
/* seconds. This can be used when installing the     */
/* sensor in a network.                              */
/* Default keepalive is 30 seconds. */
#define DEFAULT_KEEPALIVETIMEOUT   30

/* Press and hold button for this period of time to enter keepalive mode */
/* Default is 3 seconds */
#define DEFAULT_KEEPALIVEACTIVATETIMEOUT  3

/* WAKEUPCOUNT holds the number of times WUT has been activated.  */
/* The value is stored in EEPROM and is used to determine */
/* when to send a Wakeup Information frame. */
/* Default is 5 which means that when the sensor has been woken */
/* 5 times it will send a Wakeup Information frame. */
#define DEFAULT_WAKEUPCOUNT 5

/* The following values determines how long the sensor sleeps */
/* i.e. it sets the delay before next wakeup. This value */
/* is stored in 24-bits and is converted during execution */
/* depending on the chip. Default value is 10 minutes */
/* 0x258 = 600 seconds */
#define DEFAULT_SLEEP_SECONDS_MSB     0x00
/* #define DEFAULT_SLEEP_SECONDS_M       0x02 */
/* #define DEFAULT_SLEEP_SECONDS_LSB     0x58 */
#define DEFAULT_SLEEP_SECONDS_M       0x00
#define DEFAULT_SLEEP_SECONDS_LSB     0x78 /// 0x96

#define DEFAULT_MASTER_NODEID       0xFF

#define SECONDS_IN_MINUTE    (DWORD)60
#define SECONDS_IN_HOUR      (DWORD)(60 * SECONDS_IN_MINUTE)
#define SECONDS_IN_DAY       (DWORD)(24 * SECONDS_IN_HOUR)

extern BYTE powerDownTicks;
extern BYTE RTCHandle;
extern BYTE wakeUpReason;

void HandleWakeupFrame( ZW_APPLICATION_TX_BUFFER *pCmd, BYTE txOption , BYTE sourceNode);
void UpdateWakeupCount( void );
void SetDefaultBatteryConfiguration( void );
void LoadBatteryConfiguration( void );
void SaveBatteryConfiguration( void );
void SetSleepPeriod( void );
void StartPowerDownTimer( void );
void StopPowerDownTimer( void );
void InitRTCActionTimer(void);
void WakeupNotification( void );

#endif /*_BATTERY_H_*/
