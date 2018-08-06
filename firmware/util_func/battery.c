/******************************* battery.c *******************************
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
 * Description: Implements functions that make is easier to support
 *              Battery Operated Nodes
 *
 * Author:   Jonas Roum-Møller
 *
 * Last Changed By:  $Author: iza $
 * Revision:         $Revision: 23076 $
 * Last Changed:     $Date: 2012-07-05 14:21:34 +0200 (to, 05 jul 2012) $
 *
 ****************************************************************************/

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
/* Enhanced Slave - needed for battery operation (RTC timer) on 100 series */
/* 200 Series have WUT */
#ifdef ZW_SLAVE_32
  #include <ZW_slave_32_api.h>
#else
  #ifdef  ZW_SLAVE
    #include <ZW_slave_api.h>
  #endif
#endif

/* ASIC power management functionality */
#if defined (ZW020x) || defined(ZW030x)
  #include <ZW_power_api.h>
#endif

#include <ZW_sysdefs.h>

/* Allows data storage of application data even after reset */
#if defined (ZW020x) || defined(ZW030x)
  #include <ZW_non_zero.h>
#endif

#include <eeprom.h>
#include <ZW_uart_api.h>
#include <ZW_pindefs.h>
#include <ZW_evaldefs.h>

#include <self_heal.h>
#include <slave_learn.h>
#include <battery.h>

#include <ZW_TransportLayer.h>

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/
#ifndef TRANSMIT_OPTION_EXPLORE       /* if library do not support explorer frame, ignore it: */
#define TRANSMIT_OPTION_EXPLORE   0
#endif

/* WUT count, when decreased to 0, wake up */
//TO3500
//#pragma save
//#pragma userclass (xdata = NON_ZERO_VARS_APP)
//XDWORD wakeupCount;
//#pragma restore

#ifdef ZW_SELF_HEAL
#define NON_ZERO_SELF_HEAL_SIZE  4 // 3 bytes are used by self_heal.c
#else
#define NON_ZERO_SELF_HEAL_SIZE  0
#endif

#ifdef SECURITY
#define NON_ZERO_SECURITY_SIZE  16 // prngState[16] are used by AES_module.h
#else
#define NON_ZERO_SECURITY_SIZE  0
#endif


XDWORD wakeupCount  _at_ (NON_ZERO_START_ADDR + NON_ZERO_SELF_HEAL_SIZE + NON_ZERO_SECURITY_SIZE); 


/* Wakeup timeout - number of seconds before wakeup */
XDWORD sleepPeriod = 0;
BYTE sleepSecondsMSB = 0;
BYTE sleepSecondsM = 0;
BYTE sleepSecondsLSB = 0;

/* Data that must be maintained after powerdown */
#ifdef ZW010x
BYTE RTCHandle = 0xFF;
static void RTCAction( BYTE status, BYTE parm);
RTC_TIMER RTCAction_Timer = {0,                   /* status */
                             RTC_ALLDAYS, 01, 00, /* On  */
                             RTC_ALLDAYS, 01, 00, /* Off */
                             0,                   /* ONE SHOT */
                             RTCAction,        /* Function called if firing */
                             0};                  /* return param */
#endif

/* Keep alive time between power downs */
BYTE powerDownTimeout;
BYTE powerDownTimerHandle = 0xFF;



/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/
BYTE powerDownTicks = 0;
BYTE wakeUpReason;

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/

/*============================   StoreWakeupCount   ======================
**    Function stores wakeup count to EEPROM
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
StoreWakeupCount( void )
{
  ZW_DEBUG_SEND_BYTE('A');
  ZW_DEBUG_SEND_BYTE('A');
  ZW_MEM_PUT_BYTE(EEOFFSET_WAKEUP_COUNT_1, ((BYTE_P)&wakeupCount)[0]);
  ZW_MEM_PUT_BYTE(EEOFFSET_WAKEUP_COUNT_2, ((BYTE_P)&wakeupCount)[1]);
  ZW_MEM_PUT_BYTE(EEOFFSET_WAKEUP_COUNT_3, ((BYTE_P)&wakeupCount)[2]);
  ZW_MEM_PUT_BYTE(EEOFFSET_WAKEUP_COUNT_4, ((BYTE_P)&wakeupCount)[3]);
}


/*============================   GetWakeupCount   ======================
**    Function reads the remaining wakeups before WAKE_UP_NOTIFICATION
**    from EEPROM.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing - global variable is set       */
GetWakeupCount( void )
{
  ZW_DEBUG_SEND_BYTE('A');
  ZW_DEBUG_SEND_BYTE('B');
  if (ZW_MEM_GET_BYTE(EEOFFSET_MAGIC) == MAGIC_VALUE)
  {
    ((BYTE_P)&wakeupCount)[0] = ZW_MEM_GET_BYTE(EEOFFSET_WAKEUP_COUNT_1);
    ((BYTE_P)&wakeupCount)[1] = ZW_MEM_GET_BYTE(EEOFFSET_WAKEUP_COUNT_2);
    ((BYTE_P)&wakeupCount)[2] = ZW_MEM_GET_BYTE(EEOFFSET_WAKEUP_COUNT_3);
    ((BYTE_P)&wakeupCount)[3] = ZW_MEM_GET_BYTE(EEOFFSET_WAKEUP_COUNT_4);
  }
  else
  {
    wakeupCount = 0;
  }
}

/*============================   WakeupNotificationCallback   ======================
**    Callback function for sending wakeup notification
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
WakeupNotificationCallback( BYTE txStatus ) /*IN   Transmission result        */
{
  /* We did not get in contact with the Wakeup Node, dont expect no more information frame */
  if(txStatus != TRANSMIT_COMPLETE_OK || masterNodeID == 0xFF)
  {
    ZW_DEBUG_SEND_BYTE(' ');
    ZW_DEBUG_SEND_BYTE('A');
    ZW_DEBUG_SEND_BYTE('F');
    currentState = STATE_APPL_IDLE;
  }
  if(masterNodeID != 0xFF)
  {
#ifdef ZW_SELF_HEAL
    UpdateLostCounter(txStatus);
#endif
  }

  ZW_DEBUG_SEND_BYTE('A');
  ZW_DEBUG_SEND_BYTE('7');

}


/*============================   WakeupNotification   ======================
**    Function sends off the Wakeup notification command
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
WakeupNotification( void )
{
  currentState    = STATE_WAKEUP_NOTIFICATION; /* Prevent sleeping */
  /* Only send wakeup notifiers when sensor is node in a network */
  /* and a recovery operation is not in progress */
  ZW_DEBUG_SEND_BYTE('W');
  ZW_DEBUG_SEND_BYTE('n');
#ifdef ZW_SELF_HEAL
  if ((myNodeID != 0) && (currentHealMode == HEAL_NONE))
#else
  if ((myNodeID != 0))
#endif
  {
    ZW_DEBUG_SEND_BYTE('A');
    ZW_DEBUG_SEND_BYTE('6');
    txBuf.ZW_WakeUpNotificationFrame.cmdClass = COMMAND_CLASS_WAKE_UP;
    txBuf.ZW_WakeUpNotificationFrame.cmd = WAKE_UP_NOTIFICATION;

/*#ifdef ZW_SELF_HEAL
if(VerifyLostCount()){
    ZW_DEBUG_SEND_BYTE('A');
    ZW_DEBUG_SEND_BYTE('L');
} else {
#endif*/
//      if (!ZW_SendData((masterNodeID == 0xFF ? NODE_BROADCAST : masterNodeID),
//                      (BYTE *)&txBuf, sizeof(ZW_WAKE_UP_NOTIFICATION_FRAME),
//                      (masterNodeID != 0xFF ? (TRANSMIT_OPTION_AUTO_ROUTE|TRANSMIT_OPTION_ACK) : 0),
//                      WakeupNotificationCallback))

  if (!Transport_SendRequest((masterNodeID == 0xFF ? NODE_BROADCAST : masterNodeID),
            (BYTE *)&txBuf, sizeof(ZW_WAKE_UP_NOTIFICATION_FRAME),
                (masterNodeID != 0xFF ? (TRANSMIT_OPTION_RETURN_ROUTE|TRANSMIT_OPTION_ACK) : 0),
            WakeupNotificationCallback,
            (masterNodeID == 0xFF ? TRUE : FALSE)))
      {
        ZW_DEBUG_SEND_BYTE('W');
        ZW_DEBUG_SEND_BYTE('f');
        currentState = STATE_APPL_IDLE;
        WakeupNotificationCallback(TRANSMIT_COMPLETE_FAIL);
      }
      SetSleepPeriod();
/*#ifdef ZW_SELF_HEAL
    }
#endif*/ /* ZW_SELF_HEAL */
  }
  else
  {
    /* We are not in any network, go idle */
    currentState = STATE_APPL_IDLE;
    SetSleepPeriod();
  }
}


/*============================   PowerDownTimeoutFunction   ======================
**    When this function is called, it's time to power down
**    the sensor.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
PowerDownTimeoutFunction( void )
{
  ZW_DEBUG_SEND_BYTE('P');
  ZW_DEBUG_SEND_BYTE('h');
  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_NUM(learnState);
  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_NUM(currentState);
  ZW_DEBUG_SEND_BYTE(' ');

  if ((learnState == FALSE) && (currentState == STATE_APPL_IDLE || currentState == STATE_CHECK_INPUT))
  {
    /* The timer is activated once every second */
    /* When PowerDownTimeout reaches 0, it's time */
    /* to power down. */
    ZW_DEBUG_SEND_BYTE('P');
    ZW_DEBUG_SEND_BYTE('t');
    ZW_DEBUG_SEND_NUM(powerDownTicks);
    if (--powerDownTicks == 0)
    {
      keepAliveActive = FALSE;
      ZW_DEBUG_SEND_BYTE('P');
      ZW_DEBUG_SEND_BYTE('d');
#ifdef ZW_DEBUG
      while(UART_SendStatus()); // Flush UART
#endif
      LED_OFF(1);

//      EX1 = 1;  /* enable int1 before power down */
#ifdef ZW_DEBUG
      /* Use 59 to indicate sleep for 1 minute, this makes the WakeUpCount indicate minutes. */
      ZW_SetWutTimeout(59 - (DEFAULT_POWERDOWNTIMEOUT + DEFAULT_KEEPALIVETIMEOUT));
#else
      /* Sleep for 60 seconds */
      ZW_SetWutTimeout(59 - (DEFAULT_POWERDOWNTIMEOUT + DEFAULT_KEEPALIVETIMEOUT));
#endif /* ZW_DEBUG */
      ZW_SetSleepMode(ZW_WUT_MODE,ZW_INT_MASK_EXT1,0);

    }
  }
}


/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/


/*============================   UpdateWakeupCount   ======================
**    Updates the Wakeup count counter in EEPROM. If the counter
**    reaches zero, a wakeup information frame is sent.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
UpdateWakeupCount( void )
{
  ZW_DEBUG_SEND_BYTE('A');
  ZW_DEBUG_SEND_BYTE('5');

  //TO3500
  //GetWakeupCount();

  if (--wakeupCount <= 0)
  {
    /* It's time to send a Wakeup Info frame */
    currentState = STATE_WAKEUP_NOTIFICATION_START;
  }
  else
  {
    //TO3500
  //StoreWakeupCount();
  }
}


/*============================   SetSleepPeriod   ======================
**    This function converts the 24-bit timeout value to a
**    big endian DWORD value that determines how long the sensor sleeps.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
SetSleepPeriod( void )
{
  ZW_DEBUG_SEND_BYTE('A');
  ZW_DEBUG_SEND_BYTE('9');

#ifdef ZW010x
  InitRTCActionTimer();
#endif /*ZW010x*/

#if defined (ZW020x) || defined(ZW030x)

  ((BYTE_P)&sleepPeriod)[0] = 0;
  ((BYTE_P)&sleepPeriod)[1] = sleepSecondsMSB;
  ((BYTE_P)&sleepPeriod)[2] = sleepSecondsM;
  ((BYTE_P)&sleepPeriod)[3] = sleepSecondsLSB;


  /* Convert from seconds to minutes - SleepPeriod then becomes the number of WUT timeouts required */
  /* to send WAKE_UP_NOTIFICATION */
  wakeupCount = sleepPeriod / SECONDS_IN_MINUTE;
  //TO3500
  //StoreWakeupCount();
#endif /* ZW020x */
}



/*============================   StartPowerDownTimer   ======================
**    This function manages the startup of the timer that, when it
**    expires, turns off the sensor. Timer is only started if it is
**    not already running.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
StartPowerDownTimer( void )
{
  ZW_DEBUG_SEND_BYTE('T');
  ZW_DEBUG_SEND_BYTE('h');

  StopPowerDownTimer();

  if (powerDownTimerHandle == 0xFF)
  {
    ZW_DEBUG_SEND_BYTE('T');
    ZW_DEBUG_SEND_BYTE('a');

    powerDownTimerHandle = ZW_TIMER_START(PowerDownTimeoutFunction,
                           TIMER_ONE_SECOND,
                           TIMER_FOREVER);
    ZW_DEBUG_SEND_NUM(powerDownTimerHandle);
    ZW_DEBUG_SEND_BYTE('d');
  }
}


/*============================   StopPowerDownTimer   ======================
**    This function stops the power down timer if it's running.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
StopPowerDownTimer( void )
{
  if (powerDownTimerHandle != 0xFF)
  {
    ZW_DEBUG_SEND_BYTE('T');
    ZW_DEBUG_SEND_BYTE('s');
    if (ZW_TIMER_CANCEL(powerDownTimerHandle) == TRUE)
    {
      powerDownTimerHandle = 0xFF;
    }
  }
}

/*============================   SetDefaultBatteryConfiguration   ======================
**    Function resets configuration to default values.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
SetDefaultBatteryConfiguration( void )
{

  powerDownTimeout = DEFAULT_POWERDOWNTIMEOUT;
  masterNodeID = DEFAULT_MASTER_NODEID;

  sleepSecondsMSB = DEFAULT_SLEEP_SECONDS_MSB;
  sleepSecondsM = DEFAULT_SLEEP_SECONDS_M;
  sleepSecondsLSB = DEFAULT_SLEEP_SECONDS_LSB;
#ifdef ZW010x
  RTCHandle = 0xFF;
#endif /* ZW010x */
  sleepPeriod = 0;

  SetSleepPeriod();

}


/*============================   SaveBatteryConfiguration   ======================
**    This function saves the current configuration to EEPROM
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
SaveBatteryConfiguration( void )
{
  ZW_MEM_PUT_BYTE(EEOFFSET_MASTER_NODEID, masterNodeID);
  ZW_MEM_PUT_BYTE(EEOFFSET_POWERDOWNTIMEOUT, powerDownTimeout);
  ZW_MEM_PUT_BYTE(EEOFFSET_SLEEP_PERIOD_1, sleepSecondsMSB);
  ZW_MEM_PUT_BYTE(EEOFFSET_SLEEP_PERIOD_2, sleepSecondsM);
  ZW_MEM_PUT_BYTE(EEOFFSET_SLEEP_PERIOD_3, sleepSecondsLSB);
#ifdef ZW010x
  ZW_MEM_PUT_BYTE(EEOFFSET_RTC_TIMER_HANDLE, RTCHandle);
#endif /* ZW010x */
}


/*============================   LoadConfiguration   ======================
**    This function loads the application settings from EEPROM.
**    If no settings are found, default values are used and saved.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /* RET  Nothing      */
LoadBatteryConfiguration( void )
{
    masterNodeID = ZW_MEM_GET_BYTE(EEOFFSET_MASTER_NODEID);
    powerDownTimeout = ZW_MEM_GET_BYTE(EEOFFSET_POWERDOWNTIMEOUT);
    sleepSecondsMSB = ZW_MEM_GET_BYTE(EEOFFSET_SLEEP_PERIOD_1);
    sleepSecondsM = ZW_MEM_GET_BYTE(EEOFFSET_SLEEP_PERIOD_2);
    sleepSecondsLSB = ZW_MEM_GET_BYTE(EEOFFSET_SLEEP_PERIOD_3);
#ifdef ZW010x
    RTCHandle = ZW_MEM_GET_BYTE(EEOFFSET_RTC_TIMER_HANDLE);
#endif /* ZW010x */
    sleepPeriod = 0;
//TO3500    
//    SetSleepPeriod();
}


void
WakeupCallback(BYTE txStatus)
{
  if(txStatus != TRANSMIT_COMPLETE_OK){
   currentState = STATE_APPL_IDLE;
  } else {
   currentState = STATE_WAKEUP_NOTIFICATION;
  }


}

/*============================   HandleWakeupFrame   ======================
**
**--------------------------------------------------------------------------*/
void                   /* RET  Nothing      */
HandleWakeupFrame(ZW_APPLICATION_TX_BUFFER *pCmd, BYTE txOption, BYTE sourceNode)
{
  BYTE param1 = ((BYTE_P)pCmd)[OFFSET_PARAM_1];
  BYTE param2 = ((BYTE_P)pCmd)[OFFSET_PARAM_2];
  BYTE param3 = ((BYTE_P)pCmd)[OFFSET_PARAM_3];
  BYTE param4 = ((BYTE_P)pCmd)[OFFSET_PARAM_4];
  BYTE param5 = ((BYTE_P)pCmd)[OFFSET_PARAM_5];
  BYTE param6 = ((BYTE_P)pCmd)[OFFSET_PARAM_6];

  if (pCmd->ZW_Common.cmd == WAKE_UP_INTERVAL_SET)
  {
    ZW_DEBUG_SEND_BYTE('W');
    ZW_DEBUG_SEND_BYTE('0');
    sleepSecondsMSB = param1;
    sleepSecondsM = param2;
    sleepSecondsLSB = param3;
    masterNodeID = param4;
    SetSleepPeriod();
    SaveBatteryConfiguration();
  }
  else if(pCmd->ZW_Common.cmd == WAKE_UP_INTERVAL_GET)
  {
    ZW_DEBUG_SEND_BYTE('W');
    ZW_DEBUG_SEND_BYTE('1');
    txBuf.ZW_WakeUpIntervalReportFrame.cmdClass = pCmd->ZW_Common.cmdClass;
    txBuf.ZW_WakeUpIntervalReportFrame.cmd = WAKE_UP_INTERVAL_REPORT;
    txBuf.ZW_WakeUpIntervalReportFrame.seconds1 = sleepSecondsMSB;
    txBuf.ZW_WakeUpIntervalReportFrame.seconds2 = sleepSecondsM;
    txBuf.ZW_WakeUpIntervalReportFrame.seconds3 = sleepSecondsLSB;
    txBuf.ZW_WakeUpIntervalReportFrame.nodeid = masterNodeID;

  Transport_SendReport(sourceNode,
            (BYTE *)&txBuf, sizeof(ZW_WAKE_UP_INTERVAL_REPORT_FRAME),
                (TRANSMIT_OPTION_RETURN_ROUTE | TRANSMIT_OPTION_ACK | TRANSMIT_OPTION_EXPLORE),
            WakeupCallback, FALSE);


//    ZW_SendData(sourceNode, (BYTE *)&txBuf, sizeof(ZW_WAKE_UP_INTERVAL_REPORT_FRAME),
//                (TRANSMIT_OPTION_AUTO_ROUTE | TRANSMIT_OPTION_ACK | TRANSMIT_OPTION_EXPLORE), WakeupCallback);
  }
  else if(pCmd->ZW_Common.cmd == WAKE_UP_NO_MORE_INFORMATION)
  {
    ZW_DEBUG_SEND_BYTE('W');
    ZW_DEBUG_SEND_BYTE('2');
    /* Delay powerdown just a little while to ensure "End Of Communication" */
    powerDownTimeout = DEFAULT_POWERDOWNTIMEOUT;
    currentState = STATE_APPL_IDLE;
  }
  else
  {
    ZW_DEBUG_SEND_BYTE('W');
    ZW_DEBUG_SEND_BYTE('3');
  }
}


#ifdef ZW010x
/****************************************************************************/
/* int0_int_routine                                                         */
/* interrupt routine used to wake up the asic when it is in power down.     */
/****************************************************************************/
void
int0_int_routine() interrupt INUM_INT0 using 1
{
  ZW_DEBUG_SEND_BYTE('A');
  ZW_DEBUG_SEND_BYTE('I');
}
#endif

#ifdef ZW010x
void InitRTCActionTimer(void)
{
  CLOCK cNow;
  CLOCK cNextWakeup;
  IBYTE NoOfDays;
  IBYTE NoOfHours;
  IBYTE NoOfMinutes;

  ZW_DEBUG_SEND_BYTE('A');
  ZW_DEBUG_SEND_BYTE('H');

  if (RTCHandle != 0xFF)
  {
    RTCTimerDelete(RTCHandle);
  }

  /* Initializes next ON event in the RTC_TIMER structure */
  ((BYTE_P)&sleepPeriod)[0] = 0;
  ((BYTE_P)&sleepPeriod)[1] = sleepSecondsMSB;
  ((BYTE_P)&sleepPeriod)[2] = sleepSecondsM;
  ((BYTE_P)&sleepPeriod)[3] = sleepSecondsLSB;

  NoOfDays = (BYTE)(sleepPeriod / SECONDS_IN_DAY);
  sleepPeriod -= ((DWORD)NoOfDays * SECONDS_IN_DAY);
  NoOfHours = sleepPeriod / SECONDS_IN_HOUR;
  sleepPeriod -= ((DWORD)NoOfHours * SECONDS_IN_HOUR);
  NoOfMinutes = sleepPeriod / SECONDS_IN_MINUTE;
  if (NoOfDays && (NoOfHours | NoOfMinutes == 0))
  {
    NoOfDays--;
  }

  /* When is now? */
  ClockGet(&RTCAction_Timer.timeOn);

  ZW_DEBUG_SEND_NL();
  ZW_DEBUG_SEND_NUM(RTCAction_Timer.timeOn.hour);
  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_NUM(RTCAction_Timer.timeOn.minute);
  RTCAction_Timer.timeOn.weekday = RTC_ALLDAYS; /* Wake up everyday */
  NoOfMinutes += RTCAction_Timer.timeOn.minute;
  NoOfHours += RTCAction_Timer.timeOn.hour;
  while (NoOfMinutes >= SECONDS_IN_MINUTE)
  {
    NoOfMinutes -= SECONDS_IN_MINUTE;
    NoOfHours++;
  }
  while (NoOfHours >= 24)
  {
    NoOfHours -= 24;
  }

  RTCAction_Timer.timeOn.minute = NoOfMinutes;
  RTCAction_Timer.timeOn.hour = NoOfHours;
  RTCAction_Timer.repeats = NoOfDays;

  ZW_DEBUG_SEND_NL();
  ZW_DEBUG_SEND_NUM(RTCAction_Timer.timeOn.hour);
  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_NUM(RTCAction_Timer.timeOn.minute);

  /* Copy timeon to timeoff */
  memcpy(&RTCAction_Timer.timeOff.weekday, &RTCAction_Timer.timeOn.weekday, 3);

  /* Start the timer */
  RTCHandle = RTCTimerCreate(&RTCAction_Timer, NULL);
  MemoryPutByte(EEOFFSET_RTC_TIMER_HANDLE, RTCHandle);
}
#endif /* ZW010x */

#ifdef ZW010x
/*==============================   intRTCAction ==============================
 *
 *  Function description
 *    RTC Timer callback function. This is called by ZW system when the
 *    RTC on/off timer is fired.
 *
 *  Side effects:
 *
 *--------------------------------------------------------------------------*/
static void     /*RET  Nothing          */
RTCAction(
  BYTE status,  /* IN  Timer status                             */
  BYTE parm)    /* IN  Parameter value set when creating timer  */
{

  ZW_DEBUG_SEND_BYTE('A');
  ZW_DEBUG_SEND_BYTE('J');

  /* Update network rediscovery count down time */
  UpdateNetworkUpdateCount( FALSE );

  if ((status & RTC_STATUS_FIRED) == 0)
  {
    // Switch on
    ZW_DEBUG_SEND_BYTE('A');
    ZW_DEBUG_SEND_BYTE('K');

    RTCTimerDelete(RTCHandle);

    LED_ON(4);
    currentState = STATE_WAKEUP_NOTIFICATION_START;
    /* TimerStart(WakeupNotification,1,TIMER_ONE_TIME); */
    InitRTCActionTimer();

    powerDownTicks = powerDownTimeout;
    StartPowerDownTimer();
  }
  else
  {
    /* Switch off */
    ZW_DEBUG_SEND_BYTE('A');
    ZW_DEBUG_SEND_BYTE('L');
  }

  currentState = STATE_APPL_IDLE;
}

#endif /* ZW010x */

