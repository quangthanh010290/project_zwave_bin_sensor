/******************************* ZW_FLiRS.c *******************************
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
 *              Copyright (c) 2009
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
 *              Battery Operated Nodes in FLiRS mode.
 *
 * Author:   Oleg Zadorozhnyy
 *
 * Last Changed By:  $Author: oza $
 * Revision:         $Revision: 1.00 $
 * Last Changed:     $Date: 2009-03-24 15:27:36 +0200 (Ср, 25 Лют 2009) $
 *
 ****************************************************************************/


#define ZW_BEAM_RX_WAKEUP

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <ZW_slave_api.h>
#include <ZW_debug_api.h>
#include <ZW_power_api.h>
#include <ZW_uart_api.h>
#include <ZW_FLiRS.h>
#include <ZW_pindefs.h>
#include <slave_learn.h>

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/
#define LONG_SLEEP_TIMEOUT    200
#define ALARM_TIMEOUT_COUNT   3   /* Timeout on an alarm is ALARM_TIMEOUT_COUNT*LONG_SLEEP_TIMEOUT */

BYTE bSleepTimeout; /* Timer handler for sleep timer */
BYTE bSleepTimerCount;
static BYTE flirsState =0;

/*================================   GotoSleep   ============================
**    Power down the ZW0201 chip
**
**
**
**--------------------------------------------------------------------------*/
void GotoSleep()
{
  ZW_DEBUG_SEND_BYTE('G');

///  PIN_IN(Button, 1);

//  EX1 = 1;  /* enable int1 (button) before power down */
  /* Goto sleep */

  if(!learnState)
  {
    /* Request sleep mode, if not possible start timer again and try later*/
    if (!ZW_SetSleepMode(ZW_FREQUENTLY_LISTENING_MODE, ZW_INT_MASK_EXT1, 0))
    {
      FLiRS_SleepTimeoutStart(FALSE);
    }
  }
}

/*================================   FLiRS_SetState   ============================
**    Set FLIRS state according reason bitmask
**
**
**
**--------------------------------------------------------------------------*/
void FLiRS_SetState(BYTE reason)
{
  flirsState |= reason;
}

/*================================   FLiRS_ResetState   ============================
**    Reset FLIRS state according reason bitmask. Switch to Sleep mode if (flirsState == 0)
**
**
**
**--------------------------------------------------------------------------*/
void FLiRS_ResetState(BYTE reason)
{
  ZW_DEBUG_SEND_BYTE('T');
  ZW_DEBUG_SEND_BYTE('F');
  flirsState &= ~reason;
  if (flirsState == 0)
  {
    ZW_DEBUG_SEND_BYTE('R');
    ZW_DEBUG_SEND_NL();
    GotoSleep();
  }
  else
  {
    ZW_DEBUG_SEND_NUM(flirsState);
    ZW_DEBUG_SEND_NL();
  }
}


/*=============================  SleepTimeout   =========================
**    Timeout function for going back to sleep
**
**    Side effects:
**
**--------------------------------------------------------------------------*/

void SleepTimeout()
{
  ZW_DEBUG_SEND_BYTE('t');
  bSleepTimerCount--;
  if (bSleepTimerCount == 0)
  {
    ZW_DEBUG_SEND_BYTE('T');
    flirsState &= ~FL_FLIRS_WAKEUP;
    if (flirsState == 0)
    {
      ZW_DEBUG_SEND_BYTE('T');
      ZW_DEBUG_SEND_NL();
      bSleepTimeout = 0;
      GotoSleep();
    }
    else
    {
      ZW_DEBUG_SEND_NUM(flirsState);
      ZW_DEBUG_SEND_NL();
    }
  }
}

/*========================  FLiRS_SleepTimeoutStop   =========================
**    Stop the timeout timer for going back to sleep
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void FLiRS_SleepTimeoutStop()
{
  FLiRS_SetState(FL_FLIRS_WAKEUP);

  ZW_DEBUG_SEND_BYTE('s');
  ZW_DEBUG_SEND_BYTE('t');

  if (bSleepTimeout)
    if(TimerCancel(bSleepTimeout))
    {
      ZW_DEBUG_SEND_BYTE('1');
      ZW_DEBUG_SEND_BYTE(' ');

    }
    else
    {
      ZW_DEBUG_SEND_BYTE('0');
      ZW_DEBUG_SEND_BYTE(' ');
    }
  ZW_DEBUG_SEND_NUM(bSleepTimeout);
  ZW_DEBUG_SEND_BYTE(' ');

  bSleepTimeout = 0;
}

/*=============================  FLiRS_SleepTimeoutStart   ===================
**    Start the timeout timer for going back to sleep
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void FLiRS_SleepTimeoutStart(BOOL bLong)
{
  ZW_DEBUG_SEND_BYTE('S');
  ZW_DEBUG_SEND_NUM(bLong);
  if (bLong)
    bSleepTimerCount = ALARM_TIMEOUT_COUNT;
  else
    bSleepTimerCount = 2;

  if (!bSleepTimeout)
    bSleepTimeout = TimerStart(SleepTimeout, LONG_SLEEP_TIMEOUT, bSleepTimerCount);
  else
  {
    TimerCancel(bSleepTimeout);
    bSleepTimeout = TimerStart(SleepTimeout, LONG_SLEEP_TIMEOUT, bSleepTimerCount);
  }
  ZW_DEBUG_SEND_NUM(bSleepTimeout);
  ZW_DEBUG_SEND_BYTE(' ');

  if(bSleepTimeout == 0xff)
  {
    bSleepTimeout = 0;
  }
}

/*=============================  FLiRS_SleepTimeoutStart   ===================
**    Restart the timer if it is running. If it isn't running then do nothing
**    nothing.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void FLiRS_SleepTimeoutReStart(BOOL bLong)
{
  if (bSleepTimeout)
  {
    FLiRS_SleepTimeoutStart(bLong);
  }
}
