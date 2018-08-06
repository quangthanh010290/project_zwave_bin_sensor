/******************************* ctrl_learn.c *******************************
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
 *              Copyright (c) 2001
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
 * Description: This file contains a sample of how learn mode could be implemented
 *              on ZW0201 standard controller.
 *              The module works for both battery operated and always listening
 *              devices.
 *
 * Author:   Henrik Holm
 *
 * Last Changed By:  $Author: jsi $
 * Revision:         $Revision: 18718 $
 * Last Changed:     $Date: 2010-09-15 13:15:58 +0200 (on, 15 sep 2010) $
 *
 ****************************************************************************/
/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <ZW_controller_api.h>
#include <ZW_uart_api.h>
#include <ctrl_learn.h>

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/
#define LEARN_MODE_CLASSIC_TIMEOUT        2   /* Timeout count for classic innlusion */
#define LEARN_MODE_NWI_TIMEOUT          720 /* Timeout count for network wide innlusion */
#define NWI_BASE_TIMEOUT                 50 /* Base delay for sending out inclusion req. */

#define MAX_NWI_REQUEST_TIMEOUT          27 /* Max number of increments in inclusiob request timeout */

#define ZW_LEARN_NODE_STATE_TIMEOUT 250     /* Learn mode base timeout  */

BYTE learnStateHandle = 0xFF;
BYTE bRequestTimeoutHandle = 0xFF;

WORD wInclusionTimeoutCount;

BYTE bRequestNWITimeoutCount;
BYTE bSavetRequestNWITimeout;

BYTE bLearnStarted = FALSE;
BYTE     bLastLearnMode;

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/
BOOL learnInProgress = FALSE;        /* Application can use this flag to check if learn
                                        mode is active */
void StartLearnInternal(BYTE bMode);

/****************************************************************************/
/*                               PROTOTYPES                                 */
/****************************************************************************/
void StopLearnInternal();

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/


/*============================   LearnModeCompleted   ========================
**    Function description
**      Callback which is called on learnmode completes
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void									/*RET	Nothing */
LearnModeCompleted(
  LEARN_INFO *glearnNodeInfo)					/* IN resulting nodeID */
{
  register BYTE bStatus;

  /* No node info transmit during neighbor discovery */
  ZW_DEBUG_SEND_BYTE('c');
  ZW_DEBUG_SEND_NUM(glearnNodeInfo->bStatus);

  bStatus = glearnNodeInfo->bStatus;

  /* Stop sending inclusion requests */
  if (bRequestTimeoutHandle != 0xff)
  {
    ZW_DEBUG_SEND_BYTE('t');
      ZW_DEBUG_SEND_NUM(bRequestTimeoutHandle);
  	TimerCancel(bRequestTimeoutHandle);
  	bRequestTimeoutHandle = 0xff;
  }

  if (bStatus == LEARN_MODE_STARTED)
  {
    learnInProgress = TRUE;
  }
  else if ((bStatus == LEARN_MODE_DONE) || (bStatus == LEARN_MODE_FAILED))
  {
    /* Assignment was complete. Tell application */
  	if (learnInProgress == TRUE)
  	{
  	  learnInProgress = FALSE;
  	  StopLearnInternal();

  	  if (bStatus == LEARN_MODE_DONE)
  	  {
        LearnCompleted(glearnNodeInfo->bSource, TRUE);
      }
      else
      {
        /* Restart learn mode */
        StartLearnInternal(bLastLearnMode);
      }
    }
  }
}

/*============================   EndLearnNodeState   ========================
**    Function description
**      Timeout function that disables learnmode.
**      Should not be called directly.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
EndLearnNodeState(void)
{
  if (!(--wInclusionTimeoutCount))
  {
    ZW_DEBUG_SEND_BYTE('E');

    if (!learnInProgress)
    {
      StopLearnInternal();
      LearnCompleted(0, FALSE);
    }
    return;
  }
}

/*============================   SendExplorerRequest   ========================
**    Function description
**      Timeout function that sends out a explorer inclusion reuest
**      Should not be called directly.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
SendExplorerRequest(void)
{
  if (!(--bRequestNWITimeoutCount))
  {
    ZW_DEBUG_SEND_BYTE('R');

    ZW_ExploreRequestInclusion();

    /* Increase timeout if we havent reached max */
    if (bSavetRequestNWITimeout < MAX_NWI_REQUEST_TIMEOUT)
      bSavetRequestNWITimeout++;

    bRequestNWITimeoutCount = bSavetRequestNWITimeout;
  }
}


/*============================   StopLearnInternal   ========================
**    Function description
**      - Disables learn mode
**      - Stop timer
**      - Disable network wide inclusion
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
StopLearnInternal(void)
{
  ZW_DEBUG_SEND_BYTE('l');

  ZW_SetLearnMode(FALSE, NULL);

  if (learnStateHandle != 0xff)
  {
    ZW_DEBUG_SEND_BYTE('w');
    ZW_DEBUG_SEND_NUM(learnStateHandle);
  	TimerCancel(learnStateHandle);
  	learnStateHandle = 0xff;
  }
  if (bRequestTimeoutHandle != 0xff)
  {
    ZW_DEBUG_SEND_BYTE('t');
    ZW_DEBUG_SEND_NUM(bRequestTimeoutHandle);
  	TimerCancel(bRequestTimeoutHandle);
  	bRequestTimeoutHandle = 0xff;
  }

  bLearnStarted = FALSE;
}

/*============================   StartLearnInternal   ======================
**    Function description
**      Call this function from the application whenever learnmode
**      should be enabled.
**      This function do the following:
**        - Set the Slave in Learnmode
**        - Starts a one second timeout after which learn mode is disabled
**        - Broadcast the NODEINFORMATION frame once when called.
**      LearnCompleted will be called if a controller performs an assignID.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
StartLearnInternal(
  BYTE bMode)
{
  ZW_DEBUG_SEND_BYTE('L');
  ZW_DEBUG_SEND_NUM(bMode);

  bLearnStarted = TRUE;
  ZW_SetLearnMode(bMode, LearnModeCompleted);

  if (learnStateHandle == 0xFF)
  {
    bLastLearnMode = bMode;

    if (bMode == ZW_SET_LEARN_MODE_CLASSIC)
    {
      /*Disable Learn mode after 1 sec.*/
      wInclusionTimeoutCount = LEARN_MODE_CLASSIC_TIMEOUT;
    }
    else
    {
      /*Disable Learn mode after 240 sec.*/
      wInclusionTimeoutCount = LEARN_MODE_NWI_TIMEOUT;

      /* Start timer sending  out a explore inclusion request */
      bSavetRequestNWITimeout = 1;
      bRequestNWITimeoutCount = bSavetRequestNWITimeout;
      bRequestTimeoutHandle = TimerStart(SendExplorerRequest,
                                         NWI_BASE_TIMEOUT + (ZW_Random() & 0x3F), /* base + random(0..63) */
                                         TIMER_FOREVER);
      ZW_DEBUG_SEND_BYTE('T');
      ZW_DEBUG_SEND_NUM(bRequestTimeoutHandle);
    }

    learnStateHandle = TimerStart(EndLearnNodeState, ZW_LEARN_NODE_STATE_TIMEOUT, TIMER_FOREVER);

    ZW_DEBUG_SEND_BYTE('W');
    ZW_DEBUG_SEND_NUM(learnStateHandle);
  }
}


/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/
/*============================   StartLearnModeNow   ======================
**    Function description
**      Call this function from the application whenever learnmode
**      should be enabled.
**      This function do the following:
**        - Set the Controller in Learnmode
**        - Starts a one second timeout after which learn mode is disabled
**        - Broadcast the NODEINFORMATION frame once when called.
**      LearnCompleted will be called if a controller performs an assignID.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
StartLearnModeNow(
  BYTE bMode)
{
  /* If learn is in progress then just exit */
  if (learnInProgress)
  {
    return;
  }

  if (bLearnStarted) /* Learn mode is started, stop it */
  {
    StopLearnInternal();
  }

  /* Start Learn mode */
  if (bMode)
  {
    StartLearnInternal(bMode);
  }
}


/*============================   StopLearnModeNow   ======================
**    Function description
**      Call this function from the application whenever learnmode
**      should be disabled.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
BYTE
StopLearnModeNow(void)
{
  if (bLearnStarted && (!learnInProgress))
  {
    StopLearnInternal();
    return TRUE;
  }

  return FALSE;
}


/*==========================   ReArmLearnModeTimeout   =======================
**    Function description
**      Rearms the LearnMode timout handler and thereby extending the time
**      that the controller are in LearnMode/Receive
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
ReArmLearnModeTimeout(void)
{
  if (learnStateHandle != 0xFF)
  {
    TimerRestart(learnStateHandle);
  }
}
