/******************************* slave_learn.c *******************************
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
 *              on ZW0102 standard slave, routing slave and enhanced slave devices.
 *              The module works for both battery operated and always listening
 *              devices.
 *
 * Author:   Henrik Holm
 *
 * Last Changed By:  $Author: jsi $
 * Revision:         $Revision: 22713 $
 * Last Changed:     $Date: 2012-05-03 18:02:00 +0200 (to, 03 maj 2012) $
 *
 ****************************************************************************/
/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <ZW_slave_api.h>
#include <ZW_uart_api.h>
#include <slave_learn.h>
/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/
#define ZW_LEARN_NODE_STATE_TIMEOUT 100   /* How long do we max. stay in */
                                          /* learn mode - 1 sec. */

#define LEARN_MODE_CLASSIC_TIMEOUT      1   /* Timeout count for classic innlusion */
#define LEARN_MODE_NWI_TIMEOUT          1800 /* Timeout count for network wide innlusion */
#define NWI_BASE_TIMEOUT                 50 /* Base delay for sending out inclusion req. */

#define MAX_NWI_REQUEST_TIMEOUT          27 /* Max number of increments in inclusiob request timeout */

BYTE learnStateHandle = 0xFF;
BYTE bRequestTimeoutHandle = 0xFF;
BOOL nodeInfoTransmitDone = TRUE;

WORD wInclusionTimeoutCount;
BYTE bRequestNWITimeoutCount;
BYTE bSavetRequestNWITimeout;

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/
BOOL learnState = FALSE;        /*Application can use this flag to check if learn
                                  mode is active*/

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
  BYTE bStatus,         /* IN Current status of Learnmode*/
  BYTE nodeID)					/* IN resulting nodeID */
{
  /* Stop sending inclusion requests */
  if (bRequestTimeoutHandle != 0xff)
  {
  	TimerCancel(bRequestTimeoutHandle);
  	bRequestTimeoutHandle = 0xff;
  }

  if (bStatus == ASSIGN_RANGE_INFO_UPDATE)
  {
    nodeInfoTransmitDone = FALSE;
    learnState = TRUE;
  }
  else
  {
    nodeInfoTransmitDone = TRUE;
  }

  if (bStatus == ASSIGN_COMPLETE)
  {
    /* Assignment was complete. Tell application */
  	if (learnState == TRUE)
  	{
      StopLearnInternal();
    }
    LearnCompleted(nodeID);
  }
  else
  {
    /* Learning in progress. Protocol will do timeout for us */
    if (learnStateHandle != 0xff)
    {
    	ZW_TIMER_CANCEL(learnStateHandle);
    }
    learnStateHandle = 0xff;
  }
}


/*========================   TransmitNodeInfoComplete   ======================
**    Function description
**      Callbackfunction called when the nodeinformation frame has
**      been transmitted. This function ensures that the Transmit Queue is not
**      flooded.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void TransmitNodeInfoComplete(BYTE bTXStatus)
{
  nodeInfoTransmitDone = TRUE;
}


/*============================   EndLearnNodeState   ========================
**    Function description
**      Timeout function that disables learnmode.
**      Should not be called directly.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void EndLearnNodeState(void)
{
  BYTE bTmpNodeID;

  if (!(--wInclusionTimeoutCount))
  {
    if (learnState)
    {
      StopLearnInternal();

      /* TO#2005 fix - Send the corredct node id to the application on timeout */
      MemoryGetID(NULL, &bTmpNodeID);
      LearnCompleted(bTmpNodeID);
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
void SendExplorerRequest()
{
  if (!(--bRequestNWITimeoutCount))
  {
    ZW_DEBUG_SEND_STRING("Send Explorer Request\n");

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
void StopLearnInternal()
{
  ZW_SetLearnMode(FALSE, NULL);

  if (learnStateHandle != 0xff)
  {
  	TimerCancel(learnStateHandle);
  	learnStateHandle = 0xff;
  }
  if (bRequestTimeoutHandle != 0xff)
  {
  	TimerCancel(bRequestTimeoutHandle);
  	bRequestTimeoutHandle = 0xff;
  }

  learnState = FALSE;
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
void StartLearnInternal(BYTE bMode)
{
  learnState = TRUE;
  ZW_SetLearnMode(bMode, LearnModeCompleted);

  if (learnStateHandle == 0xFF)
  {
    if (bMode == ZW_SET_LEARN_MODE_CLASSIC)
    {
      /*Disable Learn mode after x sec.*/
      wInclusionTimeoutCount = LEARN_MODE_CLASSIC_TIMEOUT;
    }
    else
    {
      /*Disable Learn mode after xxx sec.*/
      wInclusionTimeoutCount = LEARN_MODE_NWI_TIMEOUT;

      /* Start timer sending  out a explore inclusion request */
      bSavetRequestNWITimeout = 1;
      bRequestNWITimeoutCount = bSavetRequestNWITimeout;
      bRequestTimeoutHandle = TimerStart(SendExplorerRequest,
                                         NWI_BASE_TIMEOUT + (ZW_Random() & 0x3F), /* base + random(0..63) */
                                         TIMER_FOREVER);
    }

    learnStateHandle = ZW_TIMER_START(EndLearnNodeState, ZW_LEARN_NODE_STATE_TIMEOUT, TIMER_FOREVER);

    if ((nodeInfoTransmitDone) && (bMode == ZW_SET_LEARN_MODE_CLASSIC))
    {
      if (ZW_SEND_NODE_INFO(NODE_BROADCAST, 0, TransmitNodeInfoComplete))
      {
        nodeInfoTransmitDone = FALSE;
      }
    }
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
**        - Set the Slave in Learnmode
**        - Starts a one second timeout after which learn mode is disabled
**        - Broadcast the NODEINFORMATION frame once when called.
**      LearnCompleted will be called if a controller performs an assignID.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void StartLearnModeNow(BYTE bMode)
{
  if (learnState != FALSE) /* Learn mode is started, stop it */
  {
    StopLearnInternal();
  }

  /* Start Learn mode */

  if (bMode)
    StartLearnInternal(bMode);
}
