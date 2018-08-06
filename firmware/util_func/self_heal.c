/******************************* self_heal.c *******************************
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
 * Description: Implements functions that make is easy to support
 *              self-heal Operated Nodes
 *
 * Author:   Jonas Roum-Møller
 *
 * Last Changed By:  $Author: iza $
 * Revision:         $Revision: 21665 $
 * Last Changed:     $Date: 2011-11-07 13:25:35 +0100 (ma, 07 nov 2011) $
 *
 ****************************************************************************/

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
/* Enhanced Slave - needed for battery operation (RTC timer) on 100 series */
/* 200 Series have WUT */
#if defined(ZW_SLAVE_32)
#include <ZW_slave_32_api.h>
#elif defined(ZW_SLAVE_ROUTING)
#include <ZW_slave_routing_api.h>
#elif defined(ZW_SLAVE)
#include <ZW_slave_api.h>
#endif

/* Allows data storage of application data even after reset */
#if defined(ZW020x) || defined(ZW030x)
  #include <ZW_non_zero.h>
#endif

#include <ZW_pindefs.h>
#include <ZW_evaldefs.h>

#include <eeprom.h>
#include <ZW_uart_api.h>

#ifdef BATTERY
#include <battery.h>
#endif

#include <self_heal.h>


/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

/* Data that must be maintained after powerdown */
#if defined (ZW020x) || defined(ZW030x)
XBYTE networkUpdateDownCount    _at_ (NON_ZERO_START_ADDR);
XBYTE networkUpdateFailureCount _at_ (NON_ZERO_START_ADDR + sizeof(BYTE));
XBYTE lostCount                 _at_ (NON_ZERO_START_ADDR + 2*sizeof(BYTE));
#endif
#ifdef ZW010x
IBYTE networkUpdateDownCount;
IBYTE networkUpdateFailureCount;
IBYTE lostCount;
#endif

#ifndef BATTERY
BYTE networkUpdateTimerHandle = 0xFF;
#endif

BYTE searchNodeID;
BYTE rediscoveryTimerHandle = 0xFF;

BYTE oneMinute = 60;
BOOL abortHeal = FALSE;

void callbackDelayNextAskNodeForHelp( void );

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/
BYTE currentHealMode = HEAL_NONE;

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/

/*============================   callbackAskNodeForHelp   ======================
**    Function description
**
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
callbackAskNodeForHelp(
BYTE txStatus)        /*IN   Transmission result           */
{
  ZW_DEBUG_SEND_BYTE('H');
  ZW_DEBUG_SEND_BYTE('m');
  ZW_DEBUG_SEND_NUM(currentHealMode);

  LED_ON(3);

  if (txStatus == ZW_ROUTE_UPDATE_DONE)
  {
    HealComplete(TRUE);
  }
  else /* txStatus == ZW_ROUTE_LOST_FAILED */
  {
    ZW_DEBUG_SEND_BYTE('H');
    ZW_DEBUG_SEND_BYTE('m');
    ZW_DEBUG_SEND_BYTE('1');
    if (currentHealMode == HEAL_SUC)
    {
      /* We've asked the SUC but gotten no help or a bad response. */
      /* Now we try the WakeupNode if it's defined. */
      if (masterNodeID != 0xFF)
      {
        currentHealMode = HEAL_WAKEUPNODE;
        searchNodeID = masterNodeID;
      }
      else
      {
        currentHealMode = HEAL_GENERAL;
        searchNodeID = 1;
      }
    }
    else if (currentHealMode == HEAL_WAKEUPNODE)
    {
      /* No help from the wakeupnode so go into general */
      /* recovery mode where all nodes from nodeid 1 to ZW_MAX_NODES is */
      /* asked for help. */
      currentHealMode = HEAL_GENERAL;
      searchNodeID = 1;
    }
    else if (currentHealMode == HEAL_GENERAL)
    {
    ZW_DEBUG_SEND_BYTE('H');
    ZW_DEBUG_SEND_BYTE('m');
    ZW_DEBUG_SEND_BYTE('2');
    ZW_DEBUG_SEND_BYTE('n');
    ZW_DEBUG_SEND_NUM(searchNodeID);
      searchNodeID++;
      searchNodeID = (searchNodeID == myNodeID ? searchNodeID + 1 : searchNodeID);

      if (searchNodeID > ZW_MAX_NODES)
      {
        ZW_DEBUG_SEND_BYTE('H');
        ZW_DEBUG_SEND_BYTE('m');
        ZW_DEBUG_SEND_BYTE('M');
        /* We have reached the end of the line. Apparently there are no */
        /* nodes available at all and the recovery has failed. */
        /* Reset everything. */
        searchNodeID = 1;
        HealComplete(FALSE);
        currentHealMode = HEAL_NONE;
        currentState = STATE_APPL_IDLE;
        nextState = STATE_APPL_IDLE;
      }
    }

    /* Delay 1 second before calling ZW_RediscoveryNeeded again. */
    if (currentHealMode != HEAL_NONE)
    {
          ZW_DEBUG_SEND_BYTE('H');
    ZW_DEBUG_SEND_BYTE('m');
    ZW_DEBUG_SEND_BYTE('N');
       if(rediscoveryTimerHandle != 0xFF)
       {
         ZW_DEBUG_SEND_BYTE('f');
         ZW_DEBUG_SEND_NUM(rediscoveryTimerHandle);
         ZW_TIMER_CANCEL(rediscoveryTimerHandle);
         rediscoveryTimerHandle = 0xFF;
       }
       StartWatchdog();
       rediscoveryTimerHandle = ZW_TIMER_START(callbackDelayNextAskNodeForHelp, REDISCOVERY_TIMEOUT, TIMER_ONE_TIME);
       ZW_DEBUG_SEND_NUM(rediscoveryTimerHandle);
    }
  }
}


/*============================   AskNodeForHelp   ======================
**    Function description
**    Sends a ZW_RediscoveryNeeded message to the specified nodeid.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
AskNodeForHelp(
BYTE NodeID)           /*IN   ID of node where ZW_RediscoveryNeeded will be sent to. */
{
  ZW_DEBUG_SEND_BYTE('H');
  ZW_DEBUG_SEND_BYTE('3');
  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_NUM(NodeID);
  ZW_DEBUG_SEND_BYTE(' ');

  if (!ZW_REDISCOVERY_NEEDED(NodeID, callbackAskNodeForHelp))
  {
    ZW_DEBUG_SEND_BYTE('H');
    ZW_DEBUG_SEND_BYTE('3');
    ZW_DEBUG_SEND_BYTE('f');

    HealComplete(FALSE);
  }
}

/*============================   cbDelayNextAskNodeForHelp   ======================
**    Function description
**    The purpose of this timer callback function is to delay execution
**    of ZW_RediscoveryNeeded.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
callbackDelayNextAskNodeForHelp( void )
{
  ZW_DEBUG_SEND_BYTE('H');
  ZW_DEBUG_SEND_BYTE('5');
  LED_OFF(3);
  /* TO#2053 Do not cancel an expired timer */
  rediscoveryTimerHandle = 0xFF;

  AskNodeForHelp(searchNodeID);
}



/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/


/*============================   SetDefaultNetworkUpdateConfiguration   ======================
**    Function resets configuration to default values.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
SetDefaultNetworkUpdateConfiguration( void )
{
  lostCount = 0;
  networkUpdateDownCount = DEFAULT_NETWORK_UPDATE_COUNT;
}

/*============================   UpdateLostCounter   ======================
**    The "Lost Counter" is used to keep track of whether the sensor
**    can communicate with other nodes in the network. On each successful
**    transmission, the lost counter is reset to 0, otherwise it is
**    incremented.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
UpdateLostCounter(
BYTE txStatus)           /*IN   Transmission result           */
{

  ZW_DEBUG_SEND_BYTE('H');
  ZW_DEBUG_SEND_BYTE('0');

  if (txStatus != ZW_ROUTE_UPDATE_DONE)
  {
    ZW_DEBUG_SEND_BYTE('H');
    ZW_DEBUG_SEND_BYTE('f');

    lostCount++;

    /* It's not important to know exactly how many times our comms have failed*/
    /* it's enough to know that the threshold has been passed - hence this */
    /* adjustment to the passed parameter before saving it. */
    if (lostCount > DEFAULT_LOST_COUNTER_MAX)
    {
      lostCount = DEFAULT_LOST_COUNTER_MAX + 1;
    }
  }
  else
  {
    ZW_DEBUG_SEND_BYTE('H');
    ZW_DEBUG_SEND_BYTE('s');
    /* Default signals that everything is ok and "Lost Count" is reset to 0 */
    lostCount = 0;
  }
  VerifyLostCount();
}

/*===========================   GetRandomNodeTime   ===========================
 *
 *  Function description
 *    Get a pseudo-random time
 *
 *  Side effects:
 *
 *--------------------------------------------------------------------------*/
static BYTE                    /*RET A pseudo random nodeID */
GetRandomNetUpdateCount( void ) /* IN Nothing */
{
# if 0		// IZ:Fix TO# 02957 
  register BYTE delay;
  delay = ZW_Random();

  ZW_DEBUG_SEND_BYTE('N');
  ZW_DEBUG_SEND_BYTE('r');
  ZW_DEBUG_SEND_NUM(delay);

  /* Slow down if really lost */
  if(networkUpdateFailureCount > 50){
    networkUpdateFailureCount =  networkUpdateFailureCount / 2;
  }

  while (delay <
      (( NETWORK_UPDATE_MIN_COUNT - (networkUpdateFailureCount * 2)) > 10 ?
         NETWORK_UPDATE_MIN_COUNT - (networkUpdateFailureCount * 2) :
         NETWORK_UPDATE_MIN_COUNT ))
  {
    delay += 5;
  }

  while (delay >
      (( NETWORK_UPDATE_MAX_COUNT - (networkUpdateFailureCount * 2)) > NETWORK_UPDATE_MIN_COUNT ?
         NETWORK_UPDATE_MAX_COUNT - (networkUpdateFailureCount * 2) :
         NETWORK_UPDATE_MAX_COUNT ))
  {
    delay -= 5;
  }
  ZW_DEBUG_SEND_BYTE('N');
  ZW_DEBUG_SEND_BYTE('d');
  ZW_DEBUG_SEND_NUM(delay);

  return delay;

#else
  register BYTE delay,result;

  delay = ZW_Random();
  result = NETWORK_UPDATE_MIN_COUNT + delay % (NETWORK_UPDATE_MAX_COUNT - NETWORK_UPDATE_MIN_COUNT); 

  ZW_DEBUG_SEND_BYTE('_');
  ZW_DEBUG_SEND_BYTE('N');
  ZW_DEBUG_SEND_BYTE('r');
  ZW_DEBUG_SEND_NUM(delay);
  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_BYTE('N');
  ZW_DEBUG_SEND_BYTE('d');
  ZW_DEBUG_SEND_NUM(result);
  ZW_DEBUG_SEND_BYTE('_');

  return result;	
#endif
}


/*===========================   VerifyAssociatedTransmitCallback    ==================
 *
 *  Function description
 *    If ZW_RequestNewRouteDestinations failed, try to find the SUC
 *
 *  Side effects:
 *
 *--------------------------------------------------------------------------*/
void                    /*RET nothing */
VerifyAssociatedTransmitCallback( BYTE SUCStatus) /* IN SUCStatus  */
{
  if (SUCStatus == ZW_ROUTE_UPDATE_ABORT)
  {
   lostCount++;
  ZW_DEBUG_SEND_BYTE('V');
  ZW_DEBUG_SEND_BYTE('A');
  ZW_DEBUG_SEND_NUM(SUCStatus);

   VerifyLostCount();
  } else if ( SUCStatus == ZW_ROUTE_UPDATE_DONE){
    currentState = STATE_APPL_IDLE;
  }
}


/*===========================   VerifyAssociatedTransmit    ==================
 *
 *  Function description
 *    Verify the status of the transmit to an associated node, if failed
 *    request new Return Routes for the destinations listed in pDestList.
 *    As ZW_REQUEST_NEW_ROUTE_DESTINATIONS erase return route for destinations
 *    not in pDestList then we have to list all the Destinations we need
 *    Return routes for.
 *    As the usage of this function could potentially result in new
 *    Return Routes being requested after every transmit to a dead node it
 *    is advised only to call this after several tries have been made.
 *
 *  Side effects:
 *    If txStatus indicates failed transmit then new Destination Return Route
 *    are requested for all node listed in pDestList.
 *
 *
 *--------------------------------------------------------------------------*/
void                      /*RET nothing */
VerifyAssociatedTransmit(
  BYTE txStatus,
  BYTE attemptedNodeId   /* IN transmit status , the node id of the attempted transmit */
)
{
  if (txStatus != TRANSMIT_COMPLETE_OK)
  {
    ZW_DEBUG_SEND_BYTE('V');
    ZW_DEBUG_SEND_BYTE('f');
    ZW_DEBUG_SEND_NUM(txStatus);

    currentState = STATE_HEAL_ASSOCIATED_FAIL;
    nextState = STATE_APPL_IDLE;
    ZW_REQUEST_NEW_ROUTE_DESTINATIONS(&attemptedNodeId,1,VerifyAssociatedTransmitCallback);
  }
}




/*============================   UpdateLostCounterCallback   ======================
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
UpdateNetworkUpdateCountCallback(BYTE txStatus )           /*IN   Nothing           */
{
   if (txStatus == ZW_ROUTE_UPDATE_ABORT)
   {
     ZW_DEBUG_SEND_BYTE('N');
     ZW_DEBUG_SEND_BYTE('f');
     ZW_DEBUG_SEND_NUM(lostCount);
     lostCount++;
     VerifyLostCount();
   } else {
     lostCount = 0;
   }

}

/*============================   UpdateLostCounterOneMinute   ======================
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
UpdateNetworkUpdateCountOneMinute(void )           /*IN   Nothing           */
{
  oneMinute--;
  if(oneMinute <= 0)
    {
     ZW_DEBUG_SEND_BYTE('T');
     ZW_DEBUG_SEND_BYTE('m');
     oneMinute = 60;
     UpdateNetworkUpdateCount( FALSE );
    }
}

/*========================   UpdateNetworkUpdateCount   ===================
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
UpdateNetworkUpdateCount( BOOL reset )           /*IN   BOOL, reset if true, else increment           */
{
	if(reset || myNodeID == 0)
  {
  		ZW_DEBUG_SEND_BYTE(' ');
        ZW_DEBUG_SEND_BYTE('E');
        ZW_DEBUG_SEND_BYTE('N');
        ZW_DEBUG_SEND_BYTE('D');
        /* We have reached the end of the line. Apparently there are no */
        /* nodes available at all and the recovery has failed. */
        /* Reset everything. */
        searchNodeID = 1;
        HealComplete(FALSE);
        currentHealMode = HEAL_NONE;
        currentState = STATE_APPL_IDLE;
        nextState = STATE_APPL_IDLE;


////		networkUpdateDownCount = GetRandomNetUpdateCount();
	} else {
	  /* Update network rediscovery count down time */
	  networkUpdateDownCount = networkUpdateDownCount - 1;
	  ZW_DEBUG_SEND_BYTE('T');
	  ZW_DEBUG_SEND_BYTE('n');
	  ZW_DEBUG_SEND_NUM(networkUpdateDownCount);

	  if (networkUpdateDownCount <= 0)
	  {
	    ZW_DEBUG_SEND_BYTE('T');
	    ZW_DEBUG_SEND_BYTE('k');

	    networkUpdateDownCount = GetRandomNetUpdateCount();
	    ZW_REQUEST_NETWORK_UPDATE(UpdateNetworkUpdateCountCallback);
	  }
	}

}

/*============================   VerifyLostCount   ======================
**    Function description
**    Start Self Healing
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
BOOL                   /*RET  BOOL       */
VerifyLostCount( void )
{
    /* We are already doing something about it */
    if(currentHealMode != HEAL_NONE)
    {
      return TRUE;
    }
    ZW_DEBUG_SEND_BYTE('V');
    ZW_DEBUG_SEND_BYTE('l');
    ZW_DEBUG_SEND_BYTE(' ');
    ZW_DEBUG_SEND_NUM(lostCount);
    ZW_DEBUG_SEND_BYTE(' ');

    /* Check if the sensor has exceeded the "Lost Counter" limit and take action */
    if (lostCount >= DEFAULT_LOST_COUNTER_MAX)
    {
//		currentState = STATE_HEAL_LOST;
//		nextState = STATE_APPL_IDLE;
      ZW_DEBUG_SEND_BYTE('H');
      ZW_DEBUG_SEND_BYTE('7');
      ZW_DEBUG_SEND_BYTE('S');
      ZW_DEBUG_SEND_BYTE('s');
      ZW_DEBUG_SEND_NUM(currentState);
      ZW_DEBUG_SEND_BYTE('n');
      ZW_DEBUG_SEND_NUM(nextState);

      /* At least the last DEFAULT_LOST_COUNTER_MAX wake up notifications has failed */
      /* Try to recover from this situation */
#ifdef BATTERY
      StopPowerDownTimer();
#endif
      /* The node is lost - start healing */
      /* First, if there is a SUC in the network, try to contact it. */
      if (ZW_GetSUCNodeID() != 0)
      {
        ZW_DEBUG_SEND_BYTE('H');
        ZW_DEBUG_SEND_BYTE('8');
        currentHealMode = HEAL_SUC;
        AskNodeForHelp(ZW_GetSUCNodeID());
      }
//      else /* TO#1950 - Dont do lost if no SUC */
//      {
//        ZW_DEBUG_SEND_BYTE('H');
//        ZW_DEBUG_SEND_BYTE('1');
//        ZW_DEBUG_SEND_BYTE('0');
//        currentHealMode = HEAL_NONE;
//        HealComplete(FALSE);
//      }
      return TRUE;
    }
    else
    {
        return FALSE;
    }
}




/*============================   HealComplete   ======================
**    Function description
**    Self healing process complete.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
HealComplete( BOOL success )
{
  ZW_DEBUG_SEND_BYTE('H');
  ZW_DEBUG_SEND_BYTE('6');
  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_BYTE((success == TRUE ? 's' : 'f'));
  ZW_DEBUG_SEND_BYTE(' ');
  LED_OFF(3);
  UpdateLostCounter((success == TRUE ? ZW_ROUTE_UPDATE_DONE : ZW_ROUTE_LOST_FAILED));

  // We found the SUC - ask for updates
  if(success)
  {
    ZW_REQUEST_NETWORK_UPDATE(UpdateNetworkUpdateCountCallback);
  }
#ifdef BATTERY
  SetSleepPeriod();
#endif

  currentState = STATE_APPL_IDLE;

#ifdef BATTERY
  StartPowerDownTimer();
#endif
}


void                   /*RET  Nothing       */
CancelRediscoveryTimer()
{
  if(rediscoveryTimerHandle != 0xFF)
  {
    ZW_TIMER_CANCEL(rediscoveryTimerHandle);
  }
  ZW_DEBUG_SEND_BYTE('H');
  ZW_DEBUG_SEND_BYTE('7');
}
