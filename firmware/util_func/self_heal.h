/******************************* self_heal.h *******************************
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
 * Revision:         $Revision: 21351 $
 * Last Changed:     $Date: 2011-09-23 10:57:12 +0200 (fr, 23 sep 2011) $
 *
 ****************************************************************************/
#ifndef _SELF_HEAL_H_
#define _SELF_HEAL_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/
/* Recovery states */
#define HEAL_NONE                 0x00  /* No recovery operation in progress */
#define HEAL_SUC                  0x01  /* Next timeout - try talking to SUC if available */
#define HEAL_WAKEUPNODE           0x02  /* Next timeout - try talking to wakeupnode */
#define HEAL_GENERAL              0x03  /* Next timeout - try talking to SearchNodeID */

/* DEFAULT_LOST_COUNTER_MAX is used to determine when the node */
/* has lost the ability to communicate with it's wakeup */
/* notification node. The value specifies how many failed */
/* attempts is allowed, before the node starts yelling for help . */
#define DEFAULT_LOST_COUNTER_MAX      3

#define REDISCOVERY_TIMEOUT 100 /* 100 x 10ms */


/* Network Update timings */
/* Default 30 minutes between "Network Update Request"s */
#define DEFAULT_NETWORK_UPDATE_COUNT 7 // 30 //IZ:Fix TO# 02957 
/* Minimum 30 minutes between "Network Update Request"s */
#define NETWORK_UPDATE_MIN_COUNT  DEFAULT_NETWORK_UPDATE_COUNT
/* Maximum 180 minutes between "Network Update Request"s */
#define NETWORK_UPDATE_MAX_COUNT  19 // 180 //IZ:Fix TO# 02957 


/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

extern BYTE currentHealMode;
extern BYTE networkUpdateTimerHandle;

/* Data that must be maintained after powerdown */
extern XBYTE networkUpdateDownCount ;
extern XBYTE networkUpdateFailureCount;
#if defined (ZW020x) || defined(ZW030x)
extern XBYTE lostCount;
#endif
#ifdef ZW010x
extern IBYTE lostCount;
#endif
/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/




/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/
void SetDefaultNetworkUpdateConfiguration( void );
void VerifyAssociatedTransmit(BYTE txStatus, BYTE attemptedNodeId);
BOOL VerifyLostCount( void );
void UpdateNetworkUpdateCount( BOOL reset );
void UpdateNetworkUpdateCountOneMinute( void );
void HealComplete( BOOL success );
void AskNodeForHelp(BYTE NodeID);
void UpdateLostCounter(BYTE txStatus);
BYTE ZW_GetSUCNodeID( void );
void CancelRediscoveryTimer( void );


#endif /*_SELF_HEAL_H_*/
