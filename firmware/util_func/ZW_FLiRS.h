/******************************* ZW_FLiRS.h *******************************
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
 * Last Changed:     $Date: 2009-03-24 16:50:07 +0200 (Пт, 23 Лют 2007) $
 *
 ****************************************************************************/
#ifndef _ZW_FLIRS_H_
#define _ZW_FLIRS_H_

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/
#define ZW_BEAM_RX_WAKEUP

#ifdef ZW020x
#define FLIRS_DEVICE_OPTIONS_MASK_MODIFY(deviceOptionsMask) \
  deviceOptionsMask = (deviceOptionsMask & ~(APPLICATION_NODEINFO_LISTENING)) \
    | APPLICATION_NODEINFO_NOT_LISTENING | APPLICATION_FREQ_LISTENING_MODE_1000ms
#else
#define FLIRS_DEVICE_OPTIONS_MASK_MODIFY(deviceOptionsMask) \
  deviceOptionsMask = (deviceOptionsMask & ~(APPLICATION_NODEINFO_LISTENING)) \
    | APPLICATION_NODEINFO_NOT_LISTENING | APPLICATION_FREQ_LISTENING_MODE_1000ms
#endif


/****************************************************************************/
/*                              EXTERNAL DEFINED FUNCTIONS/DATA             */
/****************************************************************************/
#define FL_FLIRS_WAKEUP     0x01
#define FL_FLIRS_POWERLEVEL 0x02

void FLiRS_SetState(BYTE reason);
void FLiRS_ResetState(BYTE reason);

void FLiRS_SleepTimeoutStart(BOOL bLong);
void FLiRS_SleepTimeoutStop();
void FLiRS_SleepTimeoutReStart(BOOL bLong);

#endif /*_ZW_FLIRS_H_*/
