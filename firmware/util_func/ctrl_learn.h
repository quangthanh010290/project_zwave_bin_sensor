/****************************** ctrl_lear.h *******************************
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
 *              on ZW0201 standard controllers.
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
#ifndef _CTRL_LEARN_H_
#define _CTRL_LEARN_H_
/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/

/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/

/*============================   LearnCompleted   ===========================
**    Function description
**      Should be implemented by the Application.
**      Called when nodeID have been assigned or deleted.
**    Side effects:
**
**--------------------------------------------------------------------------*/
extern void LearnCompleted(
  BYTE nodeID,                  /*IN The nodeID assigned*/
  BYTE bStatus);                /*IN The status of the learn TRUE = SUCCESS, FALSE = FAILURE */

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/
/* Application can use this flag to check if learn mode is active*/
extern BOOL learnInProgress;

/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/

/*============================   StartLearnModeNow   ======================
**    Function description
**      Call this function whenever learnmode should be entered.
**      This function does the following:
**        - Set the controller in Learnmode
**        - Starts a one second timeout after which learn mode is disabled
**        - learnState will be TRUE until learnmode is done.
**      If the Controller is added or removed to/from a network the function
**      LearnCompleted will be called.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void StartLearnModeNow(BYTE bMode);

/*============================   StopLearnModeNow   ======================
**    Function description
**      Call this function from the application whenever learnmode
**      should be disabled.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
BYTE StopLearnModeNow(void);

/*==========================   ReArmLearnModeTimeout   =======================
**    Function description
**      Rearms the LearnMode timout handler and thereby extending the time
**      that the controller are to be in LearnMode/Receive.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
ReArmLearnModeTimeout(void);

#endif /*_CTRL_LEARN_H_*/
