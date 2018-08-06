/******************************* learnmode.h *******************************
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
#ifndef _SLAVE_LEARN_H_
#define _SLAVE_LEARN_H_
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
BYTE nodeID);                 /*IN The nodeID assigned*/

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/
extern BOOL learnState;        /*Application can use this flag to check if learn
                                  mode is active*/

/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/

/*============================   StartLearnModeNow   ======================
**    Function description
**      Call this function whenever learnmode should be entered.
**      This function does the following:
**        - Set the Slave in Learnmode
**        - Starts a one second timeout after which learn mode is disabled
**        - Broadcast the NODEINFORMATION frame once each time unless the slave is
**        doing neighbor discovery.
**        - learnState will be TRUE until learnmode is done.
**      If the slave is added or removed to/from a network the function
**      LearnCompleted will be called.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void StartLearnModeNow(BYTE bMode);

#endif /*_SLAVE_LEARN_H_*/
