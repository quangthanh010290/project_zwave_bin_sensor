/******************************* ZW_TransportNative.h *******************************
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
 * Description: Implements functions for transporting frames over the
 *               native Z-Wave Network.
 *
 * Author:   Valeriy Vyshnyak
 *
 * Last Changed By:  $Author: vvi $
 * Revision:         $Revision: 13417 $
 * Last Changed:     $Date: 2009-03-10 16:17:52 +0200 (Вв, 10 Бер 2009) $
 *
 ****************************************************************************/
#ifndef _TRANSPORT_NATIVE_H_
#define _TRANSPORT_NATIVE_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/


/****************************************************************************/
/*                              IMPORTED DATA                               */
/****************************************************************************/


/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/
#define TRANSPORT_EEPROM_SETTINGS_SIZE  0


/****************************************************************************/
/*                           IMPORTED FUNCTIONS                             */
/****************************************************************************/
/*===========   Transport_ApplicationCommandHandler   ======================
**      Called, when frame is received
**    Side effects :
**
**--------------------------------------------------------------------------*/

#define Transport_ApplicationCommandHandler ApplicationCommandHandler
#define Transport_ApplicationCommandHandler_Wrapped ApplicationCommandHandler_Wrapped


/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/
/*========================   Transport_SendRequest   ============================
**      Send request command over secure network
**    Side effects :
**
**--------------------------------------------------------------------------*/

#define Transport_SendRequest(tnodeID, pBufData, dataLength, txOptions, completedFunc, isForceNative) \
  ZW_SEND_DATA(tnodeID, pBufData, dataLength, txOptions, completedFunc)

/*========================   Transport_SendReport   ============================
**      This function must be called instead of Transport_SendRequest, if report
**      frame is sent.
**    Side effects :
**
**--------------------------------------------------------------------------*/

#define Transport_SendReport(tnodeID, pBufData, dataLength, txOptions, completedFunc, isForceNative) \
  ZW_SEND_DATA(tnodeID, pBufData, dataLength, txOptions, completedFunc)

/*==============   Transport_OnApplicationInitHW   ============================
**      This function must be called in ApplicationInitHW
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
                                          /* return TRUE on success */
#define Transport_OnApplicationInitHW(bStatus)

/*==============   Transport_OnApplicationInitSW   ============================
**      This function must be called in ApplicationInitSW
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
                                          /* return TRUE on success */
#define Transport_OnApplicationInitSW(commandClassesList, commandClassesListCount, eepromInit, eepromBufOffsSettings, eepromBufSizeSettings,FuncZWSecure)

/*==============   Transport_OnLearnCompleted   ============================
**      This function must be called in LearnCompleted application function
**       callback
**    Side effects :
**
**--------------------------------------------------------------------------*/
                                           /* return TRUE on success */
#define Transport_OnLearnCompleted(nodeID)





#endif /*_TRANSPORT_NATIVE_H_*/
