/******************************* ZW_TransportSecurity.h *******************************
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
 *               secure Z-Wave Network
 *
 * Author:   Valeriy Vyshnyak
 *
 * Last Changed By:  $Author: vvi $
 * Revision:         $Revision: 13417 $
 * Last Changed:     $Date: 2009-03-10 16:17:52 +0200 (Вв, 10 Бер 2009) $
 *
 ****************************************************************************/
#ifndef _TRANSPORT_SECURITY_H_
#define _TRANSPORT_SECURITY_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <ZW_Security_AES_module.h>

#define TRANSPORT_EEPROM_SETTINGS_SIZE  (NETWORK_KEY_LENGTH + 5)

/****************************************************************************/
/*                              IMPORTED DATA                               */
/****************************************************************************/
extern ZW_APPLICATION_TX_BUFFER txBuf;


/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/
typedef void (CODE *VOID_CALLBACKFUNC_BYTE)(BYTE txStatus);     /* callback function, which called by the transport layer when data is transmitted */

typedef void (CODE *TRANSPORT_STATUS_CALLBACK)(BYTE transportStatus); /* callback function, which called by transport to inform application layer of its status */
/* Values of transportStatus argument of Transport status callback function: */
#define TRANSPORT_WORK_START    0x01  /* Transport layer begin the longly work operation. This status can be used to stop the power down timeout timer in battery operated devices */
#define TRANSPORT_WORK_END      0x02  /* Transport layer successfully end the longly work operation. This status can be used to start the power down timeout timer in battery operated devices */
#define TRANSPORT_WORK_ERROR    0x03  /* Transport layer abort the longly work operation. This status can be used to start the power down timeout timer in battery operated devices */

#define SECURITY_SEND_DATA_TIME_OUT   20     

/****************************************************************************/
/*                           IMPORTED FUNCTIONS                             */
/****************************************************************************/
/*===========   Transport_ApplicationCommandHandler   ======================
**      Called, when frame is received
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern void
Transport_ApplicationCommandHandler(
  BYTE  rxStatus,                 /* IN Frame header info */
  BYTE  sourceNode,               /* IN Command sender Node ID */
  ZW_APPLICATION_TX_BUFFER *pCmd, /* IN Payload from the received frame, the union */
                                  /*    should be used to access the fields */
  BYTE   cmdLength);              /* IN Number of command bytes including the command */


/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/
/*========================   Transport_SendRequest   ============================
**      Send request command over secure network
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern BYTE
Transport_SendRequest(
  BYTE nodeID,
  BYTE *pBufData,
  BYTE dataLength,
  BYTE txOptions,
  VOID_CALLBACKFUNC_BYTE completedFunc,
  BYTE isForceNative);                    /* TRUE if data should be sent native, i.e. without using this transport.*/

/*========================   Transport_SendReport   ============================
**      This function must be called instead of Transport_SendRequest, if report
**      frame is sent.
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern BYTE
Transport_SendReport(
  BYTE nodeID,
  BYTE *pBufData,
  BYTE dataLength,
  BYTE txOptions,
  VOID_CALLBACKFUNC_BYTE completedFunc,
  BYTE isForceNative);                     /* TRUE if data should be sent native, i.e. without using this transport.*/

/*==============   Transport_OnApplicationInitHW   ============================
**      This function must be called in ApplicationInitHW
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern BYTE                               /* return TRUE on success */
Transport_OnApplicationInitHW(
  BYTE bStatus);                          /* bStatus */

/*==============   Transport_OnApplicationInitSW   ============================
**      This function must be called in ApplicationInitSW
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern BYTE                               /* return TRUE on success */
Transport_OnApplicationInitSW(
  BYTE *commandClassesList,               /* List of supported command classes, when node communicate by this transport */
  BYTE commandClassesListCount,           /* Count of elements in supported command classes list */
  BYTE eepromInit,                        /* TRUE if contents of eeprom must forced to be initialized to default values */
  BYTE eepromBufOffsSettings,             /* buffer offset in eeprom for storing transport settings*/
  BYTE eepromBufSizeSettings,             /* buffer size in eeprom for storing transport settings*/
  TRANSPORT_STATUS_CALLBACK statusCallbackFunc);  /* callback function, which called by transport to inform application layer of its status. can be NULL, if status not needed. */

/*==============   Transport_OnLearnCompleted   ============================
**      This function must be called in LearnCompleted application function
**       callback
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern BYTE                                /* return TRUE on success */
Transport_OnLearnCompleted(
  BYTE nodeID)  ;                          /* IN resulting nodeID */


/*==============   Transport_SetNodeSecure   ============================
**      This function must be called if changed nodeSecure
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern void                                      /* return TRUE on success */
Transport_SetNodeSecure(
  BYTE vNodeSecure);

extern BYTE
GetNodeSecure(BYTE eepromBufOffsSettings);

extern BYTE
Transport_SendDataSecure(
  BYTE tnodeID,
  BYTE *pBufData,
  BYTE dataLength,
  BYTE txSecOptions,
  VOID_CALLBACKFUNC_BYTE completedFunc);

extern void                      /* RET nothing */
TxSecureDoneInNonceReport(
  BYTE bStatus);                 /* IN  status */

void
StartSecuritySendDataTimer(BYTE timeOut);



#endif /*_TRANSPORT_SECURITY_H_*/
