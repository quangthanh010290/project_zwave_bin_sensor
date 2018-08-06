/************************** ZW_TransportSecurity.c ***************************
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

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#ifdef ZW_CONTROLLER
#include <ZW_controller_api.h>
#else
#include <ZW_slave_api.h>
#endif
#include <ZW_Security_AES_module.h>
#include <ZW_TransportSecurity.h>
#include <ZW_TransportLayer.h>
#ifdef FLIRS
#include <ZW_FLiRS.h>
#endif
#include <ZW_debug_api.h>
#include <ZW_uart_api.h>
//#include "Stubs.h"

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/
#define EEOFFS_NETWORK_SECURITY             0
#define EEOFFS_NETWORK_SECURITY_SIZE        1
#define EEOFFS_NETWORK_KEY_START            EEOFFS_NETWORK_SECURITY + EEOFFS_NETWORK_SECURITY_SIZE
#define EEOFFS_NETWORK_KEY_SIZE             NETWORK_KEY_LENGTH

//new eeprom variables add only before this magic byte variable (and don't forget to change offset of magic byte!!!)
#define EEOFFS_MAGIC_BYTE                   EEOFFS_NETWORK_KEY_START + EEOFFS_NETWORK_KEY_SIZE
#define EEOFFS_MAGIC_BYTE_SIZE              1
#define EEPROM_MAGIC_BYTE_VALUE             0x56

#define EEOFFS_NETWORK_SECURITY_STATUS      EEOFFS_MAGIC_BYTE + EEOFFS_MAGIC_BYTE_SIZE
#define EEOFFS_NETWORK_SECURITY_STATUS_SIZE 1

//TO# 03444 TO# 03445
#define SECURITY_INCLUSION_TIMER_STOP 0xFF
#define SECURITY_INCLUSION_TIME_OUT   10     
#define SECURITY_SEND_DATA_TIMER_STOP 0xFF
/****************************************************************************/
/*                              PRIVATE DATA                                */
/****************************************************************************/

//TO# 03444 TO# 03445
static BYTE securityInclusionTimeOut = SECURITY_INCLUSION_TIMER_STOP;
static BYTE securityInclusionTimerHandle = 0xFF;
static BYTE securitySendDataTimeOut = SECURITY_SEND_DATA_TIMER_STOP;
static BYTE securitySendDataTimerHandle = 0xFF;

BYTE nodeSecure;
BYTE securityLife;
BYTE securityTimerHandle = 0;
BYTE nodeSecureStatus = 0;
BYTE notSleep = 0;

BYTE Transport_wakeUpReason;
BYTE code tmpkey[NETWORK_KEY_LENGTH] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
BYTE ltmpkey[NETWORK_KEY_LENGTH];

BYTE eepromOffsSettings = -1;           /* buffer offset in eeprom for storing the settings */
BYTE *secureCommandClassesList = NULL;  /* List of supported command classes, when node communicate by this transport */
BYTE secureCommandClassesListCount = 0; /* Count of elements in supported command classes list */

#define TSEC_CMD_HANDLING_DEFAULT   0
#define TSEC_CMD_HANDLING_SECURE    1
#define TSEC_CMD_HANDLING_UNSECURE  2
BYTE secureCmdHandling = TSEC_CMD_HANDLING_DEFAULT;  /* type of calling of ApplicationCommandHandler*/

void ( *cbFuncZWSecure ) ( BYTE txStatus ) = NULL;

static BOOL isInclusionMode = FALSE;

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/
extern BYTE enNodeID;      /* Associated host id */
extern BYTE enNonce[8];    /* External nonce */

extern BYTE nrDataLength;                      /* Length of the payload data */
extern BYTE nrNodeID;                          /* Destignation node ID */
extern BYTE nrTXSecOptions;                    /* Security Options */
extern void (*nrCompletedFunc)(BYTE txStatus); /* Completed callback function */

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/

/*==============================   cbVoidByte   ============================
**
**  Function:  stub for callback 
**
**  Side effects: None
**
**--------------------------------------------------------------------------*/
static void cbVoidByte(BYTE b)
{
}


/*==============================   TxSecureDone   ============================
**
**  Function:  Transmit of a frame done, start the powerdown timer
**
**  Side effects: None
**
**--------------------------------------------------------------------------*/

void TxSecureDone(BYTE bStatus)
{
  isInclusionMode = FALSE;
  if (cbFuncZWSecure != NULL)
  {
    cbFuncZWSecure(TRANSPORT_WORK_END);
  }
}

/*===========================================================================
**    Function description
**      Called, when encapsulated message are send after the Nonce report received.
**    Side effects:
**
**--------------------------------------------------------------------------*/
#if 0  //TO3920
void                      /* RET nothing */
TxSecureDoneInNonceReport(
  BYTE bStatus)           /* IN  status */
{
  if (cbFuncZWSecure != NULL)
  {
    cbFuncZWSecure(TRANSPORT_WORK_END);
  }
  if (nrCompletedFunc != NULL)
  {
    nrCompletedFunc(bStatus);
  }
}
#endif

/*========================   Transport_SendSecure   ============================
**      Send data over secure network
**    Side effects :
**
**--------------------------------------------------------------------------*/
BYTE
Transport_SendDataSecure(
  BYTE tnodeID,
  BYTE *pBufData,
  BYTE dataLength,
  BYTE txSecOptions,
  VOID_CALLBACKFUNC_BYTE completedFunc)
{
  return ZW_SendDataSecure(tnodeID, pBufData, dataLength, txSecOptions, completedFunc);
}

/*=============================   SaveSecureNetworkKey   =====================
**    SaveSecureNetworkKey
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
void
SaveSecureNetworkKey(void)
{
  if (eepromOffsSettings != -1)
  {
    ZW_MEM_PUT_BUFFER_NO_CB(eepromOffsSettings + EEOFFS_NETWORK_KEY_START, networkKey, NETWORK_KEY_LENGTH);
  }
}


BYTE IsNetworkIncludeKeyInclude()
{
  BYTE y = 0, i = 0;

  for (i = 0; i < NETWORK_KEY_LENGTH; i++)
  {
    y = y | networkKey[i];
  }
  return !y;
}

static void
SecurityInclusionTimer()
{
  if (securityInclusionTimeOut == 0)
  {
    if (securityInclusionTimerHandle != 0xff)
    {
      ZW_TIMER_CANCEL(securityInclusionTimerHandle);
      securityInclusionTimerHandle = 0xff;
    }
    nodeSecure = 0;
  }
  else if ((securityInclusionTimeOut != SECURITY_INCLUSION_TIMER_STOP)&&(securityInclusionTimeOut > 0))
  {
    securityInclusionTimeOut--;
  }
}

static void
StartSecurityInclusionTimer(BYTE timeOut)
{
  securityInclusionTimeOut = timeOut;
  if (securityInclusionTimerHandle != 0xff)
  {
    ZW_TIMER_CANCEL(securityInclusionTimerHandle);
  }
  securityInclusionTimerHandle = ZW_TIMER_START(SecurityInclusionTimer, TIMER_ONE_SECOND, TIMER_FOREVER);
}

static void
StopSecurityInclusionTimer()
{
  if (securityInclusionTimerHandle != 0xff)
  {
    ZW_TIMER_CANCEL(securityInclusionTimerHandle);
    securityInclusionTimerHandle = 0xff;
  }
  securityInclusionTimeOut = SECURITY_INCLUSION_TIMER_STOP;
}

static void
SecuritySendDataTimer()
{
  if (securitySendDataTimeOut == 0)
  {
    if (securitySendDataTimerHandle != 0xff)
    {
      ZW_TIMER_CANCEL(securitySendDataTimerHandle);
      securitySendDataTimerHandle = 0xff;
    }
  }
  else if (securitySendDataTimeOut != SECURITY_SEND_DATA_TIMER_STOP)
  {
    securitySendDataTimeOut--;
  }
}

void
StartSecuritySendDataTimer(BYTE timeOut)
{
  securitySendDataTimeOut = timeOut;
  if (securitySendDataTimerHandle != 0xff)
  {
    ZW_TIMER_CANCEL(securitySendDataTimerHandle);
  }
  securitySendDataTimerHandle = ZW_TIMER_START(SecuritySendDataTimer, TIMER_ONE_SECOND, TIMER_FOREVER);
}

static void
StopSecuritySendDataTimer()
{
  if (securitySendDataTimerHandle != 0xff)
  {
    ZW_TIMER_CANCEL(securitySendDataTimerHandle);
    securitySendDataTimerHandle = 0xff;
  }
  securitySendDataTimeOut = SECURITY_SEND_DATA_TIMER_STOP;
}


/*========================   ApplicationCommandHandler   ====================
**    Handling of a received application commands and requests
**
*
**--------------------------------------------------------------------------*/
void                              /*RET Nothing                  */
ApplicationCommandHandler(
  BYTE  rxStatus,                 /* IN Frame header info */
  BYTE  sourceNode,               /* IN Command sender Node ID */
  ZW_APPLICATION_TX_BUFFER *pCmd, /* IN Payload from the received frame, the union */
  /*    should be used to access the fields */
  BYTE   cmdLength)               /* IN Number of command bytes including the command */
{
  int i;
  BYTE cbFuncCall;
  BYTE txOption;

  cbFuncCall = FALSE;
  txOption = ((rxStatus & RECEIVE_STATUS_LOW_POWER) ? TRANSMIT_OPTION_LOW_POWER : 0)
             | TRANSMIT_OPTION_ACK | TRANSMIT_OPTION_RETURN_ROUTE | TRANSMIT_OPTION_EXPLORE;

  if (pCmd->ZW_Common.cmdClass == COMMAND_CLASS_SECURITY)
  {
    if ( cbFuncZWSecure != NULL )
    {
      cbFuncZWSecure(TRANSPORT_WORK_START);
    }

    ZW_DEBUG_SEND_BYTE('U');
    ZW_DEBUG_SEND_BYTE('U');
    ZW_DEBUG_SEND_NUM(nodeSecure);

    if (nodeSecure != 0)
    {
      if (pCmd->ZW_Common.cmd != SECURITY_NONCE_REPORT)
      {
        /* Do not call the status callback for nonce report because nonce report is a part of the sending data
         *  procedure, which was initiated by application layer. Therefore application layer already know
         *  when sending are started and when sending will be finished. (Therefore, for exaple in FLiRS,
         *  application layer already stop the power down timer at start (before call of Transport_SendReport)
         *  of data sending and will start this powerdown timer in callback function of Transport_SendReport
         *  when sendig will be complited. I.e, there is no need to call the status callback of security layer
         *  to inform application layer that it should stop the power down timer again.)
         */
        if (cbFuncZWSecure != NULL)
        {
          cbFuncZWSecure(TRANSPORT_WORK_START);
        }
        cbFuncCall = TRUE;    /* at the end of this ApplicationCommandHandler we must call status callback with "work end" status. */
      }

      if (pCmd->ZW_Common.cmd == SECURITY_SCHEME_GET)
      {
        /* Security command class commands which are not encapsulated are handled here */
        nodeSecureStatus = 1;
        isInclusionMode = TRUE;
        StartSecurityInclusionTimer(SECURITY_INCLUSION_TIME_OUT);

        if (pCmd->ZW_SecuritySchemeGetFrame.supportedSecuritySchemes == SECURITY_SCHEME_0)
        {
          memset(networkKey, 0x00, NETWORK_KEY_LENGTH);
          SaveSecureNetworkKey();
          LoadKeys();
        }
        else
        {
          Transport_SetNodeSecure(NON_SECURE_NODE);
          if (eepromOffsSettings != -1)
          {
            ZW_MEM_PUT_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY, 0x0);
          }
        }

        txBuf.ZW_SecuritySchemeReportFrame.cmdClass = COMMAND_CLASS_SECURITY; /* The command class */
        txBuf.ZW_SecuritySchemeReportFrame.cmd = SECURITY_SCHEME_REPORT; /* The command */
        txBuf.ZW_SecuritySchemeReportFrame.supportedSecuritySchemes = SECURITY_SCHEME;

        if (ZW_SEND_DATA(sourceNode, (BYTE *)&txBuf, sizeof(txBuf.ZW_SecuritySchemeReportFrame), txOption | TRANSMIT_OPTION_EXPLORE, cbVoidByte))
        {
          StartSecurityTimeOut(TIME_SECURITY_LIFE);
          cbFuncCall = FALSE;   /* do not call status callback in this handler because work in progress and it will be called by the timeout or when security layer are finish work */
        }
        else
        { /* we can't sent the report - deinit the security: */
          nodeSecure = 0;
        }
      }
      else if (securityInclusionTimeOut == 0)
      {
        if (cbFuncCall && cbFuncZWSecure != NULL)
        {
          cbFuncZWSecure(TRANSPORT_WORK_END);
        }
        return;
      }

      if (pCmd->ZW_Common.cmd == SECURITY_NONCE_REPORT)
      {
        if (securityInclusionTimeOut != 0)
        {
          StopSecurityInclusionTimer();
        }

        if (securitySendDataTimeOut == 0)
        {
          if (cbFuncCall && cbFuncZWSecure != NULL)
          {
            cbFuncZWSecure(TRANSPORT_WORK_END);
          }
          return;
        }
        else if (securitySendDataTimeOut != SECURITY_SEND_DATA_TIMER_STOP)
        {
          StopSecuritySendDataTimer();
        }

        enNodeID = sourceNode;
        enNonce[0] = *((BYTE*)pCmd + 2);
        enNonce[1] = *((BYTE*)pCmd + 3);
        enNonce[2] = *((BYTE*)pCmd + 4);
        enNonce[3] = *((BYTE*)pCmd + 5);
        enNonce[4] = *((BYTE*)pCmd + 6);
        enNonce[5] = *((BYTE*)pCmd + 7);
        enNonce[6] = *((BYTE*)pCmd + 8);
        enNonce[7] = *((BYTE*)pCmd + 9);
        // TO# 03329
        // When a node sends a routed nonce_get frame.
        // Then there are possiblity that we received a nonce_report
        // while we are still waiting for the nonce_get callback.
        ZW_DEBUG_SEND_BYTE('.');
        ZW_DEBUG_SEND_BYTE('1');
        ZW_DEBUG_SEND_BYTE('.');
        ZW_DEBUG_SEND_NUM(sourceNode);
        ZW_DEBUG_SEND_BYTE('.');
        ZW_DEBUG_SEND_BYTE('.');

        if (SecurityNonceGetInProgress == FALSE)
        {
          // nonse_get callback is allready finished at this moment
          SecurityDelayedNonceReport = FALSE;
          Transport_SendDataSecure(sourceNode, (BYTE *)&txBuf, nrDataLength, TRANSMIT_OPTION_EXPLORE, cbFuncZWSecure);
        }
        else
        {
          // nonse_get callback is in waiting state,
          // we will invoke Transport_SendDataSecure later
          SecurityDelayedNonceReport = TRUE;
          ZW_DEBUG_SEND_BYTE('9');
        }
      }
      else
      {
        /* All other Security command class commands are handled in ProcessIncommingSecure !!!*/
        /* Which also decrypts the encapsulated message if any present and its proper authenticated */
        ((BYTE*)pCmd) += 1;       /* 1 - this is a size of command class value, removed, because ProcessIncomingSecure need Security Header byte as first byte. */
        cmdLength -= 1;           /* length is corrected accordingly to the above changes of pointer */

        if (nodeSecure)
        {
          nodeSecureStatus = 1;
          cmdLength = ProcessIncomingSecure(sourceNode, pCmd, txOption | TRANSMIT_OPTION_EXPLORE, cmdLength);
          ZW_DEBUG_SEND_NUM(cmdLength);
        }
        else
        {
          cmdLength = 0xff;
        }

        if (cmdLength != 0 && cmdLength != 0xff)
        {
          ((BYTE*)pCmd) += 9 + 1; /* 9 - is a offset of decrypted data, decrypted by ProcessIncomingSecure. */
          /* 1 - this is a size of service byte in message encapsulation frame with fields: Reserved, Second Frame, Sequenced, Sequence Counter. */
          cmdLength -= 1;         /* 1 - this is a size of service byte described above. */

          if (cmdLength >= 2)
          {
            switch (pCmd->ZW_Common.cmdClass)
            {
              case COMMAND_CLASS_SECURITY:
              {
                if ( cbFuncZWSecure != NULL )  cbFuncZWSecure(TRANSPORT_WORK_START);

                switch (pCmd->ZW_Common.cmd)
                {
                  case SECURITY_COMMANDS_SUPPORTED_GET:
                    enNodeID = 0xff;
                    txBuf.ZW_SecurityCommandsSupportedReport1byteFrame.cmdClass = COMMAND_CLASS_SECURITY;
                    txBuf.ZW_SecurityCommandsSupportedReport1byteFrame.cmd = SECURITY_COMMANDS_SUPPORTED_REPORT;
                    txBuf.ZW_SecurityCommandsSupportedReport1byteFrame.reportsToFollow = 0;

                    if (secureCommandClassesList != NULL)
                    {
                      memcpy(&txBuf.ZW_SecurityCommandsSupportedReport1byteFrame.commandClassSupport1,
                             secureCommandClassesList, secureCommandClassesListCount);
                    }
                    (&txBuf.ZW_SecurityCommandsSupportedReport1byteFrame.commandClassSupport1)[secureCommandClassesListCount] = COMMAND_CLASS_MARK;

                    if (Transport_SendDataSecure(sourceNode, (BYTE *)&txBuf,
                                                 sizeof(txBuf.ZW_SecurityCommandsSupportedReport1byteFrame) - 2 + secureCommandClassesListCount,
                                                 txOption | TRANSMIT_OPTION_EXPLORE, TxSecureDone))
                    {
                      cbFuncCall = FALSE;    /* do not call status callback in this handler because it will be called when report will be sent */
                    }
                    break;

                  case SECURITY_COMMANDS_SUPPORTED_REPORT:
                    /* */
                    /* Drop through */
                    break;

                  case NETWORK_KEY_SET:
                    if (!IsNetworkIncludeKeyInclude()) return;

                    nodeSecureStatus = 1;
                    if (securityTimerHandle)
                    {
                      ZW_TIMER_CANCEL(securityTimerHandle);
                    }
                    Transport_SetNodeSecure(SECURE_NODE);
                    if (eepromOffsSettings != -1)
                    {
                      ZW_MEM_PUT_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY, 0x81);
                    }

                    if (SECURITY_SCHEME == SECURITY_SCHEME_0)
                    {
                      memset(tmpkey, 0x00, NETWORK_KEY_LENGTH);
                    }

                    memcpy(networkKey, &pCmd->ZW_NetworkKeySet1byteFrame.networkKeyByte1, NETWORK_KEY_LENGTH);
                    /* Now network Key should contain the new networkKey */
                    /* Save the new networkKey in NV RAM */
                    SaveSecureNetworkKey();
                    /* Go and make the Authentication and the encryption keys */
                    LoadKeys();
                    ZW_MEM_PUT_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY_STATUS, nodeSecureStatus);
                    enNodeID = 0xff;
                    txBuf.ZW_NetworkKeyVerifyFrame.cmdClass = COMMAND_CLASS_SECURITY;
                    txBuf.ZW_NetworkKeyVerifyFrame.cmd = NETWORK_KEY_VERIFY;

                    if (Transport_SendDataSecure(sourceNode, (BYTE *)&txBuf, sizeof(txBuf.ZW_NetworkKeyVerifyFrame), txOption | TRANSMIT_OPTION_EXPLORE, TxSecureDone))
                    {
                      cbFuncCall = FALSE;    /* do not call status callback in this handler because it will be called when report will be sent */
                    }
                    break;
                }
                break;
              }

              default:
              {
                /* All other commands, decrypted from the message encapsulation - pass to the application layer: */

                /* call status callback with "work end" flag here because security are finished their job: */
                if (cbFuncZWSecure != NULL)
                {
                  cbFuncZWSecure(TRANSPORT_WORK_END);
                }

                cbFuncCall = FALSE;

                /* call application layer handler: */
                secureCmdHandling = TSEC_CMD_HANDLING_SECURE;     //message is secured, so reports also must be secured.
                Transport_ApplicationCommandHandler(rxStatus, sourceNode, pCmd, cmdLength);
                secureCmdHandling = TSEC_CMD_HANDLING_DEFAULT;
                break;
              }
            }
          }
        }
      }
    }
  }
  else
  {
    /* All other non encrypted commands and non-security class commands - pass to the application layer: */
    secureCmdHandling = TSEC_CMD_HANDLING_UNSECURE;       //message is not secured, so reports also must be sent non-secure.
//    if(!nodeSecure)
    {
      Transport_ApplicationCommandHandler(rxStatus, sourceNode, pCmd, cmdLength);
    }
    secureCmdHandling = TSEC_CMD_HANDLING_DEFAULT;
  }

  if (cbFuncCall && cbFuncZWSecure != NULL)
  {
    cbFuncZWSecure(TRANSPORT_WORK_END);
  }
}



/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/

/*========================   Transport_SendRequest   ============================
**      Send request command over secure network
**    Side effects :
**
**--------------------------------------------------------------------------*/
BYTE
Transport_SendRequest(
  BYTE tnodeID,
  BYTE *pBufData,
  BYTE dataLength,
  BYTE txOptions,
  VOID_CALLBACKFUNC_BYTE completedFunc,
  BYTE isForceNative)                     /* TRUE if data should be sent native, i.e. without using this transport.*/
{
  enNodeID = 0xff;
  if (secureCmdHandling == TSEC_CMD_HANDLING_UNSECURE)
  {
    isForceNative = TRUE;
  }
  if (!nodeSecure || isForceNative)
  {
    return ZW_SEND_DATA(tnodeID, pBufData, dataLength, txOptions, completedFunc);
  }
  else
  {
    return Transport_SendDataSecure(tnodeID, pBufData, dataLength, txOptions, completedFunc);
  }
}

/*========================   Transport_SendReport   ============================
**      This function must be called instead of Transport_SendRequest, if report
**      frame is sent.
**    Side effects :
**
**--------------------------------------------------------------------------*/
BYTE
Transport_SendReport(
  BYTE tnodeID,
  BYTE *pBufData,
  BYTE dataLength,
  BYTE txOptions,
  VOID_CALLBACKFUNC_BYTE completedFunc,
  BYTE isForceNative)                     /* TRUE if data should be sent native, i.e. without using this transport.*/
{
  enNodeID = 0xff;
  if (secureCmdHandling == TSEC_CMD_HANDLING_UNSECURE)
  {
    isForceNative = TRUE;
  }

  if (!nodeSecure || isForceNative)
  {
    return ZW_SEND_DATA(tnodeID, pBufData, dataLength, txOptions, completedFunc);
  }
  else
  {
    return Transport_SendDataSecure(tnodeID, pBufData, dataLength, txOptions, completedFunc);
  }
}

/*==============   Transport_OnApplicationInitHW   ============================
**      This function must be called in ApplicationInitHW
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
BYTE                                      /* return TRUE on success */
Transport_OnApplicationInitHW(
  BYTE bStatus)                          /* bStatus */
{
  Transport_wakeUpReason = 0x0;//bStatus;
  return TRUE;
}


/*==============   Transport_SetNodeSecure   ============================
**      This function must be called if changed nodeSecure
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
void                                      /* return TRUE on success */
Transport_SetNodeSecure(
  BYTE vNodeSecure)
{
  if (isInclusionMode)
  {
    nodeSecure = vNodeSecure;
    ZW_MEM_PUT_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY, nodeSecure);
  }
}

/*==============   Transport_OnApplicationInitSW   ============================
**      This function must be called in ApplicationInitSW
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
BYTE                                      /* return TRUE on success */
Transport_OnApplicationInitSW(
  BYTE *commandClassesList,               /* List of supported command classes, when node communicate by this transport */
  BYTE commandClassesListCount,           /* Count of elements in supported command classes list */

  BYTE eepromInit,                        /* TRUE if contents of eeprom must forced to be initialized to default values */
  BYTE eepromBufOffsSettings,             /* buffer offset in eeprom for storing transport settings*/
  BYTE eepromBufSizeSettings,             /* buffer size in eeprom for storing transport settings*/
  TRANSPORT_STATUS_CALLBACK statusCallbackFunc) /* callback function, which called by transport to inform application layer of its status. can be NULL, if status not needed. */
{
  cbFuncZWSecure = statusCallbackFunc;

  //initialization:
  secureCmdHandling = TSEC_CMD_HANDLING_DEFAULT;

  // cmd classes list storing:
  secureCommandClassesList = commandClassesList;
  secureCommandClassesListCount = commandClassesListCount;
  if (secureCommandClassesList == NULL)
    secureCommandClassesListCount = 0;

  //eeprom handling:
  if (eepromOffsSettings != -1 && !eepromInit)
  {
    eepromInit = (ZW_MEM_GET_BYTE(eepromOffsSettings + EEOFFS_MAGIC_BYTE) != EEPROM_MAGIC_BYTE_VALUE);
  }
  eepromOffsSettings = -1;
  if (eepromBufSizeSettings < TRANSPORT_EEPROM_SETTINGS_SIZE)
  {
    return FALSE;
  }
  eepromOffsSettings = eepromBufOffsSettings;
  if (eepromInit)
  {
#ifdef ZW_CONTROLLER
    SECURITY_SCHEME = SECURITY;
    GetRNGData(networkKey, NETWORK_KEY_LENGTH);
#else
    memset(networkKey, 0, NETWORK_KEY_LENGTH);
#endif
    Transport_SetNodeSecure(SECURE_NODE);
    if (eepromOffsSettings != -1)
    {
#ifdef ZW_CONTROLLER
      ZW_MEM_PUT_BYTE(eepromOffsSettings + EEOFFS_SECURITY_SCHEME, SECURITY_SCHEME);
#endif
      ZW_MEM_PUT_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY, nodeSecure);
      ZW_MEM_PUT_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY_STATUS, nodeSecureStatus);
      ZW_MEM_PUT_BUFFER_NO_CB(eepromOffsSettings + EEOFFS_NETWORK_KEY_START, NULL, NETWORK_KEY_LENGTH);
      ZW_MEM_PUT_BYTE(eepromOffsSettings + EEOFFS_MAGIC_BYTE, EEPROM_MAGIC_BYTE_VALUE);
    }
  }
  else
  {
    if (eepromOffsSettings != -1)
    {
#ifdef ZW_CONTROLLER
      SECURITY_SCHEME = ZW_MEM_GET_BYTE(eepromOffsSettings + EEOFFS_SECURITY_SCHEME);
#endif
      nodeSecure = ZW_MEM_GET_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY);
      nodeSecureStatus = ZW_MEM_GET_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY_STATUS);
//    if (nodeSecure) nodeSecureStatus = 2;
      ZW_MEM_GET_BUFFER(eepromOffsSettings + EEOFFS_NETWORK_KEY_START, networkKey, NETWORK_KEY_LENGTH);
    }
  }

  //security init:
  InitSecurity(Transport_wakeUpReason);

  return TRUE;
}

BYTE
GetNodeSecure(BYTE eepromBufOffsSettings)
{
  eepromOffsSettings = eepromBufOffsSettings;
  nodeSecure = ZW_MEM_GET_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY);
  return nodeSecure;
}


/*==============   Transport_OnLearnCompleted   ============================
**      This function must be called in LearnCompleted application function
**       callback
**    Side effects :
**
**--------------------------------------------------------------------------*/
BYTE                                      /* return TRUE on success */
Transport_OnLearnCompleted(
  BYTE nodeID)                            /* IN resulting nodeID */
{
  if (nodeID == 0)    /* Was it reset? */
  {
    StopSecurityTimeOut();
    StopSecuritySendTimeOut();

    if (SECURITY_SCHEME == SECURITY_SCHEME_0)
    {
      memset(networkKey, 0x00, NETWORK_KEY_LENGTH);
    }
    SaveSecureNetworkKey();
    /* Go and make the Authentication and the encryption keys */
    LoadKeys();
    Transport_SetNodeSecure(SECURE_NODE);

    if (eepromOffsSettings != -1)
    {
      ZW_MEM_PUT_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY, 0x1);
    }
    nodeSecureStatus = 0;
    securityTimerHandle = 0;
    ZW_MEM_PUT_BYTE(eepromOffsSettings + EEOFFS_NETWORK_SECURITY_STATUS, nodeSecureStatus);
  }
  else
  {
    ZW_DEBUG_SEND_BYTE('m');
    ZW_DEBUG_SEND_NUM(nodeSecureStatus);

    if (nodeSecureStatus == 0)
    {
      if (!securityTimerHandle)
      {
        isInclusionMode = TRUE;
        StartSecurityTimeOut(TIME_SECURITY_LIFE);
      }
    }
    else
    {
      if (!securityTimerHandle)
      {
        if ( cbFuncZWSecure != NULL )  cbFuncZWSecure(TRANSPORT_WORK_END);
      }
    }
  }
  return TRUE;
}
