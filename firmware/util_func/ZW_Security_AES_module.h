/************************  ZW_Security_AES_module.H  ************************
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
 * Description:       Header file for Security on app level AES module
 *
 * Author:            Johann Sigfredsson, Oleg Zadorozhnyy
 *
 * Last Changed By:  $Author: oza $
 * Revision:         $Revision: 1.03 $
 * Last Changed:     $Date: 2008-08-29 12:58:10 +0200 (Пт, 25 Січ 2008) $
 *
 ****************************************************************************/
#ifndef _ZW_SECURITY_AES_MODULE_H_
#define _ZW_SECURITY_AES_MODULE_H_

#define NETWORK_KEY_LENGTH  16

/* Global Nonce variables */
#define IN_TABLE_SIZE 8                       /*  Internal nonce table size */
#define NONCE_REQUEST_TIMER           3       /* Internal Nonce Request is 3 sec */
#define NONCE_TIMER                   20      /* Nonce Timer, min. 3 sec, rec.10, max 20 */
#define TIME_SECURITY_LIFE            10      /* Inclusion Timer - 10 sec */



/* Global Cryptography variables in saved NVRAM */
extern BYTE networkKey[16];

#define ZW_WAKEUP_RESET   0   /* Woken up by reset or external int */
#define ZW_WAKEUP_WUT     1   /* Woken up by the WUT timer */
#define ZW_WAKEUP_SENSOR  2   /* Woken up by a wakeup beam */

/* Security header masks */
#define MASK_SECURITY_HEADER_INVALID              0x38
#define MASK_SECURITY_HEADER_NONCE_REQEUST        0x40
#define MASK_SECURITY_HEADER_NONCE_REPORT         0x80
#define MASK_SECURITY_HEADER_NONCE_PRESENT        0x80

#define NON_SECURE_NODE     0x00
#define SECURE_NODE         0x01

#define SECURITY_SCHEME_0   0x00

#ifdef SCHEME_0
#define SECURITY_SCHEME   SECURITY_SCHEME_0
#endif

//TO# 03329   Security smaple code doesn't wait for nonce_get finish
extern BOOL SecurityDelayedNonceReport;
extern BOOL SecurityNonceGetInProgress;               //indicate nonce_get callback is in waiting state

void
StopSecuritySendTimeOut(
  void);

void
StartSecurityTimeOut(
  BYTE timeOut);

void
StopSecurityTimeOut(
  void);

/*=============================   ZW_SendDataSecure   ============================
**    ZW_SendDataSecure
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern BYTE
ZW_SendDataSecure(
  BYTE tnodeID,
  BYTE *pBufData,
  BYTE dataLength,
  BYTE txSecOptions,
  VOID_CALLBACKFUNC(completedFunc)(BYTE));


/*=========================   ZW_SendDataSecureAbort   =======================
**    ZW_SendDataSecureAbort
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
void
ZW_SendDataSecureAbort(void);


/*===============================   AES_ECB   ================================
**    AES ECB - Electronic CodeBook Mode Block
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern void
AES_ECB(
  BYTE *key,
  BYTE *input,
  BYTE *output);


/*=========================   ProcessIncomingSecure   ========================
**    ProcessIncomingSecure
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern BYTE
ProcessIncomingSecure(
  BYTE tnodeID,
  BYTE *pPacket,
  BYTE txSecOptions,
  BYTE packetSize);


/*===============================   LoadKeys   ==============================
**    LoadKeys
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern void
LoadKeys(void);



/*=============================   InitSecurePowerUp   ========================
**    InitSecurePowerUp
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern void
InitSecurePowerUp(void);


/*===========================   InitSecureWakeUp   ===========================
**    InitSecureWakeUp - Reset internal nonce table, external nonce record,
**    and nonce request record (mark all as vacant).
**
**    Side effects
**
**--------------------------------------------------------------------------*/
extern void
InitSecureWakeUp(void);


/*=============================   PRNGInit   =================================
**    PRNGInit
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern void
PRNGInit(void);


/*=============================   AES128_Decrypt   ============================
**    AES128_Decrypta
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
extern void
AES128_Decrypt(
  const BYTE *ext_input,
  BYTE *ext_output,
  const BYTE *ext_key);


/*==============================   InitSecurity   ============================
**    Initialization of the Security module, can be called in ApplicationInitSW
**
**    This is an application function example
**
**--------------------------------------------------------------------------*/
void
InitSecurity(
  BYTE wakeUpReason);

void SecurityTimeOut();

#endif
