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
 * Author:            Johann Sigfredsson
 *
 * Last Changed By:  $Author: oza $
 * Revision:         $Revision: 11214 $
 * Last Changed:     $Date: 2008-08-29 14:08:18 +0300 (Пт, 29 Сер 2008) $
 *
 ****************************************************************************/
#ifndef _ZW_AES128_h_
#define _ZW_AES128_h_


#define NETWORK_KEY_LENGTH  16


extern BYTE
ZW_SendDataSecure(
  BYTE tnodeID,
  BYTE *pBufData,
  BYTE dataLength,
  BYTE txSecOptions,
  VOID_CALLBACKFUNC(completedFunc)(BYTE));


extern void
AES_ECB(
  BYTE *key,
  BYTE *input,
  BYTE *output);


extern BYTE
ProcessIncomingSecure(
  BYTE tnodeID,
  BYTE *pPacket,
  BYTE txSecOptions,
  BYTE packetSize);


/*
Declaration: void LoadKeys()
Called: By system to generate encryption key and authentication key either at boot or
when key in NVRAM has been updated
Arguments: None
Return value: None
Global vars: networkKey[0..15] Read - from NVRAM
authKey[0..15] Write
encKey[0..15] Write
Temp vars: BYTE pattern[16]
Task: Generate authentication and encryption keys from network key.
*/
extern void
LoadKeys(void);


extern void
InitSecureNetworkKeys(void);


extern void
SaveSecureNetworkKey(void);


/*
Declaration: void InitSecurePowerUp()
Called: On power-up or reset
Task: Reset internal nonce table, external nonce record, and nonce request record
(mark all as vacant) and register timer service
Arguments: None
Return value: None
Global vars: None
ltemp data: None
*/
extern void
InitSecurePowerUp(void);


/*
Declaration: void InitSecureWakeUp()
Called: On wake-up
Task: Reset internal nonce table, external nonce record, and nonce request record
(mark all as vacant).
Arguments: None
Return value: None
Global vars: intNonce[i].nonce[0] Written
enNodeID Written
nrNodeID Written
noncePacket[1] Written
Temp data: BYTE i
*/
extern void
InitSecureWakeUp(void);


/*
Declaration: void PRNGInit()
Called: When node is powered up
Arguments: None
Return value: None
Global vars: prngState[0..15] Write
Temp vars: None
Task: Initialize PRNG State
*/
extern void
PRNGInit(void);


/*******************************************************************************
Declaration: void AES128_Decrypt(const unsigned char *ext_input,
                                 unsigned char *ext_output,
                                 const unsigned char *ext_key)

This is the main routine for Decryption. The C part only handles the parameter
passing to the assembler routines. For this reason the C-routine copies the input data
and the cipherkey to the absolut addresses ASM_input and ASM_key, which are located in
the data segment of the assembler module.
After the call of the assembler part the routine copies the decrypted data from
ASM_input to the memory area to which the generic pointer *ext_output is pointing.
*******************************************************************************/
extern void
AES128_Decrypt(
  const BYTE *ext_input,
  BYTE *ext_output,
  const BYTE *ext_key);


/* Global Cryptography variables in saved NVRAM */
extern BYTE networkKey[16];


#endif
