/***********************  ZW_Security_AES_module.c  *************************
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
 * Description: Z-Wave security AES module. Contains the functionality for
 *              implementing secure application communication using AES
 *              as encrypting/decrypting mechanism.
 *              Based on Cryptomatic Security spec and on JRMs C# OFB,
 *              ECB and CBCMAC implementation. Uses IAIK AES128
 *
 * Author:   Johann Sigfredsson, Oleg Zadorozhnyy
 *
 * Last Changed By:  $Author: oza $

 * Revision:         $Revision: 1.11 $
 * Last Changed:     $Date: 2008/09/02 12:39:20 $
 *
 ****************************************************************************/

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <AES_module.h>
#include <ZW_TransportSecurity.h>

//TO# 03329   Security smaple code doesn't wait for nonce_get finish

#define NONCEGET_TIMER_TIMEOUT 4

BOOL SecurityDelayedNonceReport = FALSE;               //we received  NonceReport before callback
BOOL SecurityNonceGetInProgress = FALSE;               //indicate nonce_get callback is in waiting state
static BYTE Security_NonceGetTimerHandle = 0xFF;     //prevent getting stuck
static char Security_NonceGetTimeOut = 0;

static void Security_StartNonceGetTimer();  //prevent getting stuck
static void Security_StopNonceGetTimer();
static void Security_NonceGetTimerCallback();


/*===============================   AES_ECB   ================================
**    AES ECB - Electronic CodeBook Mode Block
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
void
AES_ECB(
  BYTE *key,
  BYTE *inputDat,
  BYTE *outputDat)
{
  /* Make a macro instead */
  AES128_Encrypt(inputDat, outputDat, key);
}

/*===============================   AES_OFB   ================================
**    AES OFB Output Feedback Block Mode Encryption/Decryption
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
void
AES_OFB(
  BYTE *bufdata,
  BYTE bufdataLength)
{
  register BYTE i, j;
  register BYTE ivIndex;
  blockIndex = 0;

  memset((BYTE *)plaintext16ByteChunk, 0, 16);
  for (cipherIndex = 0; cipherIndex < bufdataLength; cipherIndex++)
  {
    plaintext16ByteChunk[blockIndex] = *(bufdata + cipherIndex);
    blockIndex++;
    if (blockIndex == 16)
    {
      AES_ECB(encKey, authData.iv, authData.iv);
      ivIndex = 0;
      for (i = (cipherIndex - 15); i <= cipherIndex; i++)
      {
        //  TO#03067 AES_OFB method fails with payload of 32bytes.
////        bufdata[i] = (BYTE)(plaintext16ByteChunk[i] ^ authData.iv[ivIndex]);
        bufdata[i] = (BYTE)(plaintext16ByteChunk[ivIndex] ^ authData.iv[ivIndex]);
        ivIndex++;
      }
      memset((BYTE *)plaintext16ByteChunk, 0, 16);
      blockIndex = 0;
    }
  }

  if (blockIndex != 0)
  {
    AES_ECB(encKey, authData.iv, authData.iv);
    ivIndex = 0;
    for (j = 0; j < blockIndex; j++)
    {
      bufdata[cipherIndex - blockIndex + j] = (BYTE)(plaintext16ByteChunk[j] ^ authData.iv[j]);
      ivIndex++;
    }
  }
}

/*==============================   AES_CBCMAC   ==============================
**    AES CBCMAC Cipher Block Chaining Mode MAC - Used in authentication
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
void
AES_CBCMAC(
  BYTE *bufdata,
  BYTE bufdataLength,
  BYTE *MAC)
{
  register BYTE i, j, k;

  // Generate input: [header] . [data]
  memcpy((BYTE *)&inputData[0], (BYTE *) &authData.iv[0], 20);
  memcpy((BYTE *)&inputData[20], bufdata, bufdataLength);
  // Perform initial hashing

  // Build initial input data, pad with 0 if length shorter than 16
  for (i = 0; i < 16; i++)
  {
    if (i >= sizeof(authData) + bufdataLength)
    {
      plaintext16ByteChunk[i] = 0;
    }
    else
    {
      plaintext16ByteChunk[i] = inputData[i];
    }

  }
  AES_ECB(authKey, &plaintext16ByteChunk[0], MAC);
  memset((BYTE *)plaintext16ByteChunk, 0, 16);

  blockIndex = 0;
  // XOR tempMAC with any left over data and encrypt

  for (cipherIndex = 16; cipherIndex < (sizeof(authData) + bufdataLength); cipherIndex++)
  {
    plaintext16ByteChunk[blockIndex] = inputData[cipherIndex];
    blockIndex++;
    if (blockIndex == 16)
    {
      for (j = 0; j <= 15; j++)
      {
        MAC[j] = (BYTE)(plaintext16ByteChunk[j] ^ MAC[j]);
      }
      memset((BYTE *)plaintext16ByteChunk, 0, 16);
      blockIndex = 0;

      AES_ECB(authKey, MAC, MAC);
    }
  }

  if (blockIndex != 0)
  {
    for (k = 0; k < 16; k++)
    {
      MAC[k] = (BYTE)(plaintext16ByteChunk[k] ^ MAC[k]);
    }
    AES_ECB(authKey, MAC, MAC);
  }
}

/*================================   AESRaw   ===============================
**    AES Raw
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void AESRaw(BYTE *pKey, BYTE *pSrc, BYTE *pDest)
Called: When individual 128-bit blocks of data have to be encrypted
Arguments: pKey Pointer to key (input; fixed size 16 bytes)
pSrc Pointer to source data (input; fixed size 16 bytes)
pDest Pointer to destination buffer (output; fixed size
16 bytes)
Return value: None
Global vars: None affected
Task: Encrypts 16 bytes of data at pSrc, using Raw AES and the key at pKey. The
16-byte result is written to pDest.*/
void
AESRaw(
  BYTE *pKey,
  BYTE *pSrc,
  BYTE *pDest)
{
  memcpy(pDest, pSrc, 16);
  AES_ECB(pKey, pSrc, pDest);
}

/*=============================   EncryptPayload   ============================
**    AES EncryptPayLoad
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*****************************************************************************/
/*  Declaration: void EncryptPayload(BYTE *pSrc, BYTE length)                */
/*       Called: When payload needs to be encrypted.                         */
/*    Arguments: pSrc Pointer to payload data to encrypt (input)             */
/*               length Length of payload data to encrypt (in bytes;         */
/*               between 0 and 30 both inclusive)                            */
/* Return value: None                                                        */
/*  Global vars: authData.iv[0..15] Read                                     */
/*               encKey[0..15] Read                                          */
/*               payloadPacket[9..l+8] Write (l is length)                   */
/*         Task: Encrypts dataLength bytes of data from pSrc, using AES-OFB, */
/*               the encryption key encKey and the initialization vector     */
/*               authData.iv. The result is written directly into            */
/*               payloadPacket.                                              */
/*****************************************************************************/
void
EncryptPayload(
  BYTE *pSrc,
  BYTE length)
{
  memcpy(payloadPacket + 10, pSrc, length);
  AES_OFB(payloadPacket + 10, length);
}

/*=============================   DecryptPayload   ============================
**    AES DecryptPayload
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void DecryptPayload(BYTE *pBufData, BYTE length)
Called: When payload needs to be decrypted.
Arguments: pBufData Pointer to data to decrypt (input/output)
length Length of payload data to decrypt (in bytes; can
be zero)
Return value: None
Global vars: authData.iv[0..15] Read
encKey[0..15] Read
Task: Decrypts data at pBufData[0..length-1], using AES-OFB, the encryption key
encKey and the initialization vector authData.iv. The result is written to
pBufData[0..length-1].*/
void
DecryptPayload(
  BYTE *pBufData,
  BYTE length)
{
  AES_OFB(pBufData, length);
}

/*=============================   MakeAuthTag   ==============================
**    AES MakeAuthTag
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void MakeAuthTag()
Called: When authentication tag is needed
Arguments: None
Return value: None
Global vars: authData Read (all 20 bytes)
authKey[0..15] Read
payloadPacket[9..p+8] Read - p is
authData.payloadLength)
payloadPacket[p+10..p+17] Write - p is
authData.payloadLength)
Task: Computes the authentication tag for the packet and its header information, using
AES-CBCMAC, the key authKey and the initialization vector
authData.iv. The result is written directly into the payload packet.*/
void
MakeAuthTag(void)
{
  AES_CBCMAC(payloadPacket + 10, authData.payloadLength,
             payloadPacket + authData.payloadLength + 11);
}

/*===========================   VerifyAuthTag   ==============================
**    VerifyAuthTag
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: BOOL VerifyAuthTag(BYTE *pPayload, BYTE payloadLength)
Called: When an authentication tag is to be verified
Arguments: None
Return value: True if the package was authentic, otherwise false
Global vars: authData Read (all 20 bytes)
authKey[0..15] Read
Temp vars: BYTE tag[8]
Task: Computes the authentication tag for the packet and its header information, using
AES-CBCMAC, the key authKey and the initialization vector
authData.iv. The result is compared to the authentication tag in the payload
packet.
*/
BOOL
VerifyAuthTag(
  BYTE *pPayload,
  BYTE payloadLength)
{
  AES_CBCMAC(pPayload, payloadLength, tag);
  return !memcmp(tag, pPayload + payloadLength + 1, 8);
}

/*===============================   LoadKeys   ==============================
**    LoadKeys
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
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
void
LoadKeys(void)
{
  memset((BYTE *)pattern, 0x55, 16);
  AESRaw(networkKey, pattern, authKey);   /* K_A = AES(K_N, pattern) */
  memset((BYTE *)pattern, 0xAA, 16);
  AESRaw(networkKey, pattern, encKey);    /* K_E = AES(K_N, pattern) */
}

void PRNGUpdate(void);

/*=============================   PRNGInit   =================================
**    PRNGInit
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void PRNGInit()
Called: When node is powered up
Arguments: None
Return value: None
Global vars: prngState[0..15] Write
Temp vars: None
Task: Initialize PRNG State
*/
void
PRNGInit(void)
{
  /* Reset PRNG State */
  memset(prngState, 0, 16);
  /* Update PRNG State */
  PRNGUpdate();
}

/*===============================   GetRNGData   =============================
**    GetRNGData
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
void
GetRNGData(
  BYTE *pRNDData,
  BYTE noRNDDataBytes)
{
  ZW_SetRFReceiveMode(FALSE);
  i = 0;
  do
  {
    ZW_GetRandomWord((BYTE *) (pRNDData + i), FALSE);
    i += 2;
  }
  while (--noRNDDataBytes && --noRNDDataBytes);
  /* Do we need to reenable RF? */
}

/*==============================   PRNGUpdate   ==============================
**    PRNGUpdate
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void PRNGUpdate()
Called: When fresh input from hardware RNG is needed
Arguments: None
Return value: None
Global vars: prngState[0..15] Modify
Temp data: BYTE k[16], h[16], ltemp[16], i, j
Task: Incorporate new data from hardware RNG into the PRNG State
*/
void
PRNGUpdate(void)
{
  /* H = 0xA5 (repeated x16) */
  memset((BYTE *)h, 0xA5, 16);
  /* The two iterations of the hardware generator */
  for (j = 0; j <= 1; j++)
  {
    /* Random data to K */
    GetRNGData(k, 16);
    /* ltemp = AES(K, H) */
    AESRaw(k, h, ltemp);
    /* H = AES(K, H) ^ H */
    for (i = 0; i <= 15; i++)
    {
      h[i] ^= ltemp[i];
    }
  }
  /* Update inner state */
  /* S = S ^ H */
  for (i = 0; i <= 15; i++)
  {
    prngState[i] ^= h[i];
  }
  /* ltemp = 0x36 (repeated x16) */
  memset((BYTE *)ltemp, 0x36, 16);
  /* S = AES(S, ltemp) */
  AESRaw(prngState, ltemp, btemp);
  memcpy(prngState, btemp, 16);
  /* Reenable RF */
  ZW_GetRandomWord(tag, TRUE);
  ZW_SetRFReceiveMode(TRUE);
}

/*=============================   PRNGOutput   ===============================
**    PRNGOutput
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void PRNGOutput(BYTE *pDest)
Called: When 8 bytes of output are needed.
Arguments: pDest Pointer to output data (output; always 8 bytes)
Return value: none
Global vars: prngState[0..15] Modify
Temp data: BYTE ltemp[16]
Task: Generate pseudo-random output data and update PRNG state
*/
void
PRNGOutput(
  BYTE *pDest)
{
  /* Generate output */
  /* ltemp = 0x5C (repeated x16) */
  memset((BYTE *)ltemp, 0x5C/*0xA5*/, 16);
  /* ltemp = AES(PRNGState, ltemp) */
  AESRaw(prngState, ltemp, btemp);
  /* pDest[0..7] = ltemp[0..7] */
  memcpy(pDest, btemp, 8);
  /* Generate next internal state */
  /* ltemp = 0x36 (repeated x16) */
  memset((BYTE *)ltemp, 0x36, 16);
  /* PRNGState = AES(PRNGState, ltemp) */
  AESRaw(prngState, ltemp, btemp);
  memcpy(prngState, btemp, 16);
}

void NonceTimerService(void);
void InitSecureWakeUp(void);

/*=============================   InitSecurePowerUp   ========================
**    InitSecurePowerUp
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
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
void
InitSecurePowerUp(void)
{
  /* Reset nonce tables */
  InitSecureWakeUp();
  /* Register timer service (100 ms, forever) */
  if (!NonceTimerServiceHanler)
    NonceTimerServiceHanler = TimerStart(NonceTimerService, 10, TIMER_FOREVER);
}

/*===========================   InitSecureWakeUp   ===========================
**    InitSecureWakeUp - Reset internal nonce table, external nonce record,
**    and nonce request record (mark all as vacant).
**
**    Side effects
**
**--------------------------------------------------------------------------*/
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
void
InitSecureWakeUp(void)
{
  // Reset internal nonce table
  for (i = 0; i < IN_TABLE_SIZE; i++)
  {
    intNonce[i].nonce[0] = 0;
  }
  // Reset external nonce record
  enNodeID = ILLEGAL_NODE_ID;
  // Reset nonce request record
  nrNodeID = ILLEGAL_NODE_ID;
  // Reset noncePacket (used in the message processing module)
  noncePacket[2] = 0;
}

/*============================   NonceTimerService   =========================
**    NonceTimerService
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void NonceTimerService()
Called: Every 100 ms
Task: Update lifeLeft in nonce table and records and mark timed-out records as
vacant.
Arguments: None
Return value: None
Global vars: intNonce[i].lifeLeft Modified
intNonce[i].nonce[0] May be written
enLifeLeft Modified
enNodeID May be written
nrLifeLeft Modified
nrNodeID May be read, may be written
nrCompletedFunc May be read
Temp data: BYTE i
*/
void
NonceTimerService(void)
{
  /* Update internal nonce table */

  for (i = 0; i < IN_TABLE_SIZE; i++)
  {
    /* If timed out... */
    if (intNonce[i].lifeLeft == 0)
    {
      /* Mark as vacant */
      intNonce[i].nonce[0] = 0;
    }
    /* Decrease life left counter */
    intNonce[i].lifeLeft--;
  }
}

/*=================================   MakeIN   ===============================
**    MakeIN
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void MakeIN(BYTE nodeID, BYTE *pDest)
Called: When a new internal nonce is needed
Arguments: nodeID Node ID of destination node
pDest Where to put the internal nonce (output; always
8 bytes)
Return value: None
Global vars: intNonce[i].lifeLeft May be read and/or written
intNonce[i].nonce[j] May be read and/or written
intNonce[i].nodeId Written
authData.iv[0..7] Written
Task: Find a vacant record in the intNonce[] table, generate a nonce and store it
there, copy generated nonce to the first half of the IV (but not to the packet)
Temp data: BYTE i, leastLifeLeft, newIndex
*/
void
MakeIN(
  BYTE tnodeID,
  BYTE *pDest)
{
  /* Find record in internal nonce table to use */
  leastLifeLeft = 255;
  for (i = 0; i < IN_TABLE_SIZE; i++)
  {
    /* If vacant... */
    if (intNonce[i].nonce[0] == 0)
    {
      newIndex = i;           /* Choose it */
      break;                  /* And we are done */
    }
    /* If less life left... */
    if (intNonce[i].lifeLeft < leastLifeLeft)
    {
      leastLifeLeft = intNonce[i].lifeLeft; /* Store new life left */
      newIndex = i;           /* And safe index as best bet */
    }
  }
  /* Generate nonce */
  /* Avoid collision check vs. old value */
  intNonce[newIndex].nonce[0] = 0;
  do
  {
    /* Generate new nonce */
    PRNGOutput(&authData.iv[0]);
    for (i = 0; i < IN_TABLE_SIZE; i++)
    {
      /* If collision... */
      if (authData.iv[0] == intNonce[i].nonce[0])
      {
        /* Invalidate */
        authData.iv[0] = 0;
        break;
      }
    }
  }
  while (!authData.iv[0]); /* Until valid nonce is found */
  /* Update intNonce[newIndex] and copy to pDest */
  /* Set life left */
  intNonce[newIndex].lifeLeft = INTERNAL_NONCE_LIFE;
  /* Set nodeID */
  intNonce[newIndex].nodeID = tnodeID;
  /* Copy nonce to nonce table */
  memcpy(intNonce[newIndex].nonce, authData.iv, 8);
  /* Copy nonce to destination */
  memcpy(pDest, authData.iv, 8);
}

/*================================   GetIN   =================================
**    GetIN
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: BOOL GetIN(BYTE ri)
Called: When a packet has been received and the associated internal nonce is needed
Arguments: ri: Receiver’s nonce ID of received packet
Return value: True on success, false if RI was not found
Global vars: intNonce[i].nonce[j] May be read
intNonce[i].nonce[0] May be written
intNonce[i].nodeID May be read
authData.iv[8..15] May be written
Task: Find the nonce, copy it to the second half of IV, and mark its record as vacant.
Return true if success and false if the nonce was not found.
Temp data: BYTE i, j
*/
BOOL
GetIN(
  BYTE ri)
{
  /* ri = 0 is not allowed */
  if (ri == 0)
  {
    return FALSE;
  }
  /* Find record */
  for (i = 0; i < IN_TABLE_SIZE; i++) /* For all records in the table... */
  {
    /* If ri found... */
    if (intNonce[i].nonce[0] == ri)
    {
      /* Copy to second half of IV */
      memcpy(authData.iv + 8, intNonce[i].nonce, 8);
      /* Mark all records IN TABLE with same nodeID as vacant */
      for (j = 0; j < IN_TABLE_SIZE; j++)
      {
        /* If same node id.. */
        if (intNonce[j].nodeID == intNonce[i].nodeID)
        {
          /* Mark as vacant */
          intNonce[j].nonce[0] = 0;
        }
      }
      /* Return with success */
      return TRUE;
    }
  }
  /* Return false (ri not found) */
  return FALSE;
}

/*==============================   RegisterEN   ==============================
**    RegisterEN
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void RegisterEN(BYTE nodeID, BYTE *pNonce)
Called: When a packet with external nonce has been received
Arguments: None
Return value: None
Global vars: enNonce Written
enNodeID Written
enLifeLeft Written
nrNodeId Read
nrpBufData May be read
nrDataLength May be read
nrTXSecOptions May be read
nrCompletedFunc May be read
Task: Copy the nonce to the external nonce buffer and react if there is pending data in
the nonce request record that awaits this nonce.
Temp data: None
Note: Is it OK to indirectly call ZW_SendData without checking if ready?
*/
void
RegisterEN(
  BYTE tnodeID,
  BYTE *pNonce)
{
  /* Copy nonce into external nonce record */
  memcpy(enNonce, pNonce, 8);
  enNodeID = tnodeID;
  /* Check if nonce request records needs this nonce */
  /* If NR Record’s nodeID matches... */
  if (tnodeID == nrNodeID)
  {
    /* Send content of NR Record */
    /* Mark record as vacant */
    nrNodeID = ILLEGAL_NODE_ID;
  }
}

/*=================================   GetEN   ================================
**    GetEN
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: BYTE GetEN(BYTE nodeID)
Called: When an external nonce is required
Arguments: nodeID Node ID of receiver of packet to send
Return value: RI of nonce if found, zero otherwise
Global vars: enNonce May be read
enNodeID Read, may be written
authData.iv[8..15] May be written
Task: Check if nonce is in table and copy it to second half of IV if found
Temp data: None
*/
BYTE
GetEN(
  BYTE tnodeID)
{
  /* If Node ID matches... */
  if (tnodeID == enNodeID)
  {
    /* Copy the nonce to second half of IV */
    memcpy(authData.iv + 8, enNonce, 8);
    /* Mark the nonce buffer as empty */
    enNodeID = ILLEGAL_NODE_ID;
    /* Return RI */
    return enNonce[0];
  }
  else
  {
    /* Return error */
    return 0;
  }
}

/*=============================   ZW_SendDataSecure   ============================
**    ZW_SendDataSecure
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: BYTE ZW_SendDataSecure(BYTE tnodeID, BYTE *pBufData,
  BYTE dataLength, BYTE txSecOptions,
  VOID_CALLBACKFUNC(completedFunc)(BYTE))
Called: Called from application to send a secure packet with payload
Arguments: tnodeID Node ID of receiver
pBufData Pointer to data to send
dataLength Length of data to send in bytes - max 30
txSecOptions Transmission security options
completedFunc Callback function to report back to application
Return value: TRUE on success, FALSE if error.
Global vars: nrNodeID May be read, may be written
nrpBufData May be written
nrDataLength May be written
nrLifeLeft May be written
nrTXSecOptions May be written
nrCompletedFunc May be written
authData May be written - some or all variables
payloadPacket May be written
Task: Security layer processing of outgoing packet, i.e. check if too long, get external
nonce, write header, encrypt, authenticate, and send.
Temp data: BYTE ri
Rules:
• The data buffer in the application must not be changed before completeFunc callback is called
• ZW_SendDataSecure does not support Ack. If this features is required, the receiving
application should send an acknowledge packet manually.
• It is suggested not to include the txOptions flags in this version as some flags are not allowed
and/or relevant.
txSecOptions flags:
• SEC_REQ_NONCE (0x40): Tell receiver that we would like to get a new nonce back immediately
 - for streaming data
Note: OK to call ZW_SendData() for sending nonce request without checking if ready? probably yes
since it replaces a subsequent call to ZW_SendData
*/
BYTE
ZW_SendDataSecure(
  BYTE tnodeID,
  BYTE *pBufData,
  BYTE dataLength,
  BYTE txSecOptions,
  VOID_CALLBACKFUNC(completedFunc)(BYTE))
{
#ifdef ZW_CONTROLLER
  BYTE NetTmp[16];
#endif
  if (!nodeSecure) return FALSE;
  /* Return error if dataLength > 30 */
  if (dataLength > 30)
  {
    return FALSE;
  }
  /* Get external nonce (or request one) */
  /* Get external nonce and RI */
  ri = GetEN(tnodeID);
  /* If not found (ri=0)... */
  if (ri == 0)
  {
    /* If ENR buffer in use... */
    /* Save needed data in NR record */
    nrpBufData = pBufData;
    nrDataLength = dataLength;
    nrNodeID = tnodeID;
    nrTXSecOptions = txSecOptions;
    nrCompletedFunc = completedFunc;
    /* Send nonce request */
    StartSecuritySendTimeOut(NONCE_TIMER);
    //TO# 3445
    StartSecuritySendDataTimer(SECURITY_SEND_DATA_TIME_OUT);

    nonceRequestPacket[0] = COMMAND_CLASS_SECURITY;
    nonceRequestPacket[1] = SECURITY_NONCE_GET;
    //TO# 03329
    SecurityNonceGetInProgress = TRUE;  // Nonce Get In Progress
    SecurityDelayedNonceReport = FALSE;
    Security_StartNonceGetTimer();

#ifdef ZW_CONTROLLER
    return (ZW_SendData(tnodeID, nonceRequestPacket, 2, txSecOptions | TRANSMIT_OPTION_AUTO_ROUTE | TRANSMIT_OPTION_EXPLORE, &NonceRequestCompleted));
#else
    return (ZW_SendData(tnodeID, nonceRequestPacket, 2, txSecOptions, &NonceRequestCompleted));
#endif
  }
  /* Write header (SH) */
  /* bitwise OR */
#ifdef ZW_CONTROLLER
  if (OutKeyNewController)
  {
    memcpy(NetTmp, networkKey, 16);
    memcpy(networkKey, inclusionKey, 16);
    LoadKeys();
  }
#endif

  StopSecuritySendTimeOut();


  memcpy(outPayload + 1, pBufData, dataLength);
  outPayload[0] = 0;
  pBufData = outPayload;
  dataLength++;
  authData.sh = 0x81;
  payloadPacket[0] = COMMAND_CLASS_SECURITY;
  payloadPacket[1] = authData.sh;
  /* Write sender’s nonce (SN) */
  MakeIN(tnodeID, payloadPacket + 2);
  /* Encrypt payload if present (EP) */
  EncryptPayload(pBufData, dataLength);
  /* Write receiver’s nonce identifier (RI) */
  payloadPacket[dataLength + 10] = ri;
  /* Generate authentication tag (AT) */
  authData.senderNodeID = nodeID;
  authData.receiverNodeID = tnodeID;
  authData.payloadLength = dataLength;
  memcpy(&authData.iv[0], &payloadPacket[2] , 8);
  memcpy(&authData.iv[8], enNonce, 8);
  /* Add AT */
  MakeAuthTag();
  /* Send data */
#ifdef ZW_CONTROLLER
  if (OutKeyNewController)
  {
    OutKeyNewController = 0;
    memcpy(networkKey, NetTmp, 16);
    LoadKeys();
  }
  return ZW_SendData(tnodeID, payloadPacket, dataLength + 19, nrTXSecOptions | TRANSMIT_OPTION_AUTO_ROUTE | TRANSMIT_OPTION_EXPLORE, completedFunc);
#else
  return ZW_SendData(tnodeID, payloadPacket, dataLength + 19, nrTXSecOptions, completedFunc);
#endif
}

/*=========================   ProcessIncomingSecure   ========================
**    ProcessIncomingSecure
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: BYTE ProcessIncomingSecure(BYTE tnodeID, BYTE *pPacket,
BYTE packetSize)
Called: When a packet with Command Class COMMAND_CLASS_SECURITYsecurity flag has been received.
Arguments: pPacket Pointer to packet data
packetSize Size of packet
tnodeID Node ID of sender
Return value: Size of decrypted data to be passed on to application layer (-1 if the packet did
not contain any payload; zero if it contained an empty payload)
Global vars: authData May be written (some or all variables)
noncePacket May be written
Task: Process security fields in packet and decrypt payload data if present. Decrypted
data, if any, is put in the pPacket buffer starting from address 9. Send nonce
packet if requested.
Temp data: None
Note: OK to call ZW_SendData() without checking if ready?
*/
BYTE
ProcessIncomingSecure(
  BYTE tnodeID,
  BYTE *pPacket,
  BYTE txSecOptions,
  BYTE packetSize)
{
  /* If no packet data... */
  StopSecuritySendTimeOut();
  if (packetSize < 1)
  {
    /* Done (not even a header present) */
    return 0xff;
  }
  /* Extract security header */
  authData.sh = pPacket[0];
  /* If any illegal flags are set... */
  if ((authData.sh & MASK_SECURITY_HEADER_INVALID) != 0)
  {
    /* Discard */
    return 0xff;
  }
  /* Send nonce if Nonce Request flag is set */
  if (((authData.sh & MASK_SECURITY_HEADER_NONCE_REQEUST) != 0) && (noncePacket[2] == 0))
  {
    /* If NR flag is set and noncePacket is */
    /* vacant... */
    notSleep = 0;
    noncePacket[0] = COMMAND_CLASS_SECURITY;
    /* Set nonce packet header */
    noncePacket[1] = SECURITY_NONCE_REPORT;
    /* Generate nonce */
    MakeIN(tnodeID, noncePacket + 2);
    /* Send packet */
#ifdef ZW_CONTROLLER
    ZW_SendData(tnodeID, noncePacket, 10, txSecOptions, NonceCompleted);
#else
    if (!ZW_SendData(tnodeID, noncePacket, 10, txSecOptions, NonceCompleted))
    {
      NonceCompleted(TRANSMIT_COMPLETE_FAIL);
      return 0xff;
    }
#endif
    StartSecuritySendTimeOut(NONCE_TIMER);
  }
  // Extract sender’s nonce if present
  if ((authData.sh & MASK_SECURITY_HEADER_NONCE_PRESENT) != 0)
  {
    RegisterEN(tnodeID, pPacket + 1);
  }
  /* All packets except nonce and nonce req. (which has been */
  /* processed already) has authentication tag */
  /* If too small for auth tag... */
  if (packetSize < 18)
  {
    /* Discard (incomplete packet) */
    return 0xff;
  }
  /* Copy sender’s nonce to IV */
  memcpy(authData.iv, pPacket + 1, 8);
  /* Find the internal nonce */
  /* If internal nonce is not found... check nonce identifier if present */
  if (!GetIN(pPacket[packetSize - 9]))
  {
    /* Discard (fake or expired RI) */
    return 0xff;
  }
  /* Verify authentication */
  authData.senderNodeID = tnodeID;
  authData.receiverNodeID = nodeID;
  authData.payloadLength = packetSize - 18;
  /* If not authentic... */
  if (!VerifyAuthTag(pPacket + 9, packetSize - 18))
  {
    /* Discard (wrong auth. tag) */
    return 0xff;
  }
  /* Process message if present */
  /* If contains message... */
  if ((authData.sh & ~MASK_SECURITY_HEADER_NONCE_REQEUST) == SECURITY_MESSAGE_ENCAPSULATION)
  {
    /* Decrypt it */
    DecryptPayload(pPacket + 9, packetSize - 18);
    /* Return new size */
    return (packetSize - 18);
  }
  /* Done; no payload */
  return 0xff;
}

/*=============================   NonceCompleted   ============================
**    NonceCompleted
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void NonceCompleted(BYTE txStatus)
Called: By lower protocol level when a nonce packet has been sent.
Arguments: txStatus If transmission was successful.
Return value: None
Global vars: noncePacket[2] Written
Task: Mark nonce packet buffer as vacant.
Temp data: None
*/
void
NonceCompleted(
  BYTE txStatus)
{
  /* Mark nonce packet as vacant */
  noncePacket[2] = 0;
}


//TO# 03329 Start Timer callback
void
Security_StartNonceGetTimer(
  void)
{
  if (Security_NonceGetTimerHandle != 0xFF)
  {
    ZW_TIMER_CANCEL(Security_NonceGetTimerHandle);
  }
  Security_NonceGetTimerHandle = ZW_TIMER_START(
                                   Security_NonceGetTimerCallback,
                                   TIMER_ONE_SECOND,
                                   TIMER_FOREVER);
  Security_NonceGetTimeOut = NONCEGET_TIMER_TIMEOUT;
}

//TO# 03329 Stop Timer callback
void
Security_StopNonceGetTimer(
  void)
{
  if (Security_NonceGetTimerHandle != 0xFF)
  {
    ZW_TIMER_CANCEL(Security_NonceGetTimerHandle);
  }
  Security_NonceGetTimerHandle = 0xFF;
}

//TO# 03329 Timer callback to prevent getting stuck
void
Security_NonceGetTimerCallback(
  void)
{
  Security_NonceGetTimeOut--;
  if (Security_NonceGetTimeOut <= 0)
  {
    Security_StopNonceGetTimer();
    SecurityNonceGetInProgress = FALSE;    // timer is expired
    if (SecurityDelayedNonceReport == TRUE)
    {
      // we have delayed n-r
      SecurityDelayedNonceReport = FALSE;
      Transport_SendDataSecure(enNodeID, (BYTE *)&txBuf, nrDataLength, TRANSMIT_OPTION_EXPLORE, cbFuncZWSecure);
    }
  }
}

/*=========================   NonceRequestCompleted   ========================
**    NonceRequestCompleted
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*
Declaration: void NonceRequestCompleted(BYTE txStatus)
Called: By lower protocol level when a nonce request packet has been sent.
Arguments: txStatus If transmission was successful.
Return value: None
Global vars: nrNodeID May be written
nrCompletedFunc May be read
Task: In case of error: Clear nonce request record and forward error to application
Temp data: None
*/
void
NonceRequestCompleted(
  BYTE txStatus)
{
  /* Do nothing if everything is OK */
  /* If status is OK... */
  if (txStatus == TRANSMIT_COMPLETE_OK)
  {
    /* Done */
///    StartSecuritySendTimeOut(NONCE_REQUEST_TIMER);
    //TO# 03329
    Security_StopNonceGetTimer();
    SecurityNonceGetInProgress = FALSE;   // nonse_get callback is finished

    if (SecurityDelayedNonceReport == TRUE)
    {
      SecurityDelayedNonceReport = FALSE;
      Transport_SendDataSecure(enNodeID, (BYTE *)&txBuf, nrDataLength, TRANSMIT_OPTION_EXPLORE, cbFuncZWSecure);
    }
    return;
  }
  /* Clear nonce request record */
  /* Mark as vacant */
  nrNodeID = ILLEGAL_NODE_ID;
  /* Forward error to application */
  /* Call applications compl. func */
  if (nrCompletedFunc)
  {
    nrCompletedFunc(txStatus);
  }
}

void
SecuritySendTimeOut()
{
  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_BYTE('Z');
  ZW_DEBUG_SEND_NUM(securitySendLife);
  ZW_DEBUG_SEND_BYTE(' ');

  if (securitySendLife == 0)
  {
    enNodeID = ILLEGAL_NODE_ID;
    if (securitySendTimerHandle != 0xff)
    {
      ZW_TIMER_CANCEL(securitySendTimerHandle);
      securitySendTimerHandle = 0xff;
    }

#ifdef ZW_CONTROLLER
    nodeSecure = 0;
    nodeSecureIncl = 0;
    if (isCtrlIncluded())
    {
      setCtrlNoneSecure();
    }
    else
    {
      AddSecuritySlave(nodeInWork, FALSE);
    }
#else
    ZW_DEBUG_SEND_BYTE('9');
    ZW_DEBUG_SEND_BYTE('_');
    Transport_SetNodeSecure(NON_SECURE_NODE);
#endif

    if ( cbFuncZWSecure != NULL )
    {
      cbFuncZWSecure(TRANSPORT_WORK_ERROR);
    }

    ZW_DEBUG_SEND_BYTE('_');
    ZW_DEBUG_SEND_BYTE('e');
    ZW_DEBUG_SEND_BYTE('e');

    return;
  }
  securitySendLife--;
}

void
StartSecuritySendTimeOut(
  BYTE timeOut)
{
  if (timeOut < 3) timeOut = 3;
//t
  if (timeOut > 20) timeOut = 20;

  securitySendLife = timeOut;
  if (securitySendTimerHandle != 0xff)
  {
    ZW_TIMER_CANCEL(securitySendTimerHandle);
    securitySendTimerHandle = 0xff;
  }
  securitySendTimerHandle = ZW_TIMER_START(SecuritySendTimeOut, TIMER_ONE_SECOND, TIMER_FOREVER);
  if ( cbFuncZWSecure != NULL )  cbFuncZWSecure(TRANSPORT_WORK_START);
}

void
StopSecuritySendTimeOut()
{
  if (securitySendTimerHandle != 0xff)
  {
    ZW_TIMER_CANCEL(securitySendTimerHandle);
    securitySendTimerHandle = 0xff;
  }
  if ( cbFuncZWSecure != NULL )  
    cbFuncZWSecure(TRANSPORT_WORK_END);
}

void
SecurityTimeOut()
{

  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_BYTE('X');
  ZW_DEBUG_SEND_NUM(securityLife);
  ZW_DEBUG_SEND_BYTE(' ');



  if (securityLife == 0)
  {
    if (securityTimerHandle != 0xff)
    {
      ZW_TIMER_CANCEL(securityTimerHandle);
      securityTimerHandle = 0xff;
    }
    nodeSecure = 0;
#ifdef ZW_CONTROLLER
    nodeSecureIncl = 0;
    if (isCtrlIncluded())
    {
      setCtrlNoneSecure();
    }
    else
    {
      AddSecuritySlave(nodeInWork, FALSE);
    }
#else
    Transport_SetNodeSecure(NON_SECURE_NODE);
#endif
    if ( cbFuncZWSecure != NULL )  
      cbFuncZWSecure(TRANSPORT_WORK_ERROR);

    ZW_DEBUG_SEND_BYTE('_');
    ZW_DEBUG_SEND_BYTE('d');
    ZW_DEBUG_SEND_BYTE('d');

    return;
  }
  securityLife--;
}

void
StartSecurityTimeOut(BYTE timeOut)
{
///  if (timeOut<10) timeOut = 10;
//t
  if (timeOut > 20) timeOut = 20;
  securityLife = timeOut;
  if (securityTimerHandle != 0xff)
  {
    ZW_TIMER_CANCEL(securityTimerHandle);
    securityTimerHandle = 0xff;
  }
  securityTimerHandle = ZW_TIMER_START(SecurityTimeOut, TIMER_ONE_SECOND, TIMER_FOREVER);

  if ( cbFuncZWSecure != NULL )  cbFuncZWSecure(TRANSPORT_WORK_START);

  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_BYTE('M');
  ZW_DEBUG_SEND_NUM(securityTimerHandle);
  ZW_DEBUG_SEND_BYTE(' ');

}

void
StopSecurityTimeOut()
{

  ZW_DEBUG_SEND_BYTE(' ');
  ZW_DEBUG_SEND_BYTE('S');
  ZW_DEBUG_SEND_BYTE(' ');

  if (securityTimerHandle != 0xff)
  {
    ZW_TIMER_CANCEL(securityTimerHandle);
    securityTimerHandle = 0xff;
  }
  if ( cbFuncZWSecure != NULL )  cbFuncZWSecure(TRANSPORT_WORK_END);
}

/*=============================   AES128_Encrypt   ============================
**    AES128_Encrypt
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
/*******************************************************************************
Declaration: void AES128_Encrypt(const unsigned char *ext_input,
                                 unsigned char *ext_output,const
                                 unsigned char *ext_key )

This is the main routine for Encryption. The C part only handles the parameter
passing to the assembler routines. For this reason the C-routine copies the input data
and the cipherkey to the absolut addresses ASM_input and ASM_key, which are located in
the data segment of the assembler module.
After the call of the assembler part the routine copies the encrypted data from
ASM_input to the memory area to which the generic pointer *ext_output is pointing.
*******************************************************************************/
void
AES128_Encrypt(
  const BYTE *ext_input,
  BYTE *ext_output,
  const BYTE *ext_key)
{
  input_idata = &ASM_input[0];
  key_idata = &ASM_key[0];

  memcpy(input_idata, ext_input, 16);
  memcpy(key_idata, ext_key, 16);
  ASM_AES128_Encrypt();
  memcpy(ext_output, input_idata, 16);
}

/*=============================   AES128_Decrypt   ============================
**    AES128_Decrypta
**
**    Side effects :
**
**--------------------------------------------------------------------------*/
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
void
AES128_Decrypt(
  const BYTE *ext_input,
  BYTE *ext_output,
  const BYTE *ext_key)
{

  input_idata = &ASM_input[0];
  key_idata = &ASM_key[0];

  memcpy(input_idata, ext_input, 16);
  memcpy(key_idata, ext_key, 16);
  ASM_AES128_Decrypt();
  memcpy(ext_output, input_idata, 16);
}

/*==============================   InitSecurity   ============================
**    Initialization of the Security module, can be called in ApplicationInitSW
**
**    This is an application function example
**
**--------------------------------------------------------------------------*/
void
InitSecurity(
  BYTE wakeUpReason)
{
  LoadKeys();
  if (wakeUpReason == ZW_WAKEUP_RESET)
  {
    /* Reset or External Int is the wakeup reason */
    /* Reinitialize Pseudo Random seed, for random generator, */
    /* with Z-Wave TRUE RF random generator */
    PRNGInit();
    /* Init the Security module */
    InitSecurePowerUp();
  }
  else if (wakeUpReason == ZW_WAKEUP_WUT)
  {
    /* WUT timer is the wakeup reason */
    /* Pseudo Random seed for random generator should still */
    /* be valid as placed in NON_ZERO_START_ADDR */
    /* Init the Security module */
    InitSecurePowerUp();
  }
#ifdef ZW_ZENSOR
  else if (wakeUpReason == ZW_WAKEUP_SENSOR)
  {
    /* Pseudo Random seed for random generator should still */
    /* be valid as placed in NON_ZERO_START_ADDR */
    /* Wakeup Beam is the wakeup reason */
    /* Init the Security module */
    InitSecurePowerUp();
  }
#endif
}





