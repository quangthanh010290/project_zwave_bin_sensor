/***********************  AES_module.h  *************************
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
 * Author:      Oleg Zadorozhnyy
 *
 * Last Changed By:  $Author: oza $

 * Revision:         $Revision: 1.00 $
 * Last Changed:     $Date: 2008/10/06 12:39:20 $
 *
 ****************************************************************************/
#ifndef _AES_MODULE_H_
#define _AES_MODULE_H_
/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/

#include <ZW_basis_api.h>
#include <ZW_debug_api.h>
#include <ZW_uart_api.h>
#include <ZW_non_zero.h>
#include <ZW_Security_AES_module.h>
#include <ZW_TransportLayer.h>
/****************************************************************************/
//* IMPORTANT:
//*
//* The AES128 module used is optimized for performance and memory.
//* Therefore it uses fixed memory areas and shares registers
//* between subroutines.
//*
//****************************************************************************/
extern void ASM_AES128_Encrypt(void);
extern void ASM_AES128_Decrypt(void);

/* AES assembler code needed fixed memory area specifics */
extern BYTE data ASM_input[16], ASM_key[16];

extern BYTE notSleep;


IBYTE *input_idata, *key_idata;
/* AES assembler code specifics END */

BYTE plaintext16ByteChunk[16];
BYTE cipherIndex;
BYTE blockIndex;
BYTE inputData[98];

BYTE outPayload[55];

/////////////////////////
/* Global Nonce variables */
#define IN_TABLE_SIZE 8                      /*  Internal nonce table size */
/* (variable; max 128) */
#define INTERNAL_NONCE_LIFE 200/*30*/               /* Internal nonce life is 3 sec */
/////////////////////////

/* (unit: 100 ms) */
#define ILLEGAL_NODE_ID NODE_BROADCAST       /* To indicate host-id as uninitialized */

/*
The internal nonce table "intNonce[]" is used to store internal nonces. A record in the table is vacant if
nonce[0] is zero. lifeLeft is decreased by one every 100 ms. When it has reached zero, nonce[0]
is set to zero.
*/
/* Internal nonce table */
typedef struct _INT_NONCE_
{
  /* First byte=0 -> vacant record */
  BYTE nonce[8];
  /* Life left (unit: 100 ms) */
  BYTE lifeLeft;
  /* ID of the node it was sent to */
  BYTE nodeID;
} INT_NONCE;


INT_NONCE intNonce[IN_TABLE_SIZE];

/*
The external nonce record is used to store the last received external nonce (enNonce[]), its associated
node ID "enNodeID", and the time until expiry "enLifeLeft". enLifeLeft is decreased by one every
100 ms. When is has reached zero, enNodeID is set to ILLEGAL_NODE_ID to indicate that the record is
vacant.
*/
// External nonce record
BYTE enNonce[8];    /* External nonce */
BYTE enNodeID=0;      /* Associated host id */

/*
The nonce request record stores the data necessary to send a payload packet once a response to a
nonce request is received. nrLifeLeft is decreased by one every 100 ms. When it has reached zero,
nrNodeID is set to ILLEGAL_NODE_ID to indicate that the record is vacant.
*/
/* Nonce request record */
BYTE *nrpBufData;                           /* Pointer to the payload data */
BYTE nrDataLength;                      /* Length of the payload data */
BYTE nrNodeID;                          /* Destignation node ID */
BYTE nrTXSecOptions;                    /* Security Options */
  void (*nrCompletedFunc)(BYTE txStatus); /* Completed callback function */


/* Global temporary variables in SRAM */
/* Auxiliary authentication data */
typedef struct _AUTHDATA_
{
  BYTE iv[16];          /* Initialization vector for enc, dec,& auth */
  BYTE sh;              /* Security Header for authentication */
  BYTE senderNodeID;    /* Sender ID for authentication */
  BYTE receiverNodeID;  /* Receiver ID for authentication */
  BYTE payloadLength;   /* Length of authenticated payload */
} AUTHDATA;

AUTHDATA authData;

BYTE tag[16];
BYTE pattern[16];

/* Pseudo-random number generator seed - is only reset on reset, */
/* on wakeup by either WUT or BEAM the previous seed is maintained via usage of the NON_ZERO_START block */
#ifdef ZW_SELF_HEAL
XBYTE prngState[16]             _at_ (NON_ZERO_START_ADDR + 3);  /* PRNG state */
#else
XBYTE prngState[16]             _at_ (NON_ZERO_START_ADDR);  /* PRNG state */
#endif

/* Message processing */
BYTE noncePacket[10];                    /* Buffer for outgoing nonce packet */
BYTE payloadPacket[sizeof(ZW_SECURITY_MESSAGE_ENCAP_FRAME)]; /* Buffer for outgoing packet with payload */
BYTE nonceRequestPacket[sizeof(ZW_SECURITY_MESSAGE_ENCAP_FRAME)]; /* Buffer for outgoing packet with payload */
/////////BYTE code nonceRequestPacket[2] = { COMMAND_CLASS_SECURITY, SECURITY_NONCE_GET };  /* Outgoing nonce request packet */

BYTE k[16], h[16], ltemp[16], btemp[16], i, j;
BYTE leastLifeLeft, newIndex;
BYTE ri;

/* Global Cryptography variables in saved NVRAM */
BYTE networkKey[16];                    /* The master key */

/* Global Cryptography variables in SRAM */
BYTE encKey[16];                        /* Encryption/decryption key */
BYTE authKey[16];                       /* Authentication key */

/////////////////////////
/*extern */BYTE  securitySendLife;
/*extern*/ BYTE  securitySendTimerHandle;
/*extern*/ BYTE  NonceTimerServiceHanler = 0;

extern BYTE securityLife;
extern BYTE securityTimerHandle ;

extern BYTE nodeSecure;
/////////////////////////
extern IBYTE nodeID;                /* This nodes Node-ID */
/////////////////////////
void StartSecuritySendTimeOut(BYTE timeOut);
void StopSecuritySendTimeOut();
extern void ( *cbFuncZWSecure ) ( BYTE txStatus );
/////////////////////////

  void AES128_Encrypt(const BYTE *ext_input, BYTE *ext_output, const BYTE *ext_key);

  void NonceRequestCompleted(BYTE txStatus);

  void NonceCompleted(BYTE txStatus);

#endif /*_AES_MODULE_H_*/
