/*******************************  TEMPLATE.H  *******************************
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
 * Description: callback stubs' collection 
 *
 * Author:   
 * 
 * Last Changed By:  $Author: imj $
 * Revision:         $Revision: 2 $
 * Last Changed:     $Date: 2001-07-05 13:05:49 +0300 (Thu, 05 Jul 2001) $
 * 
 ****************************************************************************/
#ifndef STUBS_INCLUDED
#define STUBS_INCLUDED

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/

#include "ZW_typedefs.h"


/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/
#ifndef DEBUG
#define DBG_MODULE_NAME(x)
#define DBG_SCOPE_FL(x)
#define DBG_SCOPE_L(x)
#define DBG_SCOPE_F(x)
#define DBG_SCOPE_(x)
#define DBG_INIT()
#ifdef  __IAR_SYSTEMS_ICC__
#define DBG_MSG(...) //avr
//#define CODE
#else
//#define DBG_MSG  (void) //avr + 51
//#define DBG_MSG  (x) //avr + 51
#define DBG_MSG  //51
#endif
#endif


/*
typedef void (CODE *VOID_CALLBACKFUNC_VOID)(void);
//typedef void (CODE *VOID_CALLBACKFUNC_BYTE)(BYTE);
typedef void (CODE *VOID_CALLBACKFUNC_WORD)(WORD);
typedef BYTE (CODE *BYTE_CALLBACKFUNC_VOID)(void);
typedef BYTE (CODE *BYTE_CALLBACKFUNC_BYTE)(BYTE);
typedef BYTE (CODE *BYTE_CALLBACKFUNC_WORD)(WORD);
typedef WORD (CODE *WORD_CALLBACKFUNC_VOID)(void);
typedef WORD (CODE *WORD_CALLBACKFUNC_BYTE)(BYTE);
typedef WORD (CODE *WORD_CALLBACKFUNC_WORD)(WORD);
*/

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/


extern void 
cbVoidVoid(void); 

extern void 
cbVoidByte(BYTE); 

extern void 
cbVoidByteByte(BYTE, BYTE);

extern void 
cbVoidWord(WORD); 

extern BYTE 
cbByteVoid(void); 

extern BYTE 
cbByteByte(BYTE); 

extern BYTE 
cbByteWord(WORD); 

extern WORD 
cbWordVoid(void); 

extern WORD 
cbWordByte(BYTE); 

extern WORD 
cbWordWord(WORD); 


#endif


