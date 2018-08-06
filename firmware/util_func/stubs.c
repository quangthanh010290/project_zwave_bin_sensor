/*******************************  TEMPLATE.C  *******************************
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

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include "Stubs.h"

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

/****************************************************************************/
/*                              PRIVATE DATA                                */
/****************************************************************************/

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/

/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/

void cbVoidVoid(void)
{
  // do nothing
}


void cbVoidByte(BYTE b)
{
    // do nothing
}

void cbVoidByteByte(BYTE b1, BYTE b2)
{
    // do nothing
}


void cbVoidWord(WORD w)
{
    // do nothing
}


BYTE cbByteVoid(void)
{
    return 0;// do nothing
}

BYTE cbByteByte(BYTE b)
{
    return 0;// do nothing
}


BYTE cbByteWord(WORD w)
{
    return 0;// do nothing
}


WORD cbWordVoid(void)
{
    return 0;// do nothing
}


WORD cbWordByte(BYTE b)
{
    return 0;// do nothing
}


WORD cbWordWord(WORD w)
{
    return 0;// do nothing
}



