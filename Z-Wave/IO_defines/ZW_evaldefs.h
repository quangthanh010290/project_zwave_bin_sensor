/*******************************  ZW_EVALDEFS.H  *******************************
 *           #######
 *           ##  ##
 *           #  ##    ####   #####    #####  ##  ##   #####
 *             ##    ##  ##  ##  ##  ##      ##  ##  ##
 *            ##  #  ######  ##  ##   ####   ##  ##   ####
 *           ##  ##  ##      ##  ##      ##   #####      ##
 *          #######   ####   ##  ##  #####       ##  #####
 *                                           #####
 *          Z-Wave, the wireless lauguage.
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
 * Description: IO definitions for the Z-Wave Evaluation board
 *
 * Author:   Ivar Jeppesen
 *
 * Last Changed By:  $Author: sse $
 * Revision:         $Revision: 8556 $
 * Last Changed:     $Date: 2007-01-31 10:24:50 +0100 (on, 31 jan 2007) $
 *
 ****************************************************************************/
#ifndef _ZW_EVALDEFS_H_
#define _ZW_EVALDEFS_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/


/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/

/* Evaluation board LEDs */

/* Turn LED on/off
 *  led - LED number
 */
#if defined (ZW010x) || defined(ZW020x) || defined(ZW030x)

#define LED_ON(led)   PIN_OFF(LED##led)
#define LED_OFF(led)  PIN_ON(LED##led)

#define LED_TOGGLE(led) PIN_TOGGLE(LED##led)
#endif

/* LED number       Z-Wave Device pin */

#ifdef ZW010x

#define LED1Port    ZEROXpinPort
#define LED1DDR     ZEROXpinDDR
#define LED1        ZEROXpin

#define LED2Port    PWM2Port
#define LED2DDR     PWM2DDR
#define LED2        PWM2

#define LED3Port    TRIACpinPort
#define LED3DDR     TRIACpinDDR
#define LED3        TRIACpin

#define LED4Port    IO9Port
#define LED4DDR     IO9DDR
#define LED4        IO9
#endif /*ZW010x*/

#if defined(ZW020x) || defined(ZW030x)
#define LED1Port    ZEROXpinPort
#define LED1DDR     ZEROXpinDDR
#define LED1PULL    ZEROXpinPULL
#define LED1        ZEROXpin

#define LED2Port    PWMPort
#define LED2DDR     PWMDDR
#define LED2PULL    PWMPULL
#define LED2        PWM

#define LED3Port    TRIACpinPort
#define LED3DDR     TRIACpinDDR
#define LED3PULL    TRIACpinPULL
#define LED3        TRIACpin

#endif /*ZW020x*/

#endif /* _ZW_EVALDEFS_H_ */
