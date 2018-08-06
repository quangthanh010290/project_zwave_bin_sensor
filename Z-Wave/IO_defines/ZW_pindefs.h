/*******************************  ZW_PINDEFS.H  *******************************
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
 * Description: In/Out definitions for the ZW0102 based Z-Wave Standard Module
 *
 * Author:   Ivar Jeppesen
 *
 * Last Changed By:  $Author: sse $
 * Revision:         $Revision: 8506 $
 * Last Changed:     $Date: 2007-01-29 14:46:27 +0100 (ma, 29 jan 2007) $
 *
 ****************************************************************************/
#ifndef _ZW_PINDEFS_H_
#define _ZW_PINDEFS_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/


/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/

#ifdef __C51__
/* Macros for I/O Port controlling */

/* Set I/O pin as input:
 *    pin     - Z-Wave pin name
 *    pullup  - if not zero activate the internal pullup resistor
 */
#if defined(ZW020x) || defined(ZW030x)
#define PIN_IN(pin, pullup)  {pin##DDR |= (1<<pin);(pullup? (pin##PULL &= ~(1<<pin)): (pin##PULL |= (1<<pin)));}
#endif
#ifdef ZW010x
#define PIN_IN(pin, pullup)  pin##DDR |= (1<<pin)
#endif

/* Set I/O pin as output:
 *    pin     - Z-Wave pin name
 */
#if defined(ZW020x) || defined(ZW030x)
 /*disable the pullup when setting a pin to output*/
 /*Fix to#01263 disable the PULL up when seeting an IO to an output*/
#define PIN_OUT(pin)  {(pin##DDR &= ~(1<<pin));(pin##PULL |= (1<<pin));}
#endif

#ifdef ZW010x
#define PIN_OUT(pin)  (pin##DDR &= ~(1<<pin))
#endif

/* Read pin value:
 *    pin     - Z-Wave pin name
 */
#define PIN_GET(pin)  (pin##Port & (1<<pin))

/* Set output pin to 1:
 *    pin     - Z-Wave pin name
 */
#define PIN_ON(pin) (pin##Port |= (1<<pin))

/* Set output pin to 0:
 *    pin     - Z-Wave pin name
 */
#define PIN_OFF(pin) (pin##Port &= ~(1<<pin))

/* Toggle output pin:
 *    pin     - Z-Wave pin name
 */
#define PIN_TOGGLE(pin) (pin##Port ^= (1<<pin))

/* Button pressed */
#define BUTTON_PRESSED() ((PIN_GET(Button))?0:1)

#endif
#ifdef ZW010x
/* Z-Wave Button - INT0 */
#define	ButtonPort  P3
#define	ButtonDDR   P3DIR
#define	Button      2

/* Z-Wave Connector */

#ifdef ZW010x
/* Pin no. 2, IO7 - PROG_N */

/* Pin no. 3 */
#define ZEROXpinPort  P2
#define ZEROXpinDDR   P2DIR
#define ZEROXpin      2

/* Pin no. 4 */
#define PWM2Port    P3
#define PWM2DDR     P3DIR
#define PWM2        4

/* Pin no. 5 */
#define TRIACpinPort  P2
#define TRIACpinDDR   P2DIR
#define TRIACpin      3

/* Pin no. 6 */
#define INT0pinPort P3
#define INT0pinDDR  P3DIR
#define INT0pin     2

/* Pin no. 7, AD1/Vref */

/* Pin no. 8 */
#define IO9Port     P2
#define IO9DDR      P2DIR
#define IO9         4

/* Pin no. 9, AD1 */

/* Pin no. 10 */
#define MISOPort    P0
#define MISODDR     P0DIR
#define MISO        2

/* Pin no. 11, VCC */

/* Pin no. 12 */
#define CLKPort     P0
#define CLKDDR      P0DIR
#define CLK         0

/* Pin no. 13, GND */

/* Pin no. 14 */
#define MOSIPort    P0
#define MOSIDDR     P0DIR
#define MOSI        1

/* Pin no. 15, RES_n */

/* Pin no. 16 */
#define TXDpinPort  P3
#define TXDpinDDR   P3DIR
#define TXDpin      1

/* Pin no. 17, +3.3V */

/* Pin no. 18 */
#define RXDpinPort  P3
#define RXDpinDDR   P3DIR
#define RXDpin      0

/* Pin no. 19 */
#define IO10Port    P0
#define IO10DDR     P0DIR
#define IO10        3

/* Pin no. 20, AD2 */

#define IO0Port    	P0		/* None existing in ZW0102 */
#define IO0DDR     	P0DIR
#define IO0        	4
#endif /* ZW010x */

#ifdef ZW0101
/* Pin no. 1, GND */

/* Pin no. 3 */
#define IO0Port     P1
#define IO0DDR      P1DIR
#define IO0         0

/* Pin no. 4 */
#define IO1Port     P1
#define IO1DDR      P1DIR
#define IO1         1

/* Pin no. 5 */
#define IO2Port     P1
#define IO2DDR      P1DIR
#define IO2         2

/* Pin no. 6 */
#define IO3Port     P1
#define IO3DDR      P1DIR
#define IO3         3

/* Pin no. 7 */
#define IO4Port     P1
#define IO4DDR      P1DIR
#define IO4         4

/* Pin no. 8 */
#define IO5Port     P1
#define IO5DDR      P1DIR
#define IO5         5

/* Pin no. 9 */
#define IO6Port     P1
#define IO6DDR      P1DIR
#define IO6         6

/* Pin no. 10, IO7 - PROG_N */

/* Pin no. 11 */
#define ZEROXPort   P2
#define ZEROXDDR    P2DIR
#define ZEROX       2

/* Pin no. 12 */
#define PWM2Port    P3
#define PWM2DDR     P3DIR
#define PWM2        4

/* Pin no. 13 */
#define TRIACPort   P2
#define TRIACDDR    P2DIR
#define TRIAC       3

/* Pin no. 14 */
#define INT0Port    P3
#define INT0DDR     P3DIR
#define INT0        2

/* Pin no. 15, AD1/Vref */

/* Pin no. 16 */
#define IO9Port     P2
#define IO9DDR      P2DIR
#define IO9         4

/* Pin no. 17, AD0 */

/* Pin no. 18 */
#define MISOPort    P0
#define MISODDR     P0DIR
#define MISO        2

/* Pin no. 19, VCC */

/* Pin no. 20 */
#define CLKPort     P0
#define CLKDDR      P0DIR
#define CLK         0

/* Pin no. 21, GND */

/* Pin no. 22 */
#define MOSIPort    P0
#define MOSIDDR     P0DIR
#define MOSI        1

/* Pin no. 23, RES_n */

/* Pin no. 24 */
#define TXDPort     P3
#define TXDDDR      P3DIR
#define TXD         1

/* Pin no. 25, +3.3V */

/* Pin no. 26 */
#define RXDPort     P3
#define RXDDDR      P3DIR
#define RXD         0

/* Pin no. 27 (EEPROM CS) */
#define IO10Port    P0
#define IO10DDR     P0DIR
#define IO10        3

/* Pin no. 28, AD2 */

/* Pin no. 29 */
#define IO12Port    	P1
#define IO12DDR     	P1DIR
#define IO12        	4

/* Pin no. 30 */
#define IO13Port    	P2
#define IO13DDR     	P2DIR
#define IO13        	0

/* Pin no. 31 */
#define IO14Port    	P2
#define IO14DDR     	P2DIR
#define IO14        	1

/* Pin no. 32 */
#define IO15Port    	P2
#define IO15DDR     	P2DIR
#define IO15        	2

#endif /* ZW0101 */
#endif /*ZW010x*/

#if defined(ZW020x) || defined(ZW030x)
/*
#define BUTTON1_PIN 		TXDpin
#define BUTTON1_PINPort	TXDpinPort
#define BUTTON1_PINDDR	TXDpinDDR
#define BUTTON1_PINPULL	TXDpinPULL
*/
/* Z-Wave Button - INT1 */
#define	ButtonPort  P1
#define	ButtonDDR   P1DIR
#define	ButtonPULL  P1PULLUP
#define	Button      7

/* Z-Wave Connector */

/* Pin no. 20 */
#define ZEROXpinPort  P0
#define ZEROXpinDDR   P0DIR
#define ZEROXpinPULL   P0PULLUP
#define ZEROXpin      0

/* Pin no. 19 */
#define TRIACpinPort  P0
#define TRIACpinDDR   P0DIR
#define TRIACpinPULL   P0PULLUP
#define TRIACpin      1

/* Pin no. 20 */
#define ADC0Port  P0
#define ADC0DDR   P0DIR
#define ADC0PULL   P0PULLUP
#define ADC0      0

/* Pin no. 19 */
#define ADC1Port  P0
#define ADC1DDR   P0DIR
#define ADC1PULL   P0PULLUP
#define ADC1      1

/* Pin no. 18 */
#define TXDpinPort  P1
#define TXDpinDDR   P1DIR
#define TXDpinPULL   P1PULLUP
#define TXDpin      0

/* Pin no. 17 */
#define RXDpinPort  P1
#define RXDpinDDR   P1DIR
#define RXDpinPULL   P1PULLUP
#define RXDpin      1

/* Pin no. 18 */
#define ADC2Port  P1
#define ADC2DDR   P1DIR
#define ADC2PULL   P1PULLUP
#define ADC2      0

/* Pin no. 17 */
#define ADC3Port  P1
#define ADC3DDR   P1DIR
#define ADC3PULL   P1PULLUP
#define ADC3      1


/* Pin no. 10 */
#define MISOPort  P1
#define MISODDR   P1DIR
#define MISOPULL   P1PULLUP
#define MISO      2

/* Pin no. 9 */
#define MOSIPort  P1
#define MOSIDDR   P1DIR
#define MOSIPULL   P1PULLUP
#define MOSI      3

/* Pin no. 7 */
#define SCKPort  P1
#define SCKDDR   P1DIR
#define SCKPULL   P1PULLUP
#define SCK      4

/* Pin no. 6 */
#define SSNPort  P1
#define SSNDDR   P1DIR
#define SSNPULL   P1PULLUP
#define SSN      5


/* Pin no. 5 */
#define INT0pinPort P1
#define INT0pinDDR  P1DIR
#define INT0pinPULL  P1PULLUP
#define INT0pin     6

/* Pin no. 5 */
#define PWMPort P1
#define PWMDDR  P1DIR
#define PWMPULL  P1PULLUP
#define PWM     6

/* Pin no. 5 */
#define INT1pinPort P1
#define INT1pinDDR  P1DIR
#define INT1pinPULL  P1PULLUP
#define INT1pin     7

#endif /*ZW020x*/
#endif /* _ZW_PINDEFS_H_ */

