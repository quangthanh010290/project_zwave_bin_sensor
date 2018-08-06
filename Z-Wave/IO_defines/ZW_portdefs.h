/*******************************  ZW_PORTDEFS.H  *******************************
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
 * Description: IO definitions for the ZW0102 based Z-Wave Standard Module
 *
 * Author:   Ivar Jeppesen
 *
 * Last Changed By:  $Author: sse $
 * Revision:         $Revision: 8506 $
 * Last Changed:     $Date: 2007-01-29 14:46:27 +0100 (ma, 29 jan 2007) $
 *
 ****************************************************************************/
#ifndef _ZW_PORTDEFS_H_
#define _ZW_PORTDEFS_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/


/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/
#ifdef ZW010x
/* I/O Port initialization values */

/* Port 0 *******/
/* 0  I CLK   SPI Bus Serial Clock                   */
/* 1  I MOSI  SPI Bus Master Output/Slave Input      */
/* 2  I MISO  SPI Bus Master Input/Slave Output      */
/* 3  I IO10                                         */
#define P0_INIT     0x00
#define P0DIR_INIT  0xFF

/* Port 1 *******/
/* Internal only */

/* Port 2 **********************/
/* 0  I RXD1  UART Input line  */
/* 1  I TXD1  UART Output line */
/* 2  I ZEROX                  */
/* 3  I TRIAC                  */
/* 4  I IO9                    */
/* 5  I NC                     */
/* 6  I NC                     */
/* 7  I NC                     */
#define P2_INIT     0x00
#define P2DIR_INIT  0xFF

/* Port 3 *********/
/* 0  I RXD0  UART Input line                             */
/* 1  I TXD0  UART Output line                            */
/* 2  I INT0 External interrupt 0 input                   */
/* 3  I Button (INT1 External interrupt 1 input)          */
/* 4  I PWM2/T0       */
/* 5  I PWM3/T1       */
#define P3_INIT    	0x00
#define P3DIR_INIT  0xFF

#ifdef ZW0101
/* Production RF test pin definitions  (TP1) */
#define	RFTestPort   P2
#define RFTestDDR    P2DIR
#define	RFTest       5
#endif

/* EEPROM CS */
#define EECSPort    P0
#define EECSDDR     P0DIR
#define EECS        3
#endif
#if defined(ZW020x) || defined(ZW030x)

/* I/O Port initialization values */

/* Port 0 *******/
/* 0  I ZeroX   Triac ZeroX input                   */
/* 1  I Fire    Triac Pulse output      */
#define P0_INIT     0x00
#define P0DIR_INIT  0xFF
#define P0PULLUP_INIT 0x00


/* Port 1 **********************/
/* 0  I Tx Uart Output line  */
/* 1  I Rx UART input line   */
/* 2  I MISO                 */
/* 3  I MOSI                 */
/* 4  I SCK                  */
/* 5  I SS_N                 */
/* 6  I Int0                 */
/* 7  I Int1                 */
#define P1_INIT     0x00
#define P1DIR_INIT  0xFF
#define P1PULLUP_INIT 0x00



/* EEPROM CS */
#define EECSPort    P1
#define EECSDDR     P1DIR
#define EECSPULL    P1PULLUP
#define EECS        5

#endif


#endif /* _ZW_PORTDEFS_H_ */
