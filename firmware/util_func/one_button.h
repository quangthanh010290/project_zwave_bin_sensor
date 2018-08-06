/******************************* one_button.h *******************************
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
 * Description: Implements functions that detects if a button has
 *              been pressed shortly or is beeing held
 *
 *              The module has 2 functions that can be used by an
 *              application:
 *
 *              OneButtonInit()  Initializes the 10ms timer that polls the
 *                               button state
 *
 *              OneButtonLastAction() This function returns the last action
 *                                    performed with the button.
 *
 *              The definitions of the timers used to determine when a
 *              button is pressed or held is in the one_button.h file and
 *              they are defined in 10ms counts.
 *
 * Author:   Peter Shorty
 *
 * Last Changed By:  $Author: psh $
 * Revision:         $Revision: 12502 $
 * Last Changed:     $Date: 2009-01-14 10:58:28 +0100 (on, 14 jan 2009) $
 *
 ****************************************************************************/
#ifndef _ONE_BUTTON_H_
#define _ONE_BUTTON_H_


/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

/* Minimum tinmes the button should be detected before we accept it */
#define DEBOUNCE_COUNT        2

/* Maximum number of times a button should be detected before we no longer
   detect it as a short button press */
#define SHORT_PRESS_COUNT    50

/* Minimum number of times a button should be detected before we say that it
   is held down */
#define BUTTON_HELD_COUNT   100


/****************************************************************************/
/*                              EXTERNAL DEFINED FUNCTIONS/DATA             */
/****************************************************************************/

/* Return values from LastButtonAction */
#define BUTTON_WAS_PRESSED  1
#define BUTTON_IS_HELD      2
#define BUTTON_WAS_RELEASED 3
#define BUTTON_TRIPLE_PRESS  4

/*==============================   LastButtonAction   =======================
**    This function returns the last button action detected.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
BYTE OneButtonLastAction();

/*===============================   OneButtonInit   ========================
**    This function initializes the one button polling
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
BOOL OneButtonInit();

#endif /*_ONE_BUTTON_H_*/
