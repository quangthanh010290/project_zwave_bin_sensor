/******************************* one_button.c *******************************
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
 * Revision:         $Revision: 12576 $
 * Last Changed:     $Date: 2009-01-16 12:53:30 +0100 (fr, 16 jan 2009) $
 *
 ****************************************************************************/
/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <ZW_basis_api.h>
#include <ZW_timer_api.h>
#include <ZW_pindefs.h>
#include <ZW_evaldefs.h>
#include <one_button.h>
#ifdef ZW_BUTTON_UART_CONTROL
#include <ZW_uart_api.h>
#endif
/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/
BYTE  buttonAction;
BYTE  buttonCount;

BYTE bTriplePress;
BYTE bTriplePressHandle;

#define TIME_TRIPLE_PRESS     100 /* Triple press timeout is set to 1.5sec */

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/


/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/
void
OneButtonTriplePressTimeout();


/*================================   OneButtonPoll   =========================
**    Poll function that polls the button every 10ms
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
static void             /*RET  Nothing                  */
OneButtonPoll( void )  /*IN  Nothing                   */
{
  /* Check button state */
#ifdef ZW_BUTTON_UART_CONTROL
  if(ZW_UART_REC_STATUS)
  {
    BYTE recData = ZW_UART_REC_BYTE;
    switch (recData)
    {
      case 'B':
        {
          buttonAction = BUTTON_WAS_PRESSED;
          ZW_UART_SEND_BYTE('B');
        }
        break;
      case 'D':
        {
          buttonAction = BUTTON_IS_HELD;
          ZW_UART_SEND_BYTE('D');
        }
        break;
      case 'U':
        {
          buttonAction = BUTTON_WAS_RELEASED;
          ZW_UART_SEND_BYTE('U');
        }
        break;
      case 'R':
        {
          /*Reset the module*/
          ZW_UART_SEND_BYTE('R');
          while(ZW_UART_SEND_STATUS);
          ZW_WATCHDOG_ENABLE; /*reset asic*/
          while(1);
        }
      break;
      default:
        {
          ZW_UART_SEND_BYTE('!');
        }
      break;
    }
  }
#endif
  if (BUTTON_PRESSED())
  {
    /* Check if button is pushed long */
    if (buttonCount<=BUTTON_HELD_COUNT)
      buttonCount++;
    else
      buttonAction = BUTTON_IS_HELD;
  }
  else
  {
    if (buttonCount)  /* Button has been pressed and is now released */
    {
      if  (buttonCount>DEBOUNCE_COUNT)
      {
        if ((buttonCount<SHORT_PRESS_COUNT))
        {
          buttonAction = BUTTON_WAS_PRESSED;
          buttonCount = 0;

          /* Handle tripple press */
          bTriplePress++;

          if (bTriplePress == 1)
          {
            /* First press, start timer */
            bTriplePressHandle = TimerStart(OneButtonTriplePressTimeout, TIME_TRIPLE_PRESS, 1);
            if (bTriplePressHandle == 0xFF)
              bTriplePressHandle = 0;
          }
          else if (bTriplePress == 3)
          {
            /* Triple press detected */
            if (bTriplePressHandle)
            {
              buttonAction = BUTTON_TRIPLE_PRESS;
              TimerCancel(bTriplePressHandle);
              bTriplePressHandle = 0;
            }
            bTriplePress = 0;
          }
        }
        else if (buttonAction == BUTTON_IS_HELD)
        {
          buttonAction = BUTTON_WAS_RELEASED;
          buttonCount = 0;
        }
      }
    }
  }
}

/*=========================   OneButtonTriplePressTimeout   =================
**    Timeout function for the tripple press detection
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
OneButtonTriplePressTimeout()
{
  bTriplePress = 0;
  bTriplePressHandle = 0;
}


/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/

/*===============================   OneButtonInit   ========================
**    This function initializes the one button polling
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
BOOL OneButtonInit()
{
  register BYTE bButtonPollHandler;

/****************************************************************************/
/*                 Initialize PRIVATE TYPES and DEFINITIONS                 */
/****************************************************************************/
  buttonAction = 0;
  buttonCount = 0;
  bTriplePress = 0;
  bTriplePressHandle = 0;

  bButtonPollHandler = TimerStart(OneButtonPoll, 1, TIMER_FOREVER);

  if (bButtonPollHandler == 0xFF)
    return FALSE;
#ifdef ZW_BUTTON_UART_CONTROL
  ZW_UART_INIT(1152);
  ZW_UART_SEND_BYTE('s');
#endif
  return TRUE;
}


/*==============================   LastButtonAction   =======================
**    This function returns the last button action detected.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
BYTE OneButtonLastAction()
{
  register bTemp;

  bTemp = buttonAction;
  if (buttonAction != BUTTON_IS_HELD)
    buttonAction = 0;
  return bTemp;
}
