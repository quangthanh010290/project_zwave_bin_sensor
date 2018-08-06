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
#include "MyButtons.h"
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

static BYTE  b1_flag,b2_flag,b3_flag;
static BYTE b1_old, b1_new, b2_old, b2_new, b3_old, b3_new;
static BOOL  b1_update, b2_update, b3_update;
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
	buttonsUpdate(); //  Doc Gia tri, neu co thay doi thi update lai va gan co bao co thay doi
	
	
  /* Neu nut nhan dang bi nhan
  Tang bien dem de phat hien long press*/
  if (PIN_GET(MOSI) == FALSE)
  {
    /* Check if button is pushed long */
    if (buttonCount<=BUTTON_HELD_COUNT)
      buttonCount++;
    else
      buttonAction = BUTTON_IS_HELD;
  }
  /**/
  else
  {
    if (buttonCount)  /* Button has been pressed and is now released */
    {
      if  (buttonCount>DEBOUNCE_COUNT) // Kiem tra khoang thoi gian tu luc nhan den luc tha
      {
        if ((buttonCount<SHORT_PRESS_COUNT)) // Nho hon thoi gian Doi nut thi ko lam gi
        {
          buttonAction = BUTTON_WAS_PRESSED; // Xac dinh action da xay ra voi nut nhan
          buttonCount = 0; // Reset co

          /* 
		  Check Xem Co Double click khong
		  */
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
	buttonsInit();
  bButtonPollHandler = TimerStart(OneButtonPoll, 1, TIMER_FOREVER);

  if (bButtonPollHandler == 0xFF)
    return FALSE;
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


void buttonsInit(void)
{
		b1_new = PIN_GET(SCK);
		b2_new = PIN_GET(TRIACpin);
		b3_new = PIN_GET(MISO);
		b1_old = b1_new;
		b2_old = b2_new;
		b3_old = b3_new;
		b1_flag = FALSE;
		b2_flag = FALSE;
		b2_flag = FALSE;
}

void buttonsUpdate(void )
{
		b1_new = PIN_GET(SCK);
		b2_new = PIN_GET(TRIACpin);
		b3_new = PIN_GET(MISO);
		if((b1_new != b1_old)&&(b1_flag == FALSE)){
				b1_old = b1_new;
				b1_flag = TRUE;
		}
		if((b2_new != b2_old)&&(b2_flag == FALSE)){
				b2_old = b2_new;
				b2_flag = TRUE;
		}
		
		if((b3_new != b3_old)&&(b3_flag == FALSE)){
			b3_old = b3_new;
			b3_flag = TRUE;
		}
		
}

void buttonReset(BYTE button_num)
{
	switch(button_num){
		case 1:
			b1_flag = FALSE;
		break;
		case 2:
			b2_flag = FALSE;
		break;
		case 3:
			b3_flag = FALSE;
		break;
	}
}

BYTE buttonGetValue(BYTE button_num)
{
	switch(button_num){
		case 1:
			return b1_new;
		break;
		case 2:
			return b2_new;
		break;
		case 3:
			return b3_new;
		break;
	}
	return 0;
}

BOOL buttonGetFlag(BYTE button_num)
{
	switch(button_num){
		case 1:
			return b1_flag;
		break;
		case 2:
			return b2_flag;
		break;
		case 3:
			return b3_flag;
		break;
	}
	return FALSE;
		
}
void buttonSetUpdate(BYTE button_ind, BOOL value)
{
	switch(button_ind)
	{
		case 1:
				b1_update = value;
		break;
		case 2:
				b2_update = value;
		break;
		case 3:
				b3_update = value;
		break;
	}
}

BOOL buttonGetUpdate(BYTE button_ind)
{
		switch(button_ind)
		{
			case 1:
				return b1_update;
				break;
			case 2:
				return b2_update;
				break;
			case 3:
				return b3_update;
				break;
			default:
				return FALSE;
		}
}
