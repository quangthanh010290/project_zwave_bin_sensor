/******************************* filename *******************************
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
 * Description:
 *
 * Author:   Thomas Kristensen
 *
 * Last Changed By:  $Author: iza $
 * Revision:         $Revision: 23076 $
 * Last Changed:     $Date: 2012-07-05 14:21:34 +0200 (to, 05 jul 2012) $
 *
 ****************************************************************************/
#ifndef _BIN_SENSOR_H_
#define _BIN_SENSOR_H_
/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/


/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/
#if defined(__ICCAVR__) || defined(__C51__)

/* Offsets into frame received */
#define OFFSET_CLASSCMD                       0x00
#define OFFSET_CMD                            0x01
#define OFFSET_PARAM_1                        0x02
#define OFFSET_PARAM_2                        0x03
#define OFFSET_PARAM_3                        0x04
#define OFFSET_PARAM_4                        0x05
#define OFFSET_PARAM_5                        0x06
#define OFFSET_PARAM_6                        0x07

/* How many classes do we belong to */
#ifdef BATTERY
#ifdef ZW_SELF_HEAL
#define CLASS_MEMBERSHIP_COUNT  6
#endif
#else
#define CLASS_MEMBERSHIP_COUNT  5
#endif


typedef struct s_nodeInfo_
{
   BYTE memberClass[CLASS_MEMBERSHIP_COUNT];  /* Command class membership */
} t_nodeInfo;

typedef struct s_nodeStatus_ {
  BYTE status;    /* Sensor report status    */
  BYTE level;     /* Reported sensor level   */
  BYTE onValue;   /* Destination node BASIC_SET on level */
  BYTE onTime;    /* Destination node on time (after sensor go off) */
} t_nodeStatus;

#if defined(ZW020x) || defined(ZW030x)
#define SENSOR_ACTIVATED() (PIN_GET(INT1pin)? FALSE : TRUE)
#endif /* ZW020x */

/* Default values */

#define MAGIC_VALUE                 0x42

/* Application states */
/* IDLE AND EXECUTE states are required by the battery libraray */ 

typedef enum {
	STATE_APPL_IDLE = 0, 		/* Application is idle = doing nothing - allow powerdown */
	STATE_HEAL_LOST,				/* HEAL: Device is Lost, unable to contact SUC */
	STATE_HEAL_ASSOCIATED_FAIL, /* HEAL: Unable to contact an associated Node, contact SUC */
	STATE_BASIC_CMD,						/* Button was pressed, node is sending a Basic Set to
                                             associated nodes (if they exist) or a Basic Report broadcast */
	STATE_BASIC_CMD_START,			/* Button was pressed, start sending a Basic Set to
                                             associated nodes or broadcast Basic Report */
	STATE_KEEP_ALIVE,						/* Button was pressed, start sending a Basic Set to
                                             associated nodes or broadcast Basic Report */
	STATE_LEARN_MODE,						/* Button was held down, keep node alive for an extended
                                             period to accept messages */
	STATE_LEARN_MODE_START,			/* Button was pressed, device is in learn mode */
	STATE_CHECK_INPUT,					/* Button was pressed, put device in learn mode */
	STATE_EXECUTE,							/* Polling the button and the sensor */
	STATE_WAKEUP_NOTIFICATION_START,	/* Waiting for a callback or other operation to finish, power down is prohibited */
	STATE_WAKEUP_NOTIFICATION,
	STATE_COMMAND_HANDLER,
	STATE_WAIT_FOR_BUTTON_RELEASE, /*Stay here until button is released*/
	STATE_BASIC_CMD_WAIT						/* Delay basic set to see if a triple press is performed */
}AppState_Typedef;


/* TO#2024 fix, changed timeout from 5 to 8 seconds */
#define WATCHDOG_TIMEOUT              8 /* If state lasts longer it should restart timer at safe location */

/* Sensor bounce count determines how many consecutive */
/* "Sensor activated" or "Sensor deactivated" messages must */
/* be received before a change is triggered by sending off */
/* a binary sensor report. */
#define DEFAULT_SENSORBOUNCECOUNT 5

/* PowerDownTimeout determines the number of seconds */
/* the sensor is kept alive between powerdowns.      */
/* The default is one second, which is probably      */
/* too little if you are routing in your network.    */
#define BIN_POWERDOWNTIMEOUT    (DEFAULT_POWERDOWNTIMEOUT + DEFAULT_KEEPALIVETIMEOUT)

/* KeepAliveTimeout determines the number of seconds */
/* the sensor is kept alive when the button is       */
/* activated for more than KEEPALIVEACTIVATEPERIOD   */
/* seconds. This can be used when installing the     */
/* sensor in a network.                              */
/* Default keepalive is 30 seconds. */
#define BIN_KEEPALIVETIMEOUT    (DEFAULT_KEEPALIVETIMEOUT + 5)

#define TIME_DELAY_BASIC_SET      100

#endif /* defined(__ICCAVR__) || defined(__C51__) */

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/
extern ZW_APPLICATION_TX_BUFFER txBuf;
extern BYTE myNodeID;
extern BYTE myHomeID[4];
extern BYTE currentState;
extern BYTE nextState;
extern BOOL keepAliveActive;
extern BYTE masterNodeID;

/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/

void StartWatchdog( void );
void StopWatchdog( void );

#endif /*_BATTERY_SENSOR_H_*/
