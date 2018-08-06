
/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/

#include "config_app.h"

/* Enhanced Slave - needed for battery operation (RTC timer) on 100 series */
/* 200 Series have WUT */
#ifdef ZW_SLAVE_32
  #include <ZW_slave_32_api.h>
#else
  #ifdef  ZW_SLAVE
    #include <ZW_slave_api.h>
  #endif
#endif

#ifdef ZW_SELF_HEAL
  /* Lost functionality */
  #include <self_heal.h>
#endif
/* Z-Wave libraries */
#include <ZW_sysdefs.h>
#include <ZW_pindefs.h>
#include <ZW_evaldefs.h>
#include <ZW_classcmd.h>
#include <ZW_uart_api.h>
#include <ZW_debug_api.h>

/* Allows data storage of application data even after reset */
#include <ZW_non_zero.h>

#include <Switch3.h>
#include <eeprom.h>
#include <slave_learn.h>

/* Support functions for association */
#include <association.h>
/* Support functions for button press detection */
#include <MyButtons.h>

#include <ZW_TransportLayer.h>

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

#define BUTTON_KEEPALIVECOUNT     0xFE  /* x 10msec 2seconds for KeepAlive */

BYTE currentGroupIndex;
BYTE currentGroupNodeIndex;
BYTE frameSize;
BYTE stateWakeUPTimerHandle = 0xFF;

BYTE stateWatchdogTimerHandle = 0xFF;
BYTE stateWatchdogTimeout = WATCHDOG_TIMEOUT;

/* Function Prototypes */
BOOL AssociationSendBasicSet(BYTE broadcast);
void SendCompleted(BYTE txStatus);
void AssociationSendToGroup(void);
static void AssociationSendToNode1(void);
static void AssociationSendToNode2(void);
static void AssociationSendToNode3(void);
void AssociationSendNode1Fcn( BYTE bStatus);
void AssociationSendNode2Fcn( BYTE bStatus);
void AssociationSendNode3Fcn( BYTE bStatus);

BYTE myNodeID = 0;
BYTE myHomeID[4];
BYTE currentState    = STATE_APPL_IDLE;
BYTE nextState       = STATE_APPL_IDLE;
BYTE watchState      = STATE_APPL_IDLE;

BYTE currentGroupSize;

BOOL keepAliveActive = FALSE;

/* Next value to send in the BasicSet */
BYTE toggleBasicSet;

/* MasterNodeID specifies the node the sensor will try to contact on wakeup */
/* and when lost. Defined extern, currenly used by "battery" and "self-heal"*/
BYTE masterNodeID = 0x00;

BYTE bNWIStartup;

BYTE bTimerBasicSet;

/****************************************************************************/
/*                              PRIVATE DATA                                */
/****************************************************************************/

/* Node Status */
volatile t_nodeStatus sensorNodeStatus;

/* A list of the known command classes. Except the basic class which allways */
/* should be supported. Used when node info is send */
/* TO#1938 fix - COMMAND_CLASS_BASIC need never be mentioned here, as it is mandatory */
static code t_nodeInfo nodeInfo = {
                       COMMAND_CLASS_SENSOR_BINARY,
                       COMMAND_CLASS_MANUFACTURER_SPECIFIC,
                       COMMAND_CLASS_VERSION,
                       COMMAND_CLASS_ASSOCIATION,
};

#define nodeInfoForTransport nodeInfo
#define nodeInfoAfterIncluded nodeInfo
/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

ZW_APPLICATION_TX_BUFFER txBuf;

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/

extern BYTE powerDownTimeout;

/*==============================   cbVoidByte   ============================
**
**  Function:  stub for callback 
**
**  Side effects: None
**
**--------------------------------------------------------------------------*/
static void cbVoidByte(BYTE b)
{
}
static BYTE print_state;
static void PrintDebug(BYTE lastState, const char *s);
static void PrepareBasicSetWithValue(BYTE value);



/*============================   WatchdogCallback  ==============================
**    Check to see if the current state have run longer than WATCHDOG_TIMEOUT
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
WakeUPCallbackStart(void)
{
	ZW_DEBUG_SEND_STRING("In Wakup callback\n");
}

/*============================   StartWatchdog  ==============================
**    WatchdogTimer, make sure no state runs forever
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
StartWakeUP(void)
{
  if(stateWakeUPTimerHandle != 0xFF)
  {
    ZW_TIMER_CANCEL(stateWakeUPTimerHandle);
  }
  stateWakeUPTimerHandle = ZW_TIMER_START(WakeUPCallbackStart,
                                            TIMER_ONE_SECOND/2,
                                            TIMER_ONE_TIME);
  bTimerBasicSet = 0xFF;
}

/*============================   WatchdogCallback  ==============================
**    Check to see if the current state have run longer than WATCHDOG_TIMEOUT
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
WatchdogCallback( void )
{
  stateWatchdogTimeout--;
  if(stateWatchdogTimeout <= 0)
  {
    stateWatchdogTimeout = WATCHDOG_TIMEOUT;
    // State not changed
    if(watchState == currentState)
    {
			ZW_DEBUG_SEND_STRING("Watchdog callback\n");
      // Force into IDLE state
      currentState = STATE_APPL_IDLE;
    }
  }
}



/*============================   StartWatchdog  ==============================
**    WatchdogTimer, make sure no state runs forever
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
StartWatchdog( void )
{
  watchState = currentState;
  if(stateWatchdogTimerHandle != 0xFF)
  {
    ZW_TIMER_CANCEL(stateWatchdogTimerHandle);
  }
  ZW_DEBUG_SEND_STRING("Watchdog start\n");
  stateWatchdogTimerHandle = ZW_TIMER_START(WatchdogCallback,
                                            TIMER_ONE_SECOND,
                                            TIMER_FOREVER);
  ZW_DEBUG_SEND_NUM(stateWatchdogTimerHandle);
}

/*============================   StopWatchdog  ==============================
**    WatchdogTimer, stop the timer
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
StopWatchdog( void )
{
  watchState = STATE_APPL_IDLE;
  if(stateWatchdogTimerHandle != 0xFF)
  {
    ZW_TIMER_CANCEL(stateWatchdogTimerHandle);
    stateWatchdogTimerHandle = 0xFF;
  }
}

/*============================   SaveConfiguration   ======================
**    This function saves the current configuration to EEPROM
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
SaveConfiguration( void )
{
  ZW_DEBUG_SEND_STRING("Sve Configuration\n");

  ZW_MEM_PUT_BYTE(EEOFFSET_LOST_COUNTER, 0);

  /* Mark stored configuration as OK */
  ZW_MEM_PUT_BYTE(EEOFFSET_MAGIC, MAGIC_VALUE);
}

/*============================   SetDefaultConfiguration   ======================
**    Function resets configuration to default values.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /*RET  Nothing       */
SetDefaultConfiguration( void )
{
#ifdef ZW_SELF_HEAL
  SetDefaultNetworkUpdateConfiguration();
#endif
	ZW_DEBUG_SEND_STRING("Set default configuration\n");
}


/*============================   LoadConfiguration   ======================
**    This function loads the application settings from EEPROM.
**    If no settings are found, default values are used and saved.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                   /* RET  Nothing      */
LoadConfiguration( void )
{
  currentState = STATE_EXECUTE;
  /* Get this sensors identification on the network */
  MemoryGetID(myHomeID, &myNodeID);

  /* Check to see, if any valid configuration is stored in the EEPROM */
  if (ZW_MEM_GET_BYTE(EEOFFSET_MAGIC) == MAGIC_VALUE)
  {
    /* There is a configuration stored, so load it */
    ZW_DEBUG_SEND_STRING("Load Configuration\n");
  }
  else
  {
    /* Apparently there is no valid configuration in EEPROM, so load */
    /* default values and save them to EEPROM. */
    SetDefaultConfiguration();
    SaveConfiguration();
  }
  currentState = STATE_APPL_IDLE;
}

/*=====================   HasAssociatedNode   ====================
**    Checks if any nodes have been associated with this node.
**
**    Side effects:
**
**---------------------------------------------------------------*/
BOOL                   /*RET TRUE if any nodes have been associated */
HasAssociatedNode ( void )
{
  BYTE nodeIndex;

  for (nodeIndex = 0; nodeIndex < MAX_ASSOCIATION_IN_GROUP; nodeIndex++)
  {
    if (groups[currentGroupIndex].nodeID[nodeIndex])
    {
      return TRUE;
    }
  }
  return FALSE;
}

/*=========================   BasicSetWaitEnd   ====================
**    Timeoutfunction used to delay sending basic set to avoid
**    blocking learn mode with explorer frames
**
**    Side effects:
**
**---------------------------------------------------------------*/
void BasicSetWaitEnd()
{
  if (currentState == STATE_BASIC_CMD_WAIT)
    currentState = STATE_BASIC_CMD_START;
  bTimerBasicSet = 0xFF;

}

/*=========================   PrepareBasicSet   ====================
**    Prepares the transmit buffer with data for a Basic Set frame.
**
**    Side effects:
**
**---------------------------------------------------------------*/
void                    /*RET  Nothing       */
PrepareBasicSet ( void )
{
    ZW_DEBUG_SEND_STRING("Prapare Basic Set packet\n");

    txBuf.ZW_BasicSetFrame.cmdClass = COMMAND_CLASS_BASIC;
    txBuf.ZW_BasicSetFrame.cmd = BASIC_SET;
    txBuf.ZW_BasicSetFrame.value =  toggleBasicSet;
    frameSize = sizeof(ZW_BASIC_SET_FRAME);
}
void PrepareBasicSetWithValue(BYTE value)
{
		txBuf.ZW_BasicSetFrame.cmdClass = COMMAND_CLASS_BASIC;
    txBuf.ZW_BasicSetFrame.cmd = BASIC_SET;
    txBuf.ZW_BasicSetFrame.value =  value;
    frameSize = sizeof(ZW_BASIC_SET_FRAME);
}
/*=======================   PrepareBasicReport   ===================
**    Prepares the transmit buffer with data for a Basic Report
**    frame.
**
**    Side effects:
**
**---------------------------------------------------------------*/
void                    /*RET  Nothing       */
PrepareBasicReport ( void )
{
    txBuf.ZW_BasicSetFrame.cmdClass = COMMAND_CLASS_BASIC;
    txBuf.ZW_BasicSetFrame.cmd = BASIC_REPORT;
    txBuf.ZW_BasicSetFrame.value = toggleBasicSet;
    frameSize = sizeof(ZW_BASIC_REPORT_FRAME);
}

/*============================   BasicCmd   ======================
**    Send a Basic Set command to associated nodes if any,
**    otherwise broadcast a Basic Report (broadcasting
**    Basic Set is inadvisable).
**
**    Side effects:
**
**---------------------------------------------------------------*/
void                   /*RET  Nothing       */
BasicCmd( void )
{
  currentState = STATE_BASIC_CMD;
  /* A basic set is only sent when the node is member of a network */
  if (myNodeID != 0)
  {
		ZW_DEBUG_SEND_STRING("Have Node ID\n");
    if (HasAssociatedNode())
    {
			ZW_DEBUG_SEND_STRING("Have node in association list\n");

			if (buttonGetUpdate(1)){
				ZW_DEBUG_SEND_STRING("B1\n");
				buttonSetUpdate(1,FALSE);
				if (buttonGetValue(1))
				{
					PrepareBasicSetWithValue(FALSE);
				}
				else 
				{
					PrepareBasicSetWithValue(TRUE);
				}
				if (groups[currentGroupIndex].nodeID[0]) AssociationSendToNode1();
				else currentState = STATE_APPL_IDLE;
			}
			if (buttonGetUpdate(2)){
				ZW_DEBUG_SEND_STRING("B2\n");
				buttonSetUpdate(2,FALSE);
				if (buttonGetValue(2))
				{
					PrepareBasicSetWithValue(FALSE);
				}
				else 
				{
					PrepareBasicSetWithValue(TRUE);
				}
				if (groups[currentGroupIndex].nodeID[1])  AssociationSendToNode2();
				else currentState = STATE_APPL_IDLE;
			}
			if (buttonGetUpdate(3)){
				ZW_DEBUG_SEND_STRING("B3\n");
				buttonSetUpdate(3,FALSE);
				if (buttonGetValue(3))
				{
					PrepareBasicSetWithValue(FALSE);
				}
				else 
				{
					PrepareBasicSetWithValue(TRUE);
				}
				if (groups[currentGroupIndex].nodeID[2])  AssociationSendToNode3();
				else currentState = STATE_APPL_IDLE;
			}
			
    }
    else
    {
      currentState = STATE_APPL_IDLE;
			ZW_DEBUG_SEND_STRING("No Node In Asociation List\n");		
    }
    toggleBasicSet = (toggleBasicSet ? 0 : 1 );
    ZW_MEM_PUT_BYTE(EEOFFSET_SENSOR_LEVEL, toggleBasicSet);
		
	}
  else
  {
		ZW_DEBUG_SEND_STRING("Dont Have Node ID\n");
    currentState = STATE_APPL_IDLE;
  }
}



/*============================   AssociationStoreAll   ======================
**    Function description
**      Stores all groups in external Nonvolatile memory. Used by association.c
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
AssociationStoreAll( void )
{
  ZW_DEBUG_SEND_STRING("Association store all\n");
  MemoryPutBuffer(EEOFFSET_ASSOCIATION_START, (BYTE *)&groups[0], ASSOCIATION_SIZE, NULL);
}

/*============================   AssociationClearAll   ======================
**    Function description
**      Clears the Association area in Nonvolatile memory. Used by association.c
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
AssociationClearAll( void )
{
  ZW_DEBUG_SEND_STRING("Association Clear All\n");
  MemoryPutBuffer(EEOFFSET_ASSOCIATION_START, NULL, ASSOCIATION_SIZE, NULL);
  ZW_MEM_PUT_BYTE(EEOFFSET_ASSOCIATION_MAGIC, MAGIC_VALUE); /* Now ASSOCIATION should be OK */
}

/*============================   AssociationInit   ==========================
**    Function description
**      Reads the groups stored in the Nonvolatile memory
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
AssociationInit(void)
{
  ZW_DEBUG_SEND_STRING("Association Init\n");
  if (ZW_MEM_GET_BYTE(EEOFFSET_ASSOCIATION_MAGIC) != MAGIC_VALUE)
  {
    /* Clear it */
    AssociationClearAll();
  }
  MemoryGetBuffer(EEOFFSET_ASSOCIATION_START, (BYTE *)&groups[0], ASSOCIATION_SIZE);
}

/*============================   AssociationSendNext   ======================
**    Function description
**      Callback used when sending to association command
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
AssociationSendNext(
  BYTE bStatus)
{
///m  VerifyAssociatedTransmit(bStatus, &groups[currentGroupIndex].nodeID[0], currentGroupSize);
  VerifyAssociatedTransmit(bStatus, groups[currentGroupIndex].nodeID[currentGroupNodeIndex]);
  ZW_DEBUG_SEND_STRING("Association Send Next\n");
  currentGroupNodeIndex++;
///m  if (!AssociationSendBasicSet())
  if (!AssociationSendBasicSet(FALSE))
  {
    SendCompleted(bStatus);  /* Now done... */
  }
}

void AssociationSendNode1Fcn(BYTE bStatus)
{
		VerifyAssociatedTransmit(bStatus, groups[currentGroupIndex].nodeID[0]);
		ZW_DEBUG_SEND_STRING("Node 1 FCn\n");
		SendCompleted(bStatus);
}

void AssociationSendNode2Fcn(BYTE bStatus)
{
		VerifyAssociatedTransmit(bStatus, groups[currentGroupIndex].nodeID[1]);
		ZW_DEBUG_SEND_STRING("Node 2 FCn\n");
		SendCompleted(bStatus);
}

void AssociationSendNode3Fcn(BYTE bStatus)
{
		VerifyAssociatedTransmit(bStatus, groups[currentGroupIndex].nodeID[2]);
		ZW_DEBUG_SEND_STRING("Node 3 FCn\n");
		SendCompleted(bStatus);
}

/*============================   AssociationSendBasicSet   ======================
**    Function description
**      Sends Basic Set frame to the associated node currentGroupNodeIndex (if exist)
**      in the current currentGroupIndex group.
**    Side effects:
**
**--------------------------------------------------------------------------*/
BOOL            /*RET TRUE if there may be more associated nodes to send to */
AssociationSendBasicSet(
BYTE broadcast)
{
  
  for (; currentGroupNodeIndex < MAX_ASSOCIATION_IN_GROUP; currentGroupNodeIndex++)
  {
    if (groups[currentGroupIndex].nodeID[currentGroupNodeIndex])
    {
			ZW_DEBUG_SEND_STRING("Association Send Basic Set ");
			ZW_DEBUG_SEND_STRING("to Node ID:");
      ZW_DEBUG_SEND_NUM(groups[currentGroupIndex].nodeID[currentGroupNodeIndex]);
			ZW_DEBUG_SEND_STRING("\n");
			
      Transport_SendRequest(groups[currentGroupIndex].nodeID[currentGroupNodeIndex], (BYTE*)&txBuf, frameSize,
        (TRANSMIT_OPTION_RETURN_ROUTE | TRANSMIT_OPTION_ACK | TRANSMIT_OPTION_EXPLORE), AssociationSendNext, FALSE);
      return TRUE;
    }
  }
  if (broadcast)
  {
    //ZW_DEBUG_SEND_BYTE('E');
		ZW_DEBUG_SEND_STRING("Association Broadcast\n");
    Transport_SendRequest(NODE_BROADCAST, (BYTE*)&txBuf, frameSize, (TRANSMIT_OPTION_RETURN_ROUTE | TRANSMIT_OPTION_ACK), SendCompleted, FALSE);
    return TRUE;
  }
	ZW_DEBUG_SEND_STRING("Finish send to all node\n");
  return FALSE;
}

/*============================   SendCompleted   ======================
**    Callback for association sends...
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                    /*RET  Nothing       */
SendCompleted(
BYTE txStatus)          /*IN   Transmission result           */
{
  ZW_DEBUG_SEND_STRING("Send Completed Callback\n");
  currentState = STATE_APPL_IDLE;
}

/*============================   AssociationSendToGroup   ====================
**    Function description
**      Initiate transmission of a preprepared Basic Set frame to nodes in
**      first association group.
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
AssociationSendToGroup(void)
{
  
  /* Setup to start transmit to first group (for now we have only one group!) */
  currentGroupIndex = 0;
  /* Setup to start with first node */
  currentGroupNodeIndex = 0;
///m  if (!AssociationSendBasicSet())
    if (!AssociationSendBasicSet(TRUE))
{
    SendCompleted(TRANSMIT_COMPLETE_OK);  /* Now done... */
  }
}
void AssociationSendToNode1(void)
{
	ZW_DEBUG_SEND_STRING("Association Send to Node");
	ZW_DEBUG_SEND_NUM(groups[currentGroupIndex].nodeID[0]);
	ZW_DEBUG_SEND_STRING("\n");
	Transport_SendRequest(groups[currentGroupIndex].nodeID[0], (BYTE*)&txBuf, frameSize,
        (TRANSMIT_OPTION_RETURN_ROUTE | TRANSMIT_OPTION_ACK | TRANSMIT_OPTION_EXPLORE), AssociationSendNode1Fcn, FALSE);
	
}
void AssociationSendToNode2(void)
{
	ZW_DEBUG_SEND_STRING("Association Send to Node");
	ZW_DEBUG_SEND_NUM(groups[currentGroupIndex].nodeID[1]);
	ZW_DEBUG_SEND_STRING("\n");
	Transport_SendRequest(groups[currentGroupIndex].nodeID[1], (BYTE*)&txBuf, frameSize,
        (TRANSMIT_OPTION_RETURN_ROUTE | TRANSMIT_OPTION_ACK | TRANSMIT_OPTION_EXPLORE), AssociationSendNode2Fcn, FALSE);
	
}
void AssociationSendToNode3(void)
{
	ZW_DEBUG_SEND_STRING("Association Send to Node");
	ZW_DEBUG_SEND_NUM(groups[currentGroupIndex].nodeID[2]);
	ZW_DEBUG_SEND_STRING("\n");
	Transport_SendRequest(groups[currentGroupIndex].nodeID[2], (BYTE*)&txBuf, frameSize,
        (TRANSMIT_OPTION_RETURN_ROUTE | TRANSMIT_OPTION_ACK | TRANSMIT_OPTION_EXPLORE), AssociationSendNode3Fcn, FALSE);
	
}
/*============================   LearnCompleted   ========================
**    Callback which is called on learnmode completed
**  Application specific handling of LearnModeCompleted - called from
**  slave_learn.c
**--------------------------------------------------------------------------*/
void                  /*RET Nothing */
LearnCompleted(
  BYTE bNodeID)         /* IN resulting nodeID */
{
  ZW_DEBUG_SEND_STRING("Learn Completed\n");
  if (myNodeID == 0)
  {
#ifdef ZW_SELF_HEAL
    UpdateNetworkUpdateCount(TRUE); // Reset networkupdatecount
#endif
    SetDefaultConfiguration();
    SaveConfiguration();
    AssociationClearAll();
    AssociationInit();
		
  }
	if(bNodeID == 0){
		SetDefaultConfiguration();
    SaveConfiguration();
    AssociationClearAll();
    AssociationInit();
	}
  currentState = STATE_APPL_IDLE;
	myNodeID = bNodeID;
  Transport_OnLearnCompleted(bNodeID);
}

/*============================   CheckButtonPress   ==========================
**    Check and handle button press from ApplicationPoll
**
**
**--------------------------------------------------------------------------*/
void
CheckButtonPress( void )
{
  register BYTE bStatus;

  bStatus = OneButtonLastAction();

  if(bStatus == BUTTON_WAS_PRESSED)
  {
#ifdef ZW_SELF_HEAL
    currentHealMode = HEAL_NONE;
#endif

    currentState = STATE_APPL_IDLE;

    if(lostCount != 0 && currentHealMode != HEAL_NONE)
    {
      HealComplete(FALSE);
      currentHealMode = HEAL_NONE;
    }

    /* TO#2043 Fix */
    // nextState = STATE_BASIC_CMD_START;
    nextState = STATE_BASIC_CMD_WAIT;
  }
  else if (bStatus == BUTTON_TRIPLE_PRESS)
  {
    currentState = STATE_LEARN_MODE_START;
    // No button press
    if(currentState == STATE_CHECK_INPUT)
    {
      currentState = STATE_APPL_IDLE;
    }
  }

	if(buttonGetFlag(1)){
		currentState = STATE_APPL_IDLE;
		nextState = STATE_BASIC_CMD_START;
		buttonSetUpdate(1,TRUE);
		buttonReset(1);
	}
	if(buttonGetFlag(2)){
		currentState = STATE_APPL_IDLE;
		nextState = STATE_BASIC_CMD_START;
		buttonSetUpdate(2,TRUE);
		buttonReset(2);
	}
	if(buttonGetFlag(3)){
		currentState = STATE_APPL_IDLE;
		nextState = STATE_BASIC_CMD_START;
		buttonSetUpdate(3,TRUE);
		buttonReset(3);
	}

}

/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/


/*============================   ApplicationInitHW   ========================
**    Initialization of non Z-Wave module hardware
**
**    Side effects:
**       Returning FALSE from this function will force the API into
**       production test mode.
**--------------------------------------------------------------------------*/

BYTE                       /* RET TRUE        */
ApplicationInitHW(
BYTE bWakeupReason
)  /* IN  Nothing     */
{
#ifdef APPL_PROD_TEST
#ifdef __C51__
  IBYTE i;

  SET_PRODUCTIONTEST_PIN;
  for (i = 0; i < 10; i++) ;  /* Short delay... */
  if (IN_PRODUCTIONTEST) /* Anyone forcing it into productiontest mode ? */
  {
    /* Return FALSE to enter production test mode */
    return(FALSE);
  }
#endif /* __C51__ */

#endif /* APPL_PROD_TEST */

  /* hardware initialization */
	PIN_IN(SCK,1);
	PIN_IN(TRIACpin,1);
  PIN_IN(MISO, 1);
	PIN_IN(MOSI,1);

  /* Setup specifics for current dim module */
  PIN_OUT(PWM);
	PIN_OUT(ZEROXpin);
	
  PIN_OFF(PWM);
	PIN_OFF(ZEROXpin);



  Transport_OnApplicationInitHW(bWakeupReason);

  /* Production test mode */
  /* return(FALSE); */

  /* Return true to continue normal operating mode */
  return(TRUE);
}
/*===========================   ApplicationInitSW   =========================
**    Initialization of the Application Software variables and states
**    100 series: Not called on RTC wakeup
**    200 series: Called on WUT wakeup
**
**--------------------------------------------------------------------------*/
BYTE                      /*RET  TRUE       */
ApplicationInitSW( void ) /* IN   Nothing   */
{
  ZW_DEBUG_INIT(96);
  ZW_UART_INIT(1152);
	ZW_DEBUG_SEND_STRING("Hello\n");

  /* Signal that the sensor is awake */
  LoadConfiguration();
  AssociationInit();

  /* Initialize button detection */
  OneButtonInit();

  Transport_OnApplicationInitSW(nodeInfoForTransport, sizeof(nodeInfoForTransport),
    FALSE, EEOFFSET_TRANSPORT_SETTINGS_START, EEOFFSET_TRANSPORT_SETTINGS_SIZE, NULL);

  bTimerBasicSet = 0xFF;

  /* Set Nwtwork wide inclusion active if we dont have aa node ID */
  if (myNodeID)
    bNWIStartup = FALSE;
  else
    bNWIStartup = TRUE;

  if (ZW_MEM_GET_BYTE(EEOFFSET_MAGIC) == MAGIC_VALUE)
  {
    toggleBasicSet = ZW_MEM_GET_BYTE(EEOFFSET_SENSOR_LEVEL);
  }
  else
  {
    toggleBasicSet = 0;
    ZW_MEM_PUT_BYTE(EEOFFSET_SENSOR_LEVEL, toggleBasicSet);
  }
  currentState = STATE_APPL_IDLE;

  return(TRUE);
}

/*============================   ApplicationTestPoll   ======================
**    Function description
**      This function is called when the slave enters test mode.
**
**    Side effects:
**       Code will not exit until it is reset
**--------------------------------------------------------------------------*/
void ApplicationTestPoll(void)
{
}

/*=============================  ApplicationPoll   =========================
**    Application poll function for the Battery operated Bin Sensor
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                    /*RET  Nothing                  */
ApplicationPoll( void ) /* IN  Nothing                  */
{
#ifdef ZW_DEBUG
  ZW_DEBUG_CMD_POLL();
#endif /* ZW_DEBUG */

  if (bNWIStartup)
  {
    StartLearnModeNow(ZW_SET_LEARN_MODE_NWI);
    bNWIStartup = FALSE;
    currentState = STATE_LEARN_MODE;
  }

  switch (currentState)
  {

    case STATE_APPL_IDLE:
    {
			PrintDebug(STATE_APPL_IDLE,"State: IDLE\n");
      /* Stop the watchdog and set the watchState to APPL_IDLE */
      StopWatchdog();
      /* If something else is in queue do that */
      if(nextState != STATE_APPL_IDLE)
      {
        currentState = nextState;
        nextState = STATE_APPL_IDLE;
      } else { /* When theres nothing else to do, check button and sensor */
        currentState = STATE_CHECK_INPUT;
      }
			
      break;
    }
    case STATE_COMMAND_HANDLER:
    {
      /* Do nothing  */
			PrintDebug(STATE_COMMAND_HANDLER,"State: STATE_COMMAND_HANDLER\n");
      break;
    }
    case STATE_HEAL_LOST:
    {
			PrintDebug(STATE_HEAL_LOST,"State: STATE_HEAL_LOST\n");
     // Do nothing, we are currently searching for a SUC node. - Accept button input
     currentState = STATE_CHECK_INPUT;
     nextState = STATE_HEAL_LOST;
		 
     break;
    }
    /* TO#2043 Fix */
    case STATE_BASIC_CMD_WAIT:
			PrintDebug(STATE_BASIC_CMD_WAIT,"State: STATE_BASIC_CMD_WAIT\n");
			if (bTimerBasicSet == 0xFF)
      {
        bTimerBasicSet = TimerStart(BasicSetWaitEnd, TIME_DELAY_BASIC_SET, TIMER_ONE_TIME);
        if (bTimerBasicSet == 0xFF)
        {
          BasicSetWaitEnd();
        }
      }
      CheckButtonPress();	
      break;
    case STATE_BASIC_CMD_START:
    {
			PrintDebug(STATE_BASIC_CMD_START,"State: STATE_BASIC_CMD_START\n");
      BasicCmd();
      /* Make sure we don't get stuck */
      StartWatchdog();
			
      break;
    }
    case STATE_BASIC_CMD:
    {
      // Do nothing but wait for basic set/report to complete
			PrintDebug(STATE_BASIC_CMD,"State: STATE_BASIC_CMD\n");
      break;
    }
    case STATE_LEARN_MODE:
      if (OneButtonLastAction()==BUTTON_WAS_PRESSED)
      {
        /* Stop learn mode */
        StartLearnModeNow(FALSE);
        currentState = STATE_APPL_IDLE;
      }
			PrintDebug(STATE_LEARN_MODE,"State: STATE_LEARN_MODE\n");
      break;

    case STATE_LEARN_MODE_START:
    {
      currentState = STATE_LEARN_MODE;
      StartWatchdog();
      StartLearnModeNow(ZW_SET_LEARN_MODE_CLASSIC);
			PrintDebug(STATE_LEARN_MODE_START,"State: STATE_LEARN_MODE_START\n");
      break;
    }
    case STATE_CHECK_INPUT:
    {
			PrintDebug(STATE_CHECK_INPUT,"State: STATE_CHECK_INPUT\n");
      CheckButtonPress();
			
      break;
    }
    case STATE_EXECUTE:
    {
      /* Do nothing but check button press - we are waiting for an operation to finish */
			PrintDebug(STATE_EXECUTE,"State: STATE_EXECUTE\n");
      CheckButtonPress();
			
      break;
    }
    default:
    {
      /* BTW: Something is terribly wrong if 'default' is ever executed */
      ZW_DEBUG_SEND_BYTE('X');
      ZW_DEBUG_SEND_NUM(currentState);
      currentState = STATE_APPL_IDLE;
      break;
    }

  }
}

/*========================   ApplicationCommandHandler   ====================
**    Handling of a received application commands and requests
**
**
**--------------------------------------------------------------------------*/
void                              /*RET Nothing                  */
Transport_ApplicationCommandHandler(
  BYTE  rxStatus,                 /* IN Frame header info */
  BYTE  sourceNode,               /* IN Command sender Node ID */
  ZW_APPLICATION_TX_BUFFER *pCmd, /* IN Payload from the received frame, the union */
                                  /*    should be used to access the fields */
  BYTE   cmdLength)               /* IN Number of command bytes including the command */
{

  BYTE param1 = ((BYTE_P)pCmd)[OFFSET_PARAM_1];
  BYTE param2 = ((BYTE_P)pCmd)[OFFSET_PARAM_2];
  BYTE param3 = ((BYTE_P)pCmd)[OFFSET_PARAM_3];
  BYTE param4 = ((BYTE_P)pCmd)[OFFSET_PARAM_4];
  BYTE param5 = ((BYTE_P)pCmd)[OFFSET_PARAM_5];
  BYTE param6 = ((BYTE_P)pCmd)[OFFSET_PARAM_6];
  BYTE txOption = ((rxStatus & RECEIVE_STATUS_LOW_POWER) ?
               TRANSMIT_OPTION_LOW_POWER : 0) | TRANSMIT_OPTION_ACK;

  /* We were about to sleep, prevent it */
  if(currentState == STATE_APPL_IDLE)
  {
    currentState = STATE_COMMAND_HANDLER;
    StartWatchdog();
  }
  ZW_DEBUG_SEND_STRING("Receive RF Packet\n");

  switch(pCmd->ZW_Common.cmdClass)
  {
    case COMMAND_CLASS_VERSION:
    {
      if (pCmd->ZW_Common.cmd == VERSION_GET)
      {
        //ZW_DEBUG_SEND_BYTE('V');
        //ZW_DEBUG_SEND_BYTE('0');
				ZW_DEBUG_SEND_STRING("Receive VERSION_GET\n");
        txBuf.ZW_VersionReportFrame.cmdClass = COMMAND_CLASS_VERSION;
        txBuf.ZW_VersionReportFrame.cmd = VERSION_REPORT;
        txBuf.ZW_VersionReportFrame.zWaveLibraryType = ZW_TYPE_LIBRARY();
        txBuf.ZW_VersionReportFrame.zWaveProtocolVersion = ZW_VERSION_MAJOR;
        txBuf.ZW_VersionReportFrame.zWaveProtocolSubVersion = ZW_VERSION_MINOR;
        txBuf.ZW_VersionReportFrame.applicationVersion = APP_VERSION;
        txBuf.ZW_VersionReportFrame.applicationSubVersion = APP_REVISION;
        Transport_SendReport(sourceNode, (BYTE *)&txBuf, sizeof(txBuf.ZW_VersionReportFrame),
                   ((rxStatus & RECEIVE_STATUS_LOW_POWER) ? TRANSMIT_OPTION_LOW_POWER : 0)
                   | TRANSMIT_OPTION_ACK | TRANSMIT_OPTION_EXPLORE, NULL, FALSE);
      }
      else if (pCmd->ZW_Common.cmd == VERSION_COMMAND_CLASS_GET)
      {
        //ZW_DEBUG_SEND_BYTE('V');
        //ZW_DEBUG_SEND_BYTE('1');
				ZW_DEBUG_SEND_STRING("Receive VERSION_COMMAND_CLASS_GET\n");
        txBuf.ZW_VersionCommandClassReportFrame.cmdClass = COMMAND_CLASS_VERSION;
        txBuf.ZW_VersionCommandClassReportFrame.cmd = VERSION_COMMAND_CLASS_REPORT;
        txBuf.ZW_VersionCommandClassReportFrame.requestedCommandClass = *((BYTE*)pCmd + OFFSET_PARAM_1);
        ZW_DEBUG_SEND_BYTE('V');
        if (*((BYTE*)pCmd + OFFSET_PARAM_1) == COMMAND_CLASS_BASIC)
        {
          ZW_DEBUG_SEND_BYTE('2');
          txBuf.ZW_VersionCommandClassReportFrame.commandClassVersion = BASIC_VERSION;
        }
        else if (*((BYTE*)pCmd + OFFSET_PARAM_1) == COMMAND_CLASS_SENSOR_BINARY)
        {
          ZW_DEBUG_SEND_BYTE('3');
          txBuf.ZW_VersionCommandClassReportFrame.commandClassVersion = SWITCH_BINARY_VERSION;
        }
        else if (*((BYTE*)pCmd + OFFSET_PARAM_1) == COMMAND_CLASS_CONFIGURATION)
        {
          ZW_DEBUG_SEND_BYTE('4');
          txBuf.ZW_VersionCommandClassReportFrame.commandClassVersion = CONFIGURATION_VERSION;
        }
        else if (*((BYTE*)pCmd + OFFSET_PARAM_1) == COMMAND_CLASS_WAKE_UP)
        {
          ZW_DEBUG_SEND_BYTE('5');
          txBuf.ZW_VersionCommandClassReportFrame.commandClassVersion = WAKE_UP_VERSION;
        }
        else if (*((BYTE*)pCmd + OFFSET_PARAM_1) == COMMAND_CLASS_MANUFACTURER_SPECIFIC)
        {
          ZW_DEBUG_SEND_BYTE('6');
          txBuf.ZW_VersionCommandClassReportFrame.commandClassVersion = MANUFACTURER_SPECIFIC_VERSION;
        }
        else if (*((BYTE*)pCmd + OFFSET_PARAM_1) == COMMAND_CLASS_VERSION)
        {
          ZW_DEBUG_SEND_BYTE('7');
          txBuf.ZW_VersionCommandClassReportFrame.commandClassVersion = VERSION_VERSION;
        }
        else
        {
          ZW_DEBUG_SEND_BYTE('8');
          txBuf.ZW_VersionCommandClassReportFrame.commandClassVersion = UNKNOWN_VERSION;
        }
        Transport_SendReport(sourceNode, (BYTE *)&txBuf, sizeof(txBuf.ZW_VersionCommandClassReportFrame),
                    ((rxStatus & RECEIVE_STATUS_LOW_POWER) ? TRANSMIT_OPTION_LOW_POWER : 0)
                      | TRANSMIT_OPTION_ACK | TRANSMIT_OPTION_EXPLORE, NULL, FALSE);
      }
    }break;
    case COMMAND_CLASS_MANUFACTURER_SPECIFIC:
    {
      if (pCmd->ZW_Common.cmd == MANUFACTURER_SPECIFIC_GET)
      {
				ZW_DEBUG_SEND_STRING("MANUFACTURER_SPECIFIC_GET");
        txBuf.ZW_ManufacturerSpecificReportFrame.cmdClass = COMMAND_CLASS_MANUFACTURER_SPECIFIC;
        txBuf.ZW_ManufacturerSpecificReportFrame.cmd = MANUFACTURER_SPECIFIC_REPORT;
        txBuf.ZW_ManufacturerSpecificReportFrame.manufacturerId1 = 0x00; /* Zensys manufacturer ID */
        txBuf.ZW_ManufacturerSpecificReportFrame.manufacturerId2 = 0x00;
        txBuf.ZW_ManufacturerSpecificReportFrame.productTypeId1 = 0x00; /* Assigned by manufacturer */
        txBuf.ZW_ManufacturerSpecificReportFrame.productTypeId2 = 0x00;
        txBuf.ZW_ManufacturerSpecificReportFrame.productId1 = 0x00; /* Assigned by manufacturer */
        txBuf.ZW_ManufacturerSpecificReportFrame.productId2 = 0x00;
        Transport_SendReport(sourceNode, (BYTE *)&txBuf, sizeof(txBuf.ZW_ManufacturerSpecificReportFrame),
                    ((rxStatus & RECEIVE_STATUS_LOW_POWER) ? TRANSMIT_OPTION_LOW_POWER : 0)
                    | TRANSMIT_OPTION_ACK | TRANSMIT_OPTION_EXPLORE, NULL, FALSE);
      }
      else
      {
        ZW_DEBUG_SEND_BYTE('M');
        ZW_DEBUG_SEND_BYTE('1');
      }
    }break;
  }
  switch(pCmd->ZW_Common.cmdClass)
  {
    case COMMAND_CLASS_SENSOR_BINARY:
    case COMMAND_CLASS_BASIC:
    {
      if (pCmd->ZW_Common.cmd == BASIC_GET){
          ZW_DEBUG_SEND_BYTE('B');
          ZW_DEBUG_SEND_BYTE('g');

          /* Controller wants the sensor level */
          txBuf.ZW_BasicReportFrame.cmdClass = pCmd->ZW_Common.cmdClass;
          txBuf.ZW_BasicReportFrame.cmd = BASIC_REPORT;
          txBuf.ZW_BasicReportFrame.value =  SENSOR_ACTIVATED();
          Transport_SendReport(sourceNode, (BYTE *)&txBuf, sizeof(ZW_BASIC_REPORT_FRAME),
            ((rxStatus & RECEIVE_STATUS_LOW_POWER) ? TRANSMIT_OPTION_LOW_POWER : 0) | TRANSMIT_OPTION_ACK, NULL, FALSE);

        }
    }break;
    case COMMAND_CLASS_WAKE_UP:
    {
    }break;

    case COMMAND_CLASS_ASSOCIATION:
    {
        switch (pCmd->ZW_Common.cmd)
        {
          case ASSOCIATION_GET:
            AssociationSendReport(param1, sourceNode, txOption);
						ZW_DEBUG_SEND_STRING("Association Command Class:Get\n");
            break;

          case ASSOCIATION_SET:
						ZW_DEBUG_SEND_STRING("Association Command Class:Set\n");
#if MAX_ASSOCIATION_GROUPS > 1
            AssociationAdd(param1, (BYTE*)pCmd + OFFSET_PARAM_2, cmdLength - OFFSET_PARAM_2);
#else
            /* If node only support one group. Ignore the group ID requested */
            /* (As defined in Device class specification */
            AssociationAdd(1, (BYTE*)pCmd + OFFSET_PARAM_2, cmdLength - OFFSET_PARAM_2);
#endif
      AssociationStoreAll();
            break;

          case ASSOCIATION_REMOVE:
#if MAX_ASSOCIATION_GROUPS > 1
            AssociationRemove(param1, (BYTE*)pCmd + OFFSET_PARAM_2, cmdLength - OFFSET_PARAM_2);
#else
            /*If node only support one group. Ignore the group ID requested (As defined in Device class specification*/
            AssociationRemove(1, (BYTE*)pCmd + OFFSET_PARAM_2, cmdLength - OFFSET_PARAM_2);
						ZW_DEBUG_SEND_STRING("Association Command Class:Remove\n");
#endif
            break;

          case ASSOCIATION_GROUPINGS_GET:
            AssociationSendGroupings(sourceNode, txOption);
						ZW_DEBUG_SEND_STRING("Association Command Class:Group Get\n");
            break;
        }

    }break;
  }

  if(currentState == STATE_COMMAND_HANDLER)
  {
    currentState = STATE_APPL_IDLE;
  }
}




/*==========================   ApplicationSlaveUpdate   =======================
**   Inform a slave application that a node information is received.
**   Called from the slave command  when a node information frame
**   is received and the Z-Wave protocol is not in a state where it is needed.
**
**--------------------------------------------------------------------------*/
void
ApplicationSlaveUpdate(
  BYTE bStatus,     /*IN  Status event */
  BYTE bNodeID,     /*IN  Node id of the node that send node info */
  BYTE* pCmd,       /*IN  Pointer to Application Node information */
  BYTE bLen)       /*IN  Node info length                        */
{
}


/*======================   ApplicationNodeInformation   =====================
**    Request Node information and current status
**    Called by the the Z-Wave application layer before transmitting a
**    "Node Information" frame.
**
**    This is an application function example
**
**--------------------------------------------------------------------------*/
extern  void                  /*RET  Nothing */
ApplicationNodeInformation(
  BYTE   *deviceOptionsMask,      /*OUT Bitmask with application options     */
  APPL_NODE_TYPE  *nodeType,  /*OUT  Device type Generic and Specific   */
  BYTE       **nodeParm,      /*OUT  Device parameter buffer pointer    */
  BYTE       *parmLength)     /*OUT  Number of Device parameter bytes   */
{
  /* this is a NON listening node and it supports optional CommandClasses*/
  *deviceOptionsMask = APPLICATION_NODEINFO_LISTENING|APPLICATION_NODEINFO_OPTIONAL_FUNCTIONALITY;
  nodeType->generic = GENERIC_TYPE_SENSOR_BINARY; /* Generic Device Type */
  nodeType->specific = SPECIFIC_TYPE_ROUTING_SENSOR_BINARY; /* Specific Device Type */
  *nodeParm = (BYTE *)&nodeInfo;        /* Send list of known command classes. */
  *parmLength = sizeof(nodeInfo);       /* Set length*/

}




/*===============================User Defined Functions=======================*/

 void PrintDebug(BYTE lastState, const char *s){
		if(print_state != lastState){
			print_state = lastState;
			ZW_DEBUG_SEND_STRING(s);
		
		}
 
 }



