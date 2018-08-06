/*******************************  ZW_TRANSPORT_API.H  ***********************
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
 * Description: Z-Wave Transport Application layer interface
 *
 * Author:   Ivar Jeppesen
 *
 * Last Changed By:  $Author: jfr $
 * Revision:         $Revision: 25278 $
 * Last Changed:     $Date: 2013-04-05 16:36:40 +0200 (fr, 05 apr 2013) $
 *
 ****************************************************************************/
#ifndef _ZW_TRANSPORT_API_H_
#define _ZW_TRANSPORT_API_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/

/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/

/* Max number of nodes in a Z-wave system */
#define ZW_MAX_NODES        232

/************************************************************/
/* Node Information frame*/
/************************************************************/
#define NODEPARM_MAX  20   /* max. number of parameters */


/* Transmit frame option flags */
#define TRANSMIT_OPTION_ACK           0x01    /* request acknowledge from destination node */
#define TRANSMIT_OPTION_LOW_POWER     0x02    /* transmit at low output power level (1/3 of normal RF range)*/
#define TRANSMIT_OPTION_AUTO_ROUTE    0x04    /* request retransmission via repeater nodes */
#ifdef ZW_SLAVE_ROUTING
/* Obsoleeted - use TRANSMIT_OPTION_AUTO_ROUTE */
#define TRANSMIT_OPTION_RETURN_ROUTE  TRANSMIT_OPTION_AUTO_ROUTE  /* request transmission via return route */
#endif
/* do not use response route - Even if available */
#define TRANSMIT_OPTION_NO_ROUTE      0x10
/* Use explore frame if needed */
#define TRANSMIT_OPTION_EXPLORE       0x20

/* Transmit frame option flag which are valid when sending explore frames  */
#define TRANSMIT_EXPLORE_OPTION_ACK         TRANSMIT_OPTION_ACK
#define TRANSMIT_EXPLORE_OPTION_LOW_POWER   TRANSMIT_OPTION_LOW_POWER

/* speedoptions for ZW_SendData */
#define TRANSMIT_SPEED_OPTION_BAUD_AUTO         0x00
#define TRANSMIT_SPEED_OPTION_BAUD_9600         0x01
#define TRANSMIT_SPEED_OPTION_BAUD_40000        0x02
#define TRANSMIT_SPEED_OPTION_BAUD_RESERVED     0x04
#define TRANSMIT_SPEED_OPTION_BEAM_AUTO         0x08
#define TRANSMIT_SPEED_OPTION_BEAM_250MS        0x08
#define TRANSMIT_SPEED_OPTION_BEAM_1000MS       0x10
#define TRANSMIT_SPEED_OPTION_BEAM_RESERVED     0x18

/* Received frame status flags */
#define RECEIVE_STATUS_ROUTED_BUSY    0x01
#define RECEIVE_STATUS_LOW_POWER      0x02  /* received at low output power level, this must */
                                            /* have the same value as TRANSMIT_OPTION_LOW_POWER */
#define RECEIVE_STATUS_TYPE_MASK      0x0C  /* Mask for masking out the received frametype bits */
#define RECEIVE_STATUS_TYPE_SINGLE    0x00  /* Received frame is singlecast frame (rxOptions == xxxx00xx) */
#define RECEIVE_STATUS_TYPE_BROAD     0x04  /* Received frame is broadcast frame  (rxOptions == xxxx01xx) */
#define RECEIVE_STATUS_TYPE_MULTI     0x08  /* Received frame is multiecast frame (rxOptions == xxxx10xx) */
/* Only TYPE_BROAD can be active at the same time as TYPE_EXPLORE */
#define RECEIVE_STATUS_TYPE_EXPLORE   0x10  /* Received frame is an explore frame (rxOptions == xxx1xxxx) */
#define RECEIVE_STATUS_FOREIGN_FRAME  0x40  /* Received frame is not send to me (useful only in promiscuous mode) */

/* Predefined Node ID's */
#define NODE_BROADCAST              0xFF    /* broadcast */
#define ZW_TEST_NOT_A_NODEID        0x00    /* */

/* Transmit complete codes */
#define TRANSMIT_COMPLETE_OK          0x00
#define TRANSMIT_COMPLETE_NO_ACK      0x01  /* retransmission error */
#define TRANSMIT_COMPLETE_FAIL        0x02  /* transmit error */
#define TRANSMIT_ROUTING_NOT_IDLE     0x03  /* transmit error */
#ifdef ZW_CONTROLLER
/* Assign route transmit complete but no routes was found */
#define TRANSMIT_COMPLETE_NOROUTE     0x04  /* no route found in assignroute */
                                            /* therefore nothing was transmitted */
#endif  /* ZW_CONTROLLER */
#define TRANSMIT_COMPLETE_HOP_0_FAIL  0x05
#define TRANSMIT_COMPLETE_HOP_1_FAIL  0x06
#define TRANSMIT_COMPLETE_HOP_2_FAIL  0x07
#define TRANSMIT_COMPLETE_HOP_3_FAIL  0x08
#define TRANSMIT_COMPLETE_HOP_4_FAIL  0x09

/* ZW_REDISCOVERY_NEEDED callback values. */
/* Note that they are different from ZW_REQUEST_NETWORK_UPDATE callbacks */
#define ZW_ROUTE_LOST_FAILED      0x04  /* Node Asked wont help us */
#define ZW_ROUTE_LOST_ACCEPT      0x05  /* Accepted to help */


/* Max hops in route */
#define TRANSMIT_ROUTED_ATTEMPT   0x08

#if defined(ZW_SLAVE_ROUTING) || defined(ZW_CONTROLLER)
#ifdef ZW_SLAVE_RETURNROUTEDEST_232
#define ZW_MAX_RETURN_ROUTE_DESTINATIONS   ZW_MAX_NODES
#else
#define ZW_MAX_RETURN_ROUTE_DESTINATIONS   5
#endif
#endif


#ifdef ZW_SLAVE_ROUTING
/*============================   ZW_RouteDestinations   ======================
**    Function description
**      This contains a list of nodes that currently can be reached via
**      return routes.
**      This list should not be altered by Applications.s
**
**--------------------------------------------------------------------------*/
#ifdef ZW_SLAVE_RETURNROUTEDEST_232
/* Libraries with this define can potentially have routes for every node in the network. */
/* We only keep a look on the 5 Return Route destination which was delivered from controller. */
extern IBYTE ZW_RouteDestinations[5];
#else
extern IBYTE ZW_RouteDestinations[ZW_MAX_RETURN_ROUTE_DESTINATIONS];
#endif
#endif


#if (!defined(ZW_SLAVE_ROUTING) && !defined(ZW_CONTROLLER)) || defined(ZW_SLAVE_SENSOR)
/*==============================   ZW_SendData   ============================
**    Transmit data buffer to a single ZW-node or all ZW-nodes (broadcast).
**
**
**    txOptions:
**          TRANSMIT_OPTION_LOW_POWER   transmit at low output power level (1/3 of
**                                      normal RF range).
**          TRANSMIT_OPTION_ACK         the multicast frame will be followed by a
**                                      singlecast frame to each of the destination nodes
**                                      and request acknowledge from each destination node.
**          TRANSMIT_OPTION_AUTO_ROUTE  request retransmission on singlecast frames
**                                      via repeater nodes (at normal output power level).
**          TRANSMIT_OPTION_EXPLORE     Use explore frame route resolution if all else fails
**
** extern BYTE             RET  FALSE if transmitter queue overflow
** ZW_SendData(
**   BYTE  nodeID,          IN  Destination node ID (0xFF == broadcast)
**   BYTE *pData,           IN  Data buffer pointer
**   BYTE  dataLength,      IN  Data buffer length
**   BYTE  txOptions,       IN  Transmit option flags
**   void (*completedFunc)( IN  Transmit completed call back function
**          BYTE txStatus));    IN Transmit status
**--------------------------------------------------------------------------*/
#define ZW_SEND_DATA(node,data,length,options,func) ZW_SendData(node,data,length,options,func)

#else

#ifndef ZW_CONTROLLER_BRIDGE
#ifdef ZW_ROUTING_CONTROL
/*=============================   ZW_SendData_Generic   ========================
**    Transmit data buffer to a single ZW-node or all ZW-nodes (broadcast).
**
**
**    txOptions:
**          TRANSMIT_OPTION_LOW_POWER   transmit at low output power level (1/3 of
**                                      normal RF range).
**          TRANSMIT_OPTION_ACK         The destination is asked to acknowledge the reception
**                                      of the frame. A maximum of 2 retries are done if no
**                                      ACK received.
**          TRANSMIT_OPTION_AUTO_ROUTE  Request retransmission on singlecast frames
**                                      via repeater nodes (at normal output power level).
**          TRANSMIT_OPTION_EXPLORE     Use explore frame route resolution if all else fails
**
** extern BYTE             RET  FALSE if transmitter queue overflow
** ZW_SendData_Generic(
**   BYTE  nodeID,          IN  Destination node ID (0xFF == broadcast)
**   BYTE *pData,           IN  Data buffer pointer
**   BYTE  dataLength,      IN  Data buffer length
**   BYTE  txOptions,       IN  Transmit option flags
**   BYTE *proute           IN  ptr to Route[4] to use. Ignore if NULL. Unused hops must be 0
**   void (*completedFunc)( IN  Transmit completed call back function
**          BYTE txStatus));    IN Transmit status
**--------------------------------------------------------------------------*/
/* For backward compilability */
#define ZW_SendData(node,data,length,options,func) ZW_SendData_Generic(node,data,length,options,NULL,func)
#define ZW_SEND_DATA(node,data,length,options,func) ZW_SendData_Generic(node,data,length,options,NULL,func)
/* Use this */
#define ZW_SEND_DATA_GENERIC(node,data,length,options,proute,func) ZW_SendData_Generic(node,data,length,options,proute,func)
#else
/*==============================   ZW_SendData   ============================
**    Transmit data buffer to a single ZW-node or all ZW-nodes (broadcast).
**
**
**    txOptions:
**          TRANSMIT_OPTION_LOW_POWER   transmit at low output power level (1/3 of
**                                      normal RF range).
**          TRANSMIT_OPTION_ACK         the multicast frame will be followed by a
**                                      singlecast frame to each of the destination nodes
**                                      and request acknowledge from each destination node.
**          TRANSMIT_OPTION_AUTO_ROUTE  request retransmission on singlecast frames
**                                      via repeater nodes (at normal output power level).
**          TRANSMIT_OPTION_EXPLORE     Use explore frame route resolution if all else fails
**
** extern BYTE             RET  FALSE if transmitter queue overflow
** ZW_SendData(
**   BYTE  nodeID,          IN  Destination node ID (0xFF == broadcast)
**   BYTE *pData,           IN  Data buffer pointer
**   BYTE  dataLength,      IN  Data buffer length
**   BYTE  txOptions,       IN  Transmit option flags
**   void (*completedFunc)( IN  Transmit completed call back function
**          BYTE txStatus));    IN Transmit status
**--------------------------------------------------------------------------*/
#define ZW_SEND_DATA(node,data,length,options,func) ZW_SendData(node,data,length,options,func)
#endif  /* ZW_ROUTING_CONTROL */
#endif  /* !ZW_CONTROLLER_BRIDGE */


#ifdef ZW_CONTROLLER_BRIDGE
/*===============================   ZW_SendData_Bridge   =======================
**    Transmit data buffer to a single ZW-node or all ZW-nodes (broadcast).
**
**
**    txOptions:
**          TRANSMIT_OPTION_LOW_POWER   transmit at low output power level (1/3 of
**                                      normal RF range).
**          TRANSMIT_OPTION_ACK         The destination is asked to acknowledge the reception
**                                      of the frame. A maximum of 2 retries are done if no
**                                      ACK received.
**          TRANSMIT_OPTION_AUTO_ROUTE  request retransmission on singlecast frames
**                                      via repeater nodes (at normal output power level).
**          TRANSMIT_OPTION_EXPLORE     Use explore frame route resolution if all else fails
**
** extern BYTE             RET  FALSE if transmitter queue overflow
** ZW_SendData_Bridge(
**   BYTE  bSrcNodeID,      IN  Source node ID
**   BYTE  nodeID,          IN  Destination node ID (0xFF == broadcast)
**   BYTE *pData,           IN  Data buffer pointer
**   BYTE  dataLength,      IN  Data buffer length
**   BYTE  txOptions,       IN  Transmit option flags
**   BYTE *proute           IN  ptr to Route[4] to use. Ignore if NULL. Unused hops must be 0
** void (*completedFunc)(   IN  Transmit completed call back function
**        BYTE txStatus));    IN Transmit status
**--------------------------------------------------------------------------*/
/* For backward compilability */
#define ZW_SendData(node,data,length,options,func) ZW_SendData_Bridge(0xff,node,data,length,options,NULL,func)
#define ZW_SEND_DATA(node,data,length,options,func) ZW_SendData_Bridge(0xff,node,data,length,options,NULL,func)
/* Use this */
#define ZW_SEND_DATA_BRIDGE(srcnode,node,data,length,options,proute,func) ZW_SendData_Bridge(srcnode,node,data,length,options,proute,func)
#endif


#endif /* (!ZW_SLAVE_ROUTING && !ZW_CONTROLLER) || ZW_SLAVE_SENSOR */


#if defined(ZW_CONTROLLER) || defined(ZW_SLAVE_ROUTING)
/*============================   ZW_SendDataAbort   ========================
**    Abort the ongoing transmit started with ZW_SendData()
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
#define ZW_SEND_DATA_ABORT() ZW_SendDataAbort()


#ifndef ZW_CONTROLLER_BRIDGE
/*=============================   ZW_SendDataMulti   ========================
**    Transmit data buffer to a list of Z-Wave Nodes (multicast frame).
**
**
**    txOptions:
**          TRANSMIT_OPTION_LOW_POWER   transmit at low output power level (1/3 of
**                                      normal RF range).
**          TRANSMIT_OPTION_ACK         the multicast frame will be followed by a
**                                      singlecast frame to each of the destination nodes
**                                      and request acknowledge from each destination node.
**          TRANSMIT_OPTION_AUTO_ROUTE  request retransmission on singlecast frames
**                                      via repeater nodes (at normal output power level).
**          TRANSMIT_OPTION_EXPLORE     Use explore frame route resolution if all else fails
**
** extern BYTE            RET  FALSE if transmitter queue overflow
** ZW_SendDataMulti(
**  BYTE *pNodeIDList,     IN  List of destination node ID's
**  BYTE  numberNodes,     IN  Number of Nodes
**  BYTE *pData,           IN  Data buffer pointer
**  BYTE  dataLength,      IN  Data buffer length
**  BYTE  txOptions,       IN  Transmit option flags
**  VOID_CALLBACKFUNC(completedFunc)( IN  Transmit completed call back function
**    BYTE txStatus)); IN Transmit status
**--------------------------------------------------------------------------*/
#define ZW_SEND_DATA_MULTI(nodelist,data,length,options,func) ZW_SendDataMulti(nodelist,data,length,options,func)
#endif


#ifdef ZW_CONTROLLER_BRIDGE
/*=============================   ZW_SendDataMulti_Bridge   ====================
**    Transmit data buffer to a list of Z-Wave Nodes (multicast frame).
**
**
**    txOptions:
**          TRANSMIT_OPTION_LOW_POWER   transmit at low output power level (1/3 of
**                                      normal RF range).
**          TRANSMIT_OPTION_ACK         the multicast frame will be followed by a
**                                      singlecast frame to each of the destination nodes
**                                      and request acknowledge from each destination node.
**          TRANSMIT_OPTION_AUTO_ROUTE  request retransmission on singlecast frames
**                                      via repeater nodes (at normal output power level).
**          TRANSMIT_OPTION_EXPLORE     Use explore frame route resolution if all else fails
**
** extern BYTE            RET  FALSE if transmitter queue overflow
** ZW_SendDataMulti_Bridge(
**  BYTE bSrcNodeID,       IN  Source NodeID - if 0xFF then controller ID is set as source
**  BYTE *pNodeIDList,     IN  List of destination node ID's
**  BYTE  numberNodes,     IN  Number of Nodes
**  BYTE *pData,           IN  Data buffer pointer
**  BYTE  dataLength,      IN  Data buffer length
**  BYTE  txOptions,       IN  Transmit option flags
**  VOID_CALLBACKFUNC(completedFunc)( IN  Transmit completed call back function
**    BYTE txStatus)); IN Transmit status
**--------------------------------------------------------------------------*/
#define ZW_SendDataMulti(nodelist,data,length,options,func) ZW_SendDataMulti_Bridge(0xff,nodelist,data,length,options,func)
#define ZW_SEND_DATA_MULTI(nodelist,data,length,options,func) ZW_SendDataMulti_Bridge(0xff,nodelist,data,length,options,func)
#define ZW_SEND_DATA_MULTI_BRIDGE(srcId,nodelist,data,length,options,func) ZW_SendDataMulti_Bridge(srcId,nodelist,data,length,options,func)
#endif

#endif  /* defined(ZW_CONTROLLER) || defined(ZW_SLAVE_ROUTING) */

/*============================   ZW_SEND_CONST =============================
**    Function description
**      If production test is enabled during start up.
**      Calling this function will transmit a constant signal until a new
**      RF function is called
**
**--------------------------------------------------------------------------*/
#define ZW_SEND_CONST() ZW_SendConst()


#ifdef ZW_SLAVE
/*============================ ZW_LOCK_RESPONSE_ROUTE ========================
**    Function description
**      This function locks and unlocks response route for specified node
**      IN  nodeID != 0x00 lock route to node
**          nodeID == 0x00 unlock entry
**    Side effects:
**
**--------------------------------------------------------------------------*/
#define ZW_LOCK_RESPONSE_ROUTE(node) ZW_LockRoute(node)

#else

/*============================ ZW_LOCK_RESPONSE_ROUTE ========================
**    Function description
**      This function locks and unlocks all Last Working Routes for purging
**      IN  bLockRoute == TRUE lock all Last Working Routes - no purging allowed
**          bLockRoute == FALSE unlock purging of Last Working Routes
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
#define ZW_LOCK_RESPONSE_ROUTE(bLockRoute) ZW_LockRoute(bLockRoute)
#endif


#if defined(ZW020x) || defined(ZW030x)

#if defined(ZW_CONTROLLER) || defined(ZW_SLAVE_ROUTING)

#ifndef ZW_CONTROLLER_BRIDGE
#ifdef ZW_ROUTING_CONTROL
/*===============================   ZW_SEND_DATA_META ==========================
**    Transmit Meta data buffer to a single Z-Wave node or all Z-Wave nodes
**    Only allowed for 40kbit nodes. If a 9600 destination is selected FALSE will
**    be returned. Parameters and return values identical with ZW_SEND_DATA
**    DOES NOT EXIST ON ZW0x0x targets
**    Side effects:
**
**--------------------------------------------------------------------------*/
#define ZW_SendDataMeta(destId,data,len,txOptions,func) ZW_SendDataMeta_Generic(destId,data,len,txOptions,NULL,func)
#define ZW_SEND_DATA_META(destId,data,len,txOptions,func) ZW_SendDataMeta_Generic(destId,data,len,txOptions,NULL,func)
#define ZW_SEND_DATA_META_GENERIC(destId,data,len,txOptions,proute,func) ZW_SendDataMeta_Generic(destId,data,len,txOptions,proute,func)
#else
/*===============================   ZW_SEND_DATA_META ==========================
**    Transmit Meta data buffer to a single Z-Wave node or all Z-Wave nodes
**    Only allowed for 40kbit nodes. If a 9600 destination is selected FALSE will
**    be returned. Parameters and return values identical with ZW_SEND_DATA
**    DOES NOT EXIST ON ZW0x0x targets
**    Side effects:
**
**--------------------------------------------------------------------------*/
#define ZW_SEND_DATA_META(destId,data,len,txOptions,func) ZW_SendDataMeta(destId,data,len,txOptions,func)
#endif  /* ZW_ROUTING_CONTROL */
#endif  /* !ZW_CONTROLLER_BRIDGE */


#ifdef ZW_CONTROLLER_BRIDGE
/*==========================   ZW_SEND_DATA_META_BRIDGE ======================
**    Transmit Meta data buffer to a single Z-Wave node or all Z-Wave nodes
**    Only allowed for 40kbit nodes. If a 9600 destination is selected FALSE will
**    be returned. Parameters and return values identical with ZW_SEND_DATA
**    DOES NOT EXIST ON ZW0x0x targets
**    Side effects:
**
**--------------------------------------------------------------------------*/
#define ZW_SendDataMeta(destId,data,len,txOptions,func) ZW_SendDataMeta_Bridge(0xff,destId,data,len,txOptions,NULL,func)
#define ZW_SEND_DATA_META(destId,data,len,txOptions,func) ZW_SendDataMeta_Bridge(0xff,destId,data,len,txOptions,NULL,func)
#define ZW_SEND_DATA_META_BRIDGE(srcId,destId,data,len,txOptions,proute,func) ZW_SendDataMeta_Bridge(srcId,destId,data,len,txOptions,proute,func)
#endif  /* ZW_CONTROLLER_BRIDGE */

#endif  /* defined(ZW_CONTROLLER) || defined(ZW_SLAVE_ROUTING) */

#endif /* defined(ZW020x) || defined(ZW030x) */


/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/

#ifdef ZW_SLAVE
/* TO#2133 fix - Keil compiler >7.50(8.xx) seems to have been changed somehow */
/* in the preprocessor part as if the 2 ZW_LockRoute definitions was ifdefed */
/* by ifdef ZW_SLAVE ... endif and ifdef ZW_CONTROLLER ... endif instead of */
/* ifdef ZW_SLAVE ... else ... endif, the Keil >7.50 reports Warning C235 */
/* on slave/slave routing/slave enhanced targets. */
/*============================   ZW_LockRoute   ==============================
**    Function description
**      This function locks and unlocks any temporary route to a specific nodeID
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
ZW_LockRoute(
  BYTE bNodeID);          /* IN if nonezero lock bNodeID entry, */
                          /*    zero unlock entry */

#else  /* ZW_SLAVE */

/*=============================   ZW_LockRoute   ============================
**    Function description
**      IF bLockRoute TRUE then any attempt to purge a LastWorkingRoute entry
**      is denied.
**
**    Side effects:
**
**
**--------------------------------------------------------------------------*/
void
ZW_LockRoute(
  BOOL bLockRoute);       /* IN TRUE lock LastWorkingRoute entry purging */
                          /*    FALSE unlock LastWorkingRoute entry purging */
#endif  /* ZW_SLAVE */


/*============================   ZW_SendConst  =============================
**    Function description
**      If production test is enabled it will transmit a constant signal
**      until a new RF function is called
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
ZW_SendConst(void);


#if defined(ZW_CONTROLLER) || defined(ZW_SLAVE_ROUTING)

#ifndef ZW_CONTROLLER_BRIDGE
#ifdef ZW_ROUTING_CONTROL
/*=============================   ZW_SendData_Generic   =========================
**    Transmit data buffer to a single ZW-node or all ZW-nodes (broadcast).
**
**
**    txOptions:
**          TRANSMIT_OPTION_LOW_POWER     transmit at low output power level
**                                        (1/3 of normal RF range).
**          TRANSMIT_OPTION_ACK           the multicast frame will be followed
**                                        by a  singlecast frame to each of
**                                        the destination nodes
**                                        and request acknowledge from each
**                                        destination node.
**          TRANSMIT_OPTION_AUTO_ROUTE    request retransmission via repeater
**                                        nodes at normal output power level).
**          TRANSMIT_OPTION_RETURN_ROUTE  transmission via return route, if
**                                        no return route is registred then
**                                        the frame will be send as a
**                                        broadcast.
**          TRANSMIT_OPTION_EXPLORE     Use explore frame route resolution if all else fails
**
** extern BYTE            RET FALSE if transmitter queue overflow
** ZW_SendData_Generic(
**  BYTE  nodeID,          IN Destination node ID (0xFF == broadcast)
**  BYTE *pData,           IN Data buffer pointer
**  BYTE  dataLength,      IN Data buffer length
**  BYTE  txOptions,       IN Transmit option flags
**  BYTE_P proute          IN Route to use. Ignore if NULL should be terminated unused hops should be 0
**  void (*completedFunc)( IN  Transmit completed call back function
**         BYTE txStatus));    IN Transmit status
**--------------------------------------------------------------------------*/
extern BYTE                   /*RET  FALSE if transmitter busy      */
ZW_SendData_Generic(
  BYTE  nodeID,               /*IN  Destination node ID (0xFF == broadcast) */
  BYTE *pData,                /*IN  Data buffer pointer           */
  BYTE  dataLength,           /*IN  Data buffer length            */
  BYTE  txOptions,            /*IN  Transmit option flags         */
  BYTE_P pRoute,  /* IN Route to use. Ignore if NULL should be terminated unused hops should be 0 */
  VOID_CALLBACKFUNC(completedFunc)(BYTE)); /*IN  Transmit completed call back function  */
#else
/*==================================   ZW_SendData ==========================
**    Transmit Meta data buffer to a single Z-Wave node or all Z-Wave nodes
**    Only allowed for 40kbit nodes. If a 9600 destination is selected FALSE will
**    be returned. Parameters and return values identical with ZW_SEND_DATA
**    DOES NOT EXIST ON ZW0x0x targets
**    Side effects:
**
**--------------------------------------------------------------------------*/
BYTE
ZW_SendData(
  BYTE  destNodeID, /* IN Destination node ID - 0xFF == all nodes */
  BYTE *pData,      /* IN Data buffer pointer           */
  BYTE  dataLength, /* IN Data buffer length            */
  BYTE  txOptions,  /* IN Transmit option flags         */
  VOID_CALLBACKFUNC(completedFunc)(BYTE)); /* IN Transmit completed call back function  */
#endif  /* ZW_ROUTING_CONTROL */
#endif  /* !ZW_CONTROLLER_BRIDGE */


#ifdef ZW_CONTROLLER_BRIDGE
/*============================   ZW_SendData_Bridge   ========================
**    Transmit data buffer to a single ZW-node or all ZW-nodes (broadcast).
**
**
**    txOptions:
**          TRANSMIT_OPTION_LOW_POWER     transmit at low output power level
**                                        (1/3 of normal RF range).
**          TRANSMIT_OPTION_ACK           the multicast frame will be followed
**                                        by a  singlecast frame to each of
**                                        the destination nodes
**                                        and request acknowledge from each
**                                        destination node.
**          TRANSMIT_OPTION_AUTO_ROUTE    request retransmission via repeater
**                                        nodes at normal output power level).
**          TRANSMIT_OPTION_RETURN_ROUTE  transmission via return route, if
**                                        no return route is registred then
**                                        the frame will be send as a
**                                        broadcast.
**
** extern BYTE            RET FALSE if transmitter queue overflow
** ZW_SendData_Bridge(
**  BYTE  bSrcNodeID,      IN Source NodeID - if 0xFF then controller ID is set as source
**  BYTE  nodeID,          IN Destination node ID (0xFF == broadcast)
**  BYTE *pData,           IN Data buffer pointer
**  BYTE  dataLength,      IN Data buffer length
**  BYTE  txOptions,       IN Transmit option flags
**  BYTE_P proute          IN Route to use. Ignore if NULL should be terminated unused hops should be 0
**  void (*completedFunc)( IN Transmit completed call back function
**         BYTE txStatus));    IN Transmit status
**--------------------------------------------------------------------------*/
extern BYTE                   /*RET FALSE if transmitter busy      */
ZW_SendData_Bridge(
  BYTE  bSrcNodeID,           /*IN  Source NodeID - if 0xFF then controller ID is set as source */
  BYTE  nodeID,               /*IN  Destination node ID (0xFF == broadcast) */
  BYTE *pData,                /*IN  Data buffer pointer           */
  BYTE  dataLength,           /*IN  Data buffer length            */
  BYTE  txOptions,            /*IN  Transmit option flags         */
  BYTE_P pRoute,              /* IN Route to use. Ignore if NULL should be terminated unused hops should be 0 */
  VOID_CALLBACKFUNC(completedFunc)(BYTE)); /*IN  Transmit completed call back function  */
#endif  /* ZW_CONTROLLER_BRIDGE */


#ifndef ZW_CONTROLLER_BRIDGE
#ifdef ZW_ROUTING_CONTROL
/*============================   ZW_SendDataMeta_Generic =======================
**    Transmit Meta data buffer to a single Z-Wave node or all Z-Wave nodes
**    Only allowed for 40kbit nodes. If a 9600 destination is selected FALSE will
**    be returned. Parameters and return values identical with ZW_SEND_DATA
**    DOES NOT EXIST ON ZW0x0x targets
**    Side effects:
**
**--------------------------------------------------------------------------*/
BYTE
ZW_SendDataMeta_Generic(
  BYTE  destNodeID, /* IN Destination node ID - 0xFF == all nodes */
  BYTE *pData,      /* IN Data buffer pointer           */
  BYTE  dataLength, /* IN Data buffer length            */
  BYTE  txOptions,  /* IN Transmit option flags         */
  BYTE_P pRoute,    /* IN Route to use. Ignore if NULL should be terminated unused hops should be 0 */
  VOID_CALLBACKFUNC(completedFunc)(BYTE)); /* IN Transmit completed call back function  */
#else
/*============================   ZW_SendDataMeta ============================
**    Transmit Meta data buffer to a single Z-Wave node or all Z-Wave nodes
**    Only allowed for 40kbit nodes. If a 9600 destination is selected FALSE will
**    be returned. Parameters and return values identical with ZW_SEND_DATA
**    DOES NOT EXIST ON ZW0x0x targets
**    Side effects:
**
**--------------------------------------------------------------------------*/
BYTE
ZW_SendDataMeta(
  BYTE  destNodeID, /* IN Destination node ID - 0xFF == all nodes */
  BYTE *pData,      /* IN Data buffer pointer           */
  BYTE  dataLength, /* IN Data buffer length            */
  BYTE  txOptions,  /* IN Transmit option flags         */
  VOID_CALLBACKFUNC(completedFunc)(BYTE)); /* IN Transmit completed call back function  */
#endif  /* ZW_ROUTING_CONTROL */
#endif  /* !ZW_CONTROLLER_BRIDGE */


#ifdef ZW_CONTROLLER_BRIDGE
/*===============================   ZW_SendDataMeta_Bridge ===================
**    Transmit Meta data buffer to a single Z-Wave node or all Z-Wave nodes
**    Only allowed for 40kbit nodes. If a 9600 destination is selected FALSE will
**    be returned. Parameters and return values identical with ZW_SEND_DATA
**    DOES NOT EXIST ON ZW0x0x targets
**    Side effects:
**
**--------------------------------------------------------------------------*/
BYTE
ZW_SendDataMeta_Bridge(
  BYTE  bSrcNodeID, /* IN  Source node ID - if 0xFF then controller ID is source */
  BYTE  destNodeID, /* IN Destination node ID - 0xFF == all nodes */
  BYTE *pData,      /* IN Data buffer pointer           */
  BYTE  dataLength, /* IN Data buffer length            */
  BYTE  txOptions,  /* IN Transmit option flags         */
  BYTE_P pRoute,    /* IN Route to use. Ignore if NULL should be terminated unused hops should be 0 */
  VOID_CALLBACKFUNC(completedFunc)(BYTE)); /* IN Transmit completed call back function  */
#endif

#else /* defined(ZW_CONTROLLER) || defined(ZW_SLAVE_ROUTING) */

/*==================================   ZW_SendData ==========================
**    Transmit Meta data buffer to a single Z-Wave node or all Z-Wave nodes
**    Only allowed for 40kbit nodes. If a 9600 destination is selected FALSE will
**    be returned. Parameters and return values identical with ZW_SEND_DATA
**    DOES NOT EXIST ON ZW0x0x targets
**    Side effects:
**
**--------------------------------------------------------------------------*/
BYTE
ZW_SendData(
  BYTE  destNodeID, /* IN Destination node ID - 0xFF == all nodes */
  BYTE *pData,      /* IN Data buffer pointer           */
  BYTE  dataLength, /* IN Data buffer length            */
  BYTE  txOptions,  /* IN Transmit option flags         */
  VOID_CALLBACKFUNC(completedFunc)(BYTE)); /* IN Transmit completed call back function  */

#endif  /* defined(ZW_CONTROLLER) || defined(ZW_SLAVE_ROUTING) */


#if defined(ZW_CONTROLLER) || defined(ZW_SLAVE_ROUTING)

/*============================   ZW_SendDataAbort   ========================
**    Abort the ongoing transmit started with ZW_SendData()
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
void                /*RET FALSE if transmitter busy      */
ZW_SendDataAbort(void);


#ifndef ZW_CONTROLLER_BRIDGE
/*===============================   ZW_SendDataMulti   ======================
**    Transmit data buffer to a list of Z-Wave Nodes (multicast frame).
**
**
**    txOptions:
**          TRANSMIT_OPTION_LOW_POWER   transmit at low output power level (1/3 of
**                                      normal RF range).
**          TRANSMIT_OPTION_ACK         the multicast frame will be followed by a
**                                      singlecast frame to each of the destination nodes
**                                      and request acknowledge from each destination node.
**          TRANSMIT_OPTION_AUTO_ROUTE  request retransmission on singlecast frames
**                                      via repeater nodes (at normal output power level).
**
**--------------------------------------------------------------------------*/
extern BYTE            /*RET  FALSE if transmitter busy      */
ZW_SendDataMulti(
  BYTE *pNodeIDList,   /*IN  List of destination node ID's */
  BYTE *pData,         /*IN  Data buffer pointer           */
  BYTE  dataLength,    /*IN  Data buffer length            */
  BYTE  txOptions,     /*IN  Transmit option flags         */
  VOID_CALLBACKFUNC(completedFunc)(BYTE)); /*IN  Transmit completed call back function  */
#endif  /* !ZW_CONTROLLER_BRIDGE */


#ifdef ZW_CONTROLLER_BRIDGE
/*===============================   ZW_SendDataMulti_Bridge   ================
**    Transmit data buffer to a list of Z-Wave Nodes (multicast frame).
**
**
**    txOptions:
**          TRANSMIT_OPTION_LOW_POWER   transmit at low output power level (1/3 of
**                                      normal RF range).
**          TRANSMIT_OPTION_ACK         the multicast frame will be followed by a
**                                      singlecast frame to each of the destination nodes
**                                      and request acknowledge from each destination node.
**          TRANSMIT_OPTION_AUTO_ROUTE  request retransmission on singlecast frames
**                                      via repeater nodes (at normal output power level).
**
**--------------------------------------------------------------------------*/
extern BYTE            /*RET  FALSE if transmitter busy      */
ZW_SendDataMulti_Bridge(
  BYTE bSrcNodeID,     /*IN Source nodeID - if 0xFF then controller is set as source */
  BYTE *pNodeIDList,   /*IN List of destination node ID's */
  BYTE *pData,         /*IN Data buffer pointer           */
  BYTE  dataLength,    /*IN Data buffer length            */
  BYTE  txOptions,     /*IN Transmit option flags         */
  VOID_CALLBACKFUNC(completedFunc)(BYTE)); /*IN  Transmit completed call back function  */
#endif /* ZW_CONTROLLER_BRIDGE */

#endif /* defined(ZW_CONTROLLER) || defined(ZW_SLAVE_ROUTING) */


#if defined(ZW_CONTROLLER) && !defined(ZW_CONTROLLER_STATIC)
/*===============================   ZW_GetNodeIDMaskList   ===================
**    Get the recieved multicast frame's destinations nodes mask list
**    Side effects:
**      The API should only be called before returning from ApplicationCommandHandler
**--------------------------------------------------------------------------*/
BYTE                            /*RET: nodeID mask offset*/
ZW_GetNodeIDMaskList(
  BYTE **pNodeIDMaskList,      /* OUT destinations nodes mask list*/
  BYTE  *pMaskListLen);        /* OUT the destinations nodes mask list length */

#endif

#endif /* _ZW_TRANSPORT_API_H_ */

