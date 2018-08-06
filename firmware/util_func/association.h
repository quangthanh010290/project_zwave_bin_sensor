/******************************* association.h *******************************
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
 * Description: Header file for associaction.c. This module implements
 *              functions that makes it easy to add association commandclass
 *              support to applications.
 *
 * Author:   Henrik Holm
 *
 * Last Changed By:  $Author: vvi $
 * Revision:         $Revision: 9752 $
 * Last Changed:     $Date: 2008-01-09 14:42:10 +0100 (on, 09 jan 2008) $
 *
 ****************************************************************************/
#ifndef _ASSOCIATION_H_
#define _ASSOCIATION_H_

/****************************************************************************/
/*                              EXTERNAL DEFINED FUNCTIONS/DATA             */
/****************************************************************************/

/*============================   AssociationStoreAll   ======================
**    Function description
**      Should be implmented by application to Store all group[]
**      in non-volatile memory
**    Side effects:
**
**--------------------------------------------------------------------------*/
void AssociationStoreAll( void );

/*============================   AssociationClearAll   ======================
**    Function description
**      Should be implmented by application to Clear all of group[]
**      in non-volatile memory
**    Side effects:
**
**--------------------------------------------------------------------------*/
void AssociationClearAll( void );

/*============================   AssociationInit   ======================
**    Function description
**      Should be implmented by application to read all groups from
**      non-volatile memory to group[]
**    Side effects:
**
**--------------------------------------------------------------------------*/
void AssociationInit(void);


/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/
/*Support for more groups can be enabled*/
#ifdef ZW_SLAVE
#define MAX_ASSOCIATION_GROUPS      1
#define MAX_ASSOCIATION_IN_GROUP    3
#endif

#define ASSOCIATION_SIZE            (MAX_ASSOCIATION_GROUPS * MAX_ASSOCIATION_IN_GROUP)

typedef struct _ASSOCIATION_GROUP_
{
  BYTE nodeID[MAX_ASSOCIATION_IN_GROUP];
} ASSOCIATION_GROUP;


/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

extern ASSOCIATION_GROUP groups[MAX_ASSOCIATION_GROUPS];

/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/

/*============================   AssociationSendReport   ======================
**    Function description
**      Sends the ASSOCIATION_REPORT for the supplied Group to the node specified.
**      Transmit options are grapped from BYTE txOption
**    Side effects:
**
**--------------------------------------------------------------------------*/
BOOL                                  /*RET true if started false if not*/
AssociationSendReport(BYTE groupNo,   /*IN Group Number to send report for*/
                      BYTE destNode,  /*IN Destination to send to*/
                      BYTE txOption);  /*IN Send Data transtmit options*/

/*========================   AssociationSendGroupings   ======================
**    Function description
**      Sends the number of groupings supported.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
BOOL AssociationSendGroupings(                /*RET TRUE If started, FALSE if not*/
                              BYTE destNode,  /*IN destination to send to*/
                              BYTE txOption);/*IN Send data transmit options*/



/*============================   AssociationAdd   ======================
**    Function description
**      Adds the nodes in nodeIdP to the group
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
AssociationAdd(
  BYTE group,     /*IN group number to add nodes to*/
  BYTE_P nodeIdP, /*IN pointer to list of nodes*/
  BYTE noOfNodes);/*IN number of nodes in List*/


/*============================   AssociationRemove   ======================
**    Function description
**      Removes association members from one or all group
**      If group = 0 nodeIds in nodeIdP list is removed from all groups
**      If noOfNodes = 0 group is removed
**      if both is 0 all groups and nodes is removed
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
AssociationRemove(
  BYTE group,      /*IN group number to remove nodes from*/
  BYTE_P nodeIdP,  /*IN pointer to array of nodes to remove*/
  BYTE noOfNodes); /*IN number of nodes in list*/
#endif /*_ASSOCIATION_H_*/
