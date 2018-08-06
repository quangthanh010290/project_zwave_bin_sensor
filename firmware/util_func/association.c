/******************************* association.c *******************************
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
 * Description: Implements functions that make is easy to support
 *              Association Command Class
 *
 * Author:   Henrik Holm
 *
 * Last Changed By:  $Author: iza $
 * Revision:         $Revision: 22224 $
 * Last Changed:     $Date: 2012-01-19 14:11:58 +0100 (to, 19 jan 2012) $
 *
 ****************************************************************************/
/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <ZW_basis_api.h>
#include <ZW_uart_api.h>
#include <ZW_debug_api.h>
#include <association.h>


#include <ZW_TransportLayer.h>
/* txBuf should be implemented elsewhere. It will be used to send */
/* ASSOCIATION commandclass frames */
extern ZW_APPLICATION_TX_BUFFER txBuf;

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

ASSOCIATION_GROUP groups[MAX_ASSOCIATION_GROUPS];

BYTE groupReportSourceNode;

/*I*/BYTE indx;

BOOL associationReportStarted = FALSE;

void
AssociationSendComplete(
  BYTE bStatus);

//BOOL
//AssociationSend(
//  BOOL doSend);

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/

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
BOOL                    /*RET TRUE if started FALSE if not */
AssociationSendReport(
  BYTE groupNo,         /* IN Group Number to send report for */
  BYTE destNode,        /* IN Destination to send to */
  BYTE txOption)        /* IN Send Data transtmit options */
{
  BYTE nodeCount;
  BYTE bGroupIndex;
  if (!associationReportStarted)
  {
    associationReportStarted = TRUE;
    nodeCount = 0;
    groupReportSourceNode = destNode;
#if MAX_ASSOCIATION_GROUPS > 1
    /*If the node supports several groups. Set the correct group index*/
    if(groupNo!=0)
    {
      bGroupIndex = groupNo-1;
    }
    if(groupNo> MAX_ASSOCIATION_GROUPS)
    {
      bGroupIndex = 0;
    }
#else
  /*If node only support one group. Ignore the group ID requested (As defined in Device class specification*/
    bGroupIndex = 0;
#endif
    txBuf.ZW_AssociationReport1byteFrame.cmdClass           = COMMAND_CLASS_ASSOCIATION;
    txBuf.ZW_AssociationReport1byteFrame.cmd                = ASSOCIATION_REPORT;
    txBuf.ZW_AssociationReport1byteFrame.groupingIdentifier = bGroupIndex+1;
    txBuf.ZW_AssociationReport1byteFrame.maxNodesSupported  = MAX_ASSOCIATION_IN_GROUP;
    txBuf.ZW_AssociationReport1byteFrame.reportsToFollow = 0; //Nodes fit in one report
    for (indx = 0; indx < MAX_ASSOCIATION_IN_GROUP; indx++)
    {
      if (groups[bGroupIndex].nodeID[indx])
      {
        *(&txBuf.ZW_AssociationReport1byteFrame.nodeid1 + nodeCount) = groups[bGroupIndex].nodeID[indx];
        nodeCount++;
      }
    }
	// TO#02827  	Not all steps of association done on secure binary sensor is carried out securely
//    if(!ZW_SEND_DATA(groupReportSourceNode,
//                 (BYTE*)&txBuf,
//                 sizeof(ZW_ASSOCIATION_REPORT_1BYTE_FRAME) - 1 + nodeCount,
//                 txOption,
//                 AssociationSendComplete
//				 ))

    if(!Transport_SendRequest(groupReportSourceNode,
                 		(BYTE*)&txBuf,
                 		sizeof(ZW_ASSOCIATION_REPORT_1BYTE_FRAME) - 1 + nodeCount,
        				txOption,
                 AssociationSendComplete
				 , FALSE
				 ))

    {
      associationReportStarted = FALSE;
    }
    return associationReportStarted;
  }
  else
  {
    return FALSE;
  }
}

/*========================   AssociationSendGroupings   ======================
**    Function description
**      Sends the number of groupings supported.
**
**    Side effects:
**
**--------------------------------------------------------------------------*/
BOOL
AssociationSendGroupings(  /*RET TRUE If started, FALSE if not*/
  BYTE destNode,           /* IN destination to send to*/
  BYTE txOption)           /* IN Send data transmit options*/
{
  if (!associationReportStarted)
  {
    associationReportStarted = TRUE;
    txBuf.ZW_AssociationGroupingsReportFrame.cmdClass = COMMAND_CLASS_ASSOCIATION;
    txBuf.ZW_AssociationGroupingsReportFrame.cmd = ASSOCIATION_GROUPINGS_REPORT;
    txBuf.ZW_AssociationGroupingsReportFrame.supportedGroupings = MAX_ASSOCIATION_GROUPS;
    groupReportSourceNode = destNode;

	// TO#02827  	Not all steps of association done on secure binary sensor is carried out securely
//    if(!ZW_SEND_DATA(groupReportSourceNode, (BYTE*)&txBuf, sizeof(txBuf.ZW_AssociationGroupingsReportFrame), txOption, AssociationSendComplete))
    if(!Transport_SendRequest(groupReportSourceNode, (BYTE*)&txBuf, sizeof(txBuf.ZW_AssociationGroupingsReportFrame), txOption, AssociationSendComplete,FALSE))

    {
      associationReportStarted = FALSE;
    }
  }
  else
  {
    return FALSE;
  }
  return associationReportStarted;
}


void
AssociationSendComplete(
  BYTE bStatus)
{
  associationReportStarted = FALSE; /* Done */
}


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
  BYTE noOfNodes) /*IN number of nodes in List*/
{
  BYTE i;
  BYTE tempID;
  if (group)
  {
    group--;
  }
  //ZW_DEBUG_SEND_BYTE('r');	
  //ZW_DEBUG_SEND_NUM(noOfNodes);
	ZW_DEBUG_SEND_STRING("Association Add, Number of Node:");
	ZW_DEBUG_SEND_NUM(noOfNodes);
	ZW_DEBUG_SEND_STRING("\n");
	
  for (i = 0; i < noOfNodes; i++)
  {
    BYTE vacant = 0xff;
    tempID = *(nodeIdP + i);
    for (indx = 0; indx < MAX_ASSOCIATION_IN_GROUP; indx++)
    {
      if (groups[group].nodeID[indx])
      {
        if (groups[group].nodeID[indx] == tempID)
        {
          break;  /* Allready in */
        }
      }
      else
      {
        if (vacant == 0xff)
        {
          vacant = indx;
        }
      }
    }
    if (vacant != 0xff)
    {
      groups[group].nodeID[vacant] = tempID;
      AssociationStoreAll();
    }
  }
}


/*============================   AssociationRemove   ======================
**    Function description
**      Removes association members from specified group
**      If noOfNodes = 0 group is removed
**    Side effects:
**
**--------------------------------------------------------------------------*/
void
AssociationRemove(
  BYTE group,      /*IN group number to remove nodes from*/
  BYTE_P nodeIdP,  /*IN pointer to array of nodes to remove*/
  BYTE noOfNodes)  /*IN number of nodes in list*/
{
  BYTE i;
  if (group)
  {
    group--;
  }
	/*
		ZW_DEBUG_SEND_BYTE('a');
  ZW_DEBUG_SEND_NUM(noOfNodes);
  ZW_DEBUG_SEND_NUM(group);
	*/
  ZW_DEBUG_SEND_STRING("Association Remove, Number of Node:");	
	ZW_DEBUG_SEND_NUM(noOfNodes);
	ZW_DEBUG_SEND_STRING(", Group: ");
	ZW_DEBUG_SEND_NUM(group);
	ZW_DEBUG_SEND_STRING(" \n");
	
	
  if (noOfNodes)
  {
    for (i = 0; i < noOfNodes; i++)
    {
      for (indx = 0; indx < MAX_ASSOCIATION_IN_GROUP; indx++)
      {
        if (groups[group].nodeID[indx] == *(nodeIdP+i))
        {
          groups[group].nodeID[indx] = 0;
          break;  /* Found */
        }
      }
    }
  }
  else
  {
    /* If no supplied nodeIDs all nodes should be removed from group */
    indx = MAX_ASSOCIATION_IN_GROUP;
    do
    {
      groups[group].nodeID[indx - 1] = 0;
    } while (--indx);
  }
  if (indx < MAX_ASSOCIATION_IN_GROUP)
  {
    AssociationStoreAll();
  }
}
