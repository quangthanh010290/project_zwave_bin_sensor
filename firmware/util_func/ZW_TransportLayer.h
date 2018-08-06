/******************************* ZW_TransportLayer.h *******************************
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
 *              Copyright (c) 2009
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
 * Description: Transpotr layer type selector.
 *
 * Author:   Valeriy Vyshnyak
 *
 * Last Changed By:  $Author: vvi $
 * Revision:         $Revision: 13417 $
 * Last Changed:     $Date: 2009-03-10 16:17:52 +0200 (Вв, 10 Бер 2009) $
 *
 ****************************************************************************/
#ifndef _TRANSPORT_LAYER_H_
#define _TRANSPORT_LAYER_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#ifdef SECURITY
#include <ZW_TransportSecurity.h>
#else
#include <ZW_TransportNative.h>
#endif


#endif /*_TRANSPORT_LAYER_H_*/
