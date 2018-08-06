/*******************************  ZW_SYSDEFS.H  *****************************
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
 * Description: Z-Wave system defines
 *
 * Author:   Ivar Jeppesen
 *
 * Last Changed By:  $Author: sse $
 * Revision:         $Revision: 9286 $
 * Last Changed:     $Date: 2007-09-11 16:13:14 +0200 (ti, 11 sep 2007) $
 *
 ****************************************************************************/
#ifndef _ZW_SYSDEFS_H_
#define _ZW_SYSDEFS_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#ifdef ZW030x
#include <ZW030x.h>
#elif defined(ZW020x)
#include <ZW020x.h>
#endif

#define __flash	code

/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/

/* Reference frequency */
#if defined(ZW020x) || defined(ZW030x)
#define CPU_FREQ  16000000
#endif

#endif /* _ZW_SYSDEFS_H_ */
