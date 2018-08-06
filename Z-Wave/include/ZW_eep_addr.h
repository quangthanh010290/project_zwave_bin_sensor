/*******************************  ZW_EEP_ADDR.H  *******************************
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
 * Description: Application EEPROM start address
 *
 * Author:   Peter Shorty
 *
 * Last Changed By:  $Author: jfr $
 * Revision:         $Revision: 25278 $
 * Last Changed:     $Date: 2013-04-05 16:36:40 +0200 (fr, 05 apr 2013) $
 *
 ****************************************************************************/
#ifndef _ZW_EEP_ADDR_H_
#define _ZW_EEP_ADDR_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/


/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/

/* The EEPROM_APPL_OFFSET is used internally in the protocol on library buildtime. */

#if defined(ZW_SLAVE) && (!defined(ZW_SLAVE_32))
/*****************************************************************************************/
/* ZW020x/ZW030x based ZW_SLAVE/ZW_SLAVE_ROUTING (!ZW_SLAVE_32) targets have their NVRAM */
/* placed in flash and Application have been allocated 125 byte of NVRAM.
/* The EEPROM_APPL_OFFSET have for these targets no real Application orientated use. */

#define EEPROM_APPL_OFFSET    0x80
#endif

/**************************************************************************************************/
/* For ZW020x/ZW030x based ZW_SLAVE_32/ZW_SLAVE_RETURNROUTEDEST_232 and all ZW_CONTROLLER targets */
/* there must exist an external EEPROM and the EEPROM_APPL_OFFSET indicates where the */
/* application NVRAM data start address is located in the external EEPROM. */

#ifdef ZW_SLAVE_32
/* TO#2508 */
#ifdef ZW_SLAVE_RETURNROUTEDEST_232
#define EEPROM_APPL_OFFSET    0x1200
#else
#define EEPROM_APPL_OFFSET    0x120
#endif
#endif  /* ZW_SLAVE_32 */

#if defined(ZW_CONTROLLER) || defined(ZW_EEP_LOADER)
#define EEPROM_APPL_OFFSET    0x2C00
#endif

#endif /* _ZW_EEP_ADDR_H_ */
