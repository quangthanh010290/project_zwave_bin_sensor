/********************************  EEPROM.H  ********************************
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
 * Description: Internal EEPROM address definitions
 *
 * Author:   Peter Shorty
 *
 * Last Changed By:  $Author: iza $
 * Revision:         $Revision: 21352 $
 * Last Changed:     $Date: 2011-09-23 10:59:46 +0200 (fr, 23 sep 2011) $
 *
 ****************************************************************************/
#ifndef _EEPROM_H_
#define _EEPROM_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <Switch3.h>

/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/

/* EEPROM address definitions */

/* EEPROM binary battery sensor node layout */
#define EEOFFSET_SENSOR_LEVEL             0x00
#define EEOFFSET_SENSOR_STATUS            EEOFFSET_SENSOR_LEVEL + 1
#define EEOFFSET_SENSOR_ONVALUE           EEOFFSET_SENSOR_STATUS + 1
#define EEOFFSET_SENSOR_ONTIME            EEOFFSET_SENSOR_ONVALUE + 1

#define EEOFFSET_POWERDOWNTIMEOUT         EEOFFSET_SENSOR_ONTIME + 1
#define EEOFFSET_KEEPALIVETIMEOUT         EEOFFSET_POWERDOWNTIMEOUT + 1
#define EEOFFSET_KEEPALIVEACTIVATETIMEOUT EEOFFSET_KEEPALIVETIMEOUT + 1
#define EEOFFSET_WAKEUP_COUNT_1           EEOFFSET_KEEPALIVEACTIVATETIMEOUT + 1
#define EEOFFSET_WAKEUP_COUNT_2           EEOFFSET_WAKEUP_COUNT_1 + 1
#define EEOFFSET_WAKEUP_COUNT_3           EEOFFSET_WAKEUP_COUNT_2 + 1
#define EEOFFSET_WAKEUP_COUNT_4           EEOFFSET_WAKEUP_COUNT_3 + 1

#define EEOFFSET_MASTER_NODEID            EEOFFSET_WAKEUP_COUNT_4 + 1
#define EEOFFSET_SLEEP_PERIOD_1           EEOFFSET_MASTER_NODEID + 1
#define EEOFFSET_SLEEP_PERIOD_2           EEOFFSET_SLEEP_PERIOD_1 + 1
#define EEOFFSET_SLEEP_PERIOD_3           EEOFFSET_SLEEP_PERIOD_2 + 1
#define EEOFFSET_RTC_TIMER_HANDLE         EEOFFSET_SLEEP_PERIOD_3 + 1

#ifdef ZW_SELF_HEAL
#define EEOFFSET_LOST_COUNTER             EEOFFSET_RTC_TIMER_HANDLE + 1
#define EEOFFSET_COMMON_END               EEOFFSET_LOST_COUNTER
#else
#define EEOFFSET_COMMON_END               EEOFFSET_RTC_TIMER_HANDLE
#endif /*ZW_SELF_HEAL*/

#define EEOFFSET_LERN_MODE		  EEOFFSET_COMMON_END + 1	
#define EEOFFSET_MAGIC                    EEOFFSET_LERN_MODE + 1  /* MAGIC */

#define EEOFFSET_ASSOCIATION_START        EEOFFSET_MAGIC + 1
#define EEOFFSET_ASSOCIATION_MAGIC        EEOFFSET_ASSOCIATION_START + ASSOCIATION_SIZE

#define EEOFFSET_TRANSPORT_SETTINGS_START EEOFFSET_ASSOCIATION_MAGIC + 1
#define EEOFFSET_TRANSPORT_SETTINGS_SIZE  TRANSPORT_EEPROM_SETTINGS_SIZE

#endif /* _EEPROM_H_ */
