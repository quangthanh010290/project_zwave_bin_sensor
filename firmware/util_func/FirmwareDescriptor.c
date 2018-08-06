/****************************************************************************
 *
 * Copyright (c) 2001-2012
 * Sigma Designs, Inc.
 * All Rights Reserved
 *
 *---------------------------------------------------------------------------
 *
 * Description: Definition and initialization of Z-Wave firmware descriptor.
 *
 * Author:   Erik Friis Harck
 *
 * Last Changed By:  $Author: iza $
 * Revision:         $Revision: 22797 $
 * Last Changed:     $Date: 2012-05-10 15:55:06 +0200 (to, 10 maj 2012) $
 *
 ****************************************************************************/

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include "config_app.h"
#include "ZW_basis_api.h"
#include "ZW_eep_addr.h"

/* Include files for references in eeprom.h to size of NVM data structures */
#include "ZW_TransportLayer.h"
#include "association.h"
#include "eeprom.h"
#include "FirmwareDescriptor.h"

extern void STARTUP_APPLICATION(void);

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

/* Firmware descriptor for OTA firmware update */
code t_firmwareDescriptor firmwareDescriptor =
{
  /* The current applications start address */
  /* VOID_CALLBACKFUNC(startAddress)(); */ &STARTUP_APPLICATION,
  /* Total amount of NVM allocated for Z-Wave application program incl. library */
  /* WORD nvmUseSize; */       EEPROM_APPL_OFFSET + EEOFFSET_APPLICATION_DATA_END,
  /* TODO: Fill in your unique and assigned manufacturer ID here                */
  /* WORD manufacturerID; */   MFG_ID_SIGMA_DESIGNS,
  /* TODO: Fill in your own unique firmware ID here                             */
  /* WORD firmwareID; */       0,
  /* A CRC-CCITT must be, and will be, filled in here by a software build       */
  /* tool (fixbootcrc.exe).                                                     */
  /* WORD checksum; */         0
};

