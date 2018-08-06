/****************************************************************************
 *
 * Copyright (c) 2001-2012
 * Sigma Designs, Inc.
 * All Rights Reserved
 *
 *---------------------------------------------------------------------------
 *
 * Description: Declaration of Z-Wave firmware descriptor.
 *
 * Author:   Erik Friis Harck
 *
 * Last Changed By:  $Author: iza $
 * Revision:         $Revision: 22797 $
 * Last Changed:     $Date: 2012-05-10 15:55:06 +0200 (to, 10 maj 2012) $
 *
 ****************************************************************************/
#ifndef _FIRMWARE_DESCRIPTOR_H_
#define _FIRMWARE_DESCRIPTOR_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/


/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

/* Firmware descriptor for OTA firmware update. Located at the end of firmware. */
typedef struct s_firmwareDescriptor_
{
  VOID_CALLBACKFUNC(startAddress)(); /* The applications start address               */
  WORD nvmUseSize;
  WORD manufacturerID;
  WORD firmwareID;
  WORD checksum;
} t_firmwareDescriptor;

/* Full firmware structure for OTA firmware update */
typedef struct s_firmware_
{
  BYTE jumpInstuction;        /* ljmp instruction byte of reset vector               */
                              /*                 CSEG    AT      0                   */
                              /* ?C_STARTUP:     LJMP    STARTUP1                    */
  WORD startAddress;          /* The applications start address of reset vector      */
  WORD applicationSize;       /* Filled-in with the actual size of this struct       */
                              /* Used to calculate the address of the checksum       */
                              /* as &sFirmware + sFirmware.applicationSize           */
  BYTE applicationCode[1000]; /* 1000 is just an arbitrary example */
                              /* Every field beyond this you must calculate the      */
                              /* pointer to, because the size of the application     */
                              /* is unknown at compilation time for the bootloader.  */
  t_firmwareDescriptor firmwareDescriptor;
} t_firmware;

extern code t_firmwareDescriptor firmwareDescriptor;
extern code WORD firmwareSize;

#endif /* _FIRMWARE_DESCRIPTOR_H_ */
