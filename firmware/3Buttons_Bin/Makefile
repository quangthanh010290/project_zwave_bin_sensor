##******************************* Makefile *********************************
##           #######
##           ##  ##
##           #  ##    ####   #####    #####  ##  ##   #####
##             ##    ##  ##  ##  ##  ##      ##  ##  ##
##            ##  #  ######  ##  ##   ####   ##  ##   ####
##           ##  ##  ##      ##  ##      ##   #####      ##
##          #######   ####   ##  ##  #####       ##  #####
##                                           #####
##          Z-Wave, the wireless language.
##
##              Copyright (c) 2001
##              Zensys A/S
##              Denmark
##
##              All Rights Reserved
##
##    This source file is subject to the terms and conditions of the
##    Zensys Software License Agreement which restricts the manner
##    in which it may be used.
##
##---------------------------------------------------------------------------
##
## Description:
##    Makefile for the ZW0x0x development kit binary battery sensor
##    This makefile defines the targets built
##
## Author:   Peter Shorty
##
## Last Changed By:  $Author: psh $
## Revision:         $Revision: 24007 $
## Last Changed:     $Date: 2013-01-17 13:23:54 +0100 (to, 17 jan 2013) $
##
##***************************************************************************/

# Find the current directory
# used as a base for all other paths.
BASEDIR=$(shell cd)

# Find the product name from the subdirectoryname
BASEDIRWORDLIST=$(subst \, ,$(BASEDIR))
BASEDIRLASTWORDINDEX=$(words $(BASEDIRWORDLIST))
PRODUCTDIR=$(word $(BASEDIRLASTWORDINDEX),$(BASEDIRWORDLIST))

# Find the Z-Wave library location (if not forced by setting the environment variable ZWLIBROOT)
ifndef ZWLIBROOT
ZWLIBROOT:=$(subst \$(PRODUCTDIR),,$(subst \Product,\Z-Wave,$(BASEDIR)))
endif

# Library location when making release of application
ifdef SVNLIBRELEASEDIRBIN
ZWLIBROOT:=$(SVNLIBRELEASEDIRBIN)
endif

# Export the variables declared above to the other makefiles.
export BASEDIR ZWLIBROOT UNDERSTAND_C

BDIRS=build

all: ZW0x0x

default: ZW0x0x


# Standard make names
ZW020x:\
 ZW020x_NOSEC

ZW030x:\
 ZW030x_NOSEC

ZW0x0x:\
 ZW0x0x_NOSEC

ZW020x_NOSEC:\
 Bin_Sensor_ZW020x\
 Bin_Sensor_noflirs_nomr_ZW020x\
 Bin_Sensor_232_ZW020x\
 Bin_Sensor_Battery_ZW020x\
 Bin_Sensor_Battery_232_ZW020x

ZW030x_NOSEC:\
 Bin_Sensor_ZW030x\
 Bin_Sensor_noflirs_nomr_ZW030x\
 Bin_Sensor_232_ZW030x\
 Bin_Sensor_Battery_ZW030x\
 Bin_Sensor_Battery_232_ZW030x

ZW0x0x_NOSEC: ZW020x_NOSEC ZW030x_NOSEC

ZW020x_EU:  Bin_Sensor_ZW020x_EU
ZW020x_US:  Bin_Sensor_ZW020x_US
ZW020x_ANZ: Bin_Sensor_ZW020x_ANZ
ZW020x_HK:  Bin_Sensor_ZW020x_HK
ZW020x_MY:  Bin_Sensor_ZW020x_MY
ZW020x_IN:  Bin_Sensor_ZW020x_IN
ZW020x_RU:  Bin_Sensor_ZW020x_RU
ZW020x_IL:  Bin_Sensor_ZW020x_IL
ZW030x_EU:  Bin_Sensor_ZW030x_EU
ZW030x_US:  Bin_Sensor_ZW030x_US
ZW030x_ANZ: Bin_Sensor_ZW030x_ANZ
ZW030x_HK:  Bin_Sensor_ZW030x_HK
ZW030x_MY:  Bin_Sensor_ZW030x_MY
ZW030x_IN:  Bin_Sensor_ZW030x_IN
ZW030x_RU:  Bin_Sensor_ZW030x_RU
ZW030x_IL:  Bin_Sensor_ZW030x_IL


# Common targets
Bin_Sensor_ZW0x0x: Bin_Sensor_ZW020x Bin_Sensor_ZW030x
Bin_Sensor_noflirs_nomr_ZW0x0x: Bin_Sensor_noflirs_nomr_ZW020x Bin_Sensor_noflirs_nomr_ZW030x
Bin_Sensor_232_ZW0x0x: Bin_Sensor_232_ZW020x Bin_Sensor_232_ZW030x
Bin_Sensor_Battery_ZW0x0x: Bin_Sensor_Battery_ZW020x Bin_Sensor_Battery_ZW030x
Bin_Sensor_Battery_232_ZW0x0x: Bin_Sensor_Battery_232_ZW020x Bin_Sensor_Battery_232_ZW030x

# ZW020x targets
Bin_Sensor_ZW020x: Bin_Sensor_ZW020x_EU Bin_Sensor_ZW020x_US Bin_Sensor_ZW020x_ANZ Bin_Sensor_ZW020x_HK Bin_Sensor_ZW020x_MY Bin_Sensor_ZW020x_IN Bin_Sensor_ZW020x_RU Bin_Sensor_ZW020x_IL
Bin_Sensor_noflirs_nomr_ZW020x: Bin_Sensor_noflirs_nomr_ZW020x_EU Bin_Sensor_noflirs_nomr_ZW020x_US Bin_Sensor_noflirs_nomr_ZW020x_ANZ Bin_Sensor_noflirs_nomr_ZW020x_HK Bin_Sensor_noflirs_nomr_ZW020x_MY Bin_Sensor_noflirs_nomr_ZW020x_IN Bin_Sensor_noflirs_nomr_ZW020x_RU Bin_Sensor_noflirs_nomr_ZW020x_IL
Bin_Sensor_232_ZW020x: Bin_Sensor_232_ZW020x_EU Bin_Sensor_232_ZW020x_US Bin_Sensor_232_ZW020x_ANZ Bin_Sensor_232_ZW020x_HK Bin_Sensor_232_ZW020x_MY Bin_Sensor_232_ZW020x_IN Bin_Sensor_232_ZW020x_RU Bin_Sensor_232_ZW020x_IL
Bin_Sensor_Battery_ZW020x: Bin_Sensor_Battery_ZW020x_EU Bin_Sensor_Battery_ZW020x_US Bin_Sensor_Battery_ZW020x_ANZ Bin_Sensor_Battery_ZW020x_HK Bin_Sensor_Battery_ZW020x_MY Bin_Sensor_Battery_ZW020x_IN Bin_Sensor_Battery_ZW020x_RU Bin_Sensor_Battery_ZW020x_IL
Bin_Sensor_Battery_232_ZW020x: Bin_Sensor_Battery_232_ZW020x_EU Bin_Sensor_Battery_232_ZW020x_US Bin_Sensor_Battery_232_ZW020x_ANZ Bin_Sensor_Battery_232_ZW020x_HK Bin_Sensor_Battery_232_ZW020x_MY Bin_Sensor_Battery_232_ZW020x_IN Bin_Sensor_Battery_232_ZW020x_RU Bin_Sensor_Battery_232_ZW020x_IL

# ZW030x targets
Bin_Sensor_ZW030x: Bin_Sensor_ZW030x_EU Bin_Sensor_ZW030x_US Bin_Sensor_ZW030x_ANZ Bin_Sensor_ZW030x_HK Bin_Sensor_ZW030x_MY Bin_Sensor_ZW030x_IN Bin_Sensor_ZW030x_RU Bin_Sensor_ZW030x_IL
Bin_Sensor_noflirs_nomr_ZW030x: Bin_Sensor_noflirs_nomr_ZW030x_EU Bin_Sensor_noflirs_nomr_ZW030x_US Bin_Sensor_noflirs_nomr_ZW030x_ANZ Bin_Sensor_noflirs_nomr_ZW030x_HK Bin_Sensor_noflirs_nomr_ZW030x_MY Bin_Sensor_noflirs_nomr_ZW030x_IN Bin_Sensor_noflirs_nomr_ZW030x_RU Bin_Sensor_noflirs_nomr_ZW030x_IL
Bin_Sensor_232_ZW030x: Bin_Sensor_232_ZW030x_EU Bin_Sensor_232_ZW030x_US Bin_Sensor_232_ZW030x_ANZ Bin_Sensor_232_ZW030x_HK Bin_Sensor_232_ZW030x_MY Bin_Sensor_232_ZW030x_IN Bin_Sensor_232_ZW030x_RU Bin_Sensor_232_ZW030x_IL
Bin_Sensor_Battery_ZW030x: Bin_Sensor_Battery_ZW030x_EU Bin_Sensor_Battery_ZW030x_US Bin_Sensor_Battery_ZW030x_ANZ Bin_Sensor_Battery_ZW030x_HK Bin_Sensor_Battery_ZW030x_MY Bin_Sensor_Battery_ZW030x_IN Bin_Sensor_Battery_ZW030x_RU Bin_Sensor_Battery_ZW030x_IL
Bin_Sensor_Battery_232_ZW030x: Bin_Sensor_Battery_232_ZW030x_EU Bin_Sensor_Battery_232_ZW030x_US Bin_Sensor_Battery_232_ZW030x_ANZ Bin_Sensor_Battery_232_ZW030x_HK Bin_Sensor_Battery_232_ZW030x_MY Bin_Sensor_Battery_232_ZW030x_IN Bin_Sensor_Battery_232_ZW030x_RU Bin_Sensor_Battery_232_ZW030x_IL

##***************************************************************************/
# Include secure targets (if any)
-include Makefile.SecureTargets

##***************************************************************************/
# Include test targets (if any)
-include Makefile.TestTargets
-include Makefile.Testdefines

dirs: $(BDIRS)

MAKEOPTS=--no-print-directory -C

#################################################
# ZW020x target makefiles

Bin_Sensor_ZW020x_EU:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=EU RF=EU SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW020x_EU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW020x_US:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=US RF=US SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW020x_US:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW020x_ANZ:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=ANZ RF=ANZ SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW020x_ANZ:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW020x_HK:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=HK RF=HK SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW020x_HK:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW020x_MY:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=MY RF=MY SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW020x_MY:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW020x_IN:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=IN RF=IN SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW020x_IN:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW020x_RU:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=RU RF=RU SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW020x_RU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW020x_IL:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=IL RF=IL SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW020x_IL:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW020x_EU:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW020x COUNTRY=EU RF=EU SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW020x_EU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW020x_US:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW020x COUNTRY=US RF=US SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW020x_US:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW020x_ANZ:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW020x COUNTRY=ANZ RF=ANZ SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW020x_ANZ:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW020x_HK:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW020x COUNTRY=HK RF=HK SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW020x_HK:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW020x_MY:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW020x COUNTRY=MY RF=MY SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW020x_MY:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW020x_IN:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW020x COUNTRY=IN RF=IN SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW020x_IN:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW020x_RU:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW020x COUNTRY=RU RF=RU SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW020x_RU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW020x_IL:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW020x COUNTRY=IL RF=IL SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW020x_IL:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW020x_EU:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=EU RF=EU SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW020x_EU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW020x_US:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=US RF=US SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW020x_US:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW020x_ANZ:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=ANZ RF=ANZ SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW020x_ANZ:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW020x_HK:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=HK RF=HK SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW020x_HK:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW020x_MY:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=MY RF=MY SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW020x_MY:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW020x_IN:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=IN RF=IN SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW020x_IN:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW020x_RU:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=RU RF=RU SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW020x_RU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW020x_IL:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=IL RF=IL SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW020x_IL:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW020x_EU:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=EU RF=EU SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW020x_EU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW020x_US:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=US RF=US SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW020x_US:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW020x_ANZ:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=ANZ RF=ANZ SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW020x_ANZ:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW020x_HK:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=HK RF=HK SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW020x_HK:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW020x_MY:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=MY RF=MY SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW020x_MY:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW020x_IN:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=IN RF=IN SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW020x_IN:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW020x_RU:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=RU RF=RU SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW020x_RU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW020x_IL:MAKELINE=LIB=slave_enhanced CHIP=ZW020x COUNTRY=IL RF=IL SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW020x_IL:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW020x_EU:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=EU RF=EU SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW020x_EU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW020x_US:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=US RF=US SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW020x_US:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW020x_ANZ:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=ANZ RF=ANZ SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW020x_ANZ:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW020x_HK:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=HK RF=HK SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW020x_HK:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW020x_MY:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=MY RF=MY SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW020x_MY:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW020x_IN:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=IN RF=IN SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW020x_IN:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW020x_RU:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=RU RF=RU SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW020x_RU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW020x_IL:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW020x COUNTRY=IL RF=IL SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW020x_IL:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

# ZW030x target makefiles

Bin_Sensor_ZW030x_EU:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=EU RF=EU SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW030x_EU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW030x_US:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=US RF=US SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW030x_US:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW030x_ANZ:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=ANZ RF=ANZ SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW030x_ANZ:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW030x_HK:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=HK RF=HK SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW030x_HK:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW030x_MY:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=MY RF=MY SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW030x_MY:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW030x_IN:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=IN RF=IN SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW030x_IN:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW030x_RU:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=RU RF=RU SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW030x_RU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_ZW030x_IL:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=IL RF=IL SENSOR_TYPE=NON_BATT
Bin_Sensor_ZW030x_IL:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW030x_EU:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW030x COUNTRY=EU RF=EU SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW030x_EU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW030x_US:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW030x COUNTRY=US RF=US SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW030x_US:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW030x_ANZ:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW030x COUNTRY=ANZ RF=ANZ SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW030x_ANZ:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW030x_HK:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW030x COUNTRY=HK RF=HK SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW030x_HK:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW030x_MY:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW030x COUNTRY=MY RF=MY SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW030x_MY:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW030x_IN:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW030x COUNTRY=IN RF=IN SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW030x_IN:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW030x_RU:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW030x COUNTRY=RU RF=RU SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW030x_RU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_noflirs_nomr_ZW030x_IL:MAKELINE=LIB=slave_enhanced_noflirs_nomr CHIP=ZW030x COUNTRY=IL RF=IL SENSOR_TYPE=NON_BATT SLTYPE=_noflirs_nomr
Bin_Sensor_noflirs_nomr_ZW030x_IL:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW030x_EU:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=EU RF=EU SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW030x_EU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW030x_US:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=US RF=US SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW030x_US:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW030x_ANZ:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=ANZ RF=ANZ SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW030x_ANZ:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW030x_HK:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=HK RF=HK SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW030x_HK:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW030x_MY:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=MY RF=MY SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW030x_MY:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW030x_IN:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=IN RF=IN SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW030x_IN:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW030x_RU:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=RU RF=RU SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW030x_RU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_232_ZW030x_IL:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=IL RF=IL SENSOR_TYPE=NON_BATT SLTYPE=_232
Bin_Sensor_232_ZW030x_IL:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW030x_EU:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=EU RF=EU SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW030x_EU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW030x_US:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=US RF=US SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW030x_US:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW030x_ANZ:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=ANZ RF=ANZ SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW030x_ANZ:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW030x_HK:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=HK RF=HK SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW030x_HK:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW030x_MY:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=MY RF=MY SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW030x_MY:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW030x_IN:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=IN RF=IN SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW030x_IN:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW030x_RU:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=RU RF=RU SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW030x_RU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_ZW030x_IL:MAKELINE=LIB=slave_enhanced CHIP=ZW030x COUNTRY=IL RF=IL SENSOR_TYPE=BATTERY
Bin_Sensor_Battery_ZW030x_IL:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW030x_EU:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=EU RF=EU SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW030x_EU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW030x_US:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=US RF=US SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW030x_US:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW030x_ANZ:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=ANZ RF=ANZ SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW030x_ANZ:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW030x_HK:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=HK RF=HK SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW030x_HK:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW030x_MY:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=MY RF=MY SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW030x_MY:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW030x_IN:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=IN RF=IN SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW030x_IN:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW030x_RU:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=RU RF=RU SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW030x_RU:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

Bin_Sensor_Battery_232_ZW030x_IL:MAKELINE=LIB=slave_enhanced_232 CHIP=ZW030x COUNTRY=IL RF=IL SENSOR_TYPE=BATTERY SLTYPE=_232
Bin_Sensor_Battery_232_ZW030x_IL:
	$(MAKE) depend $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common
	$(MAKE) $(MAKEOPTS) . $(MAKELINE) -f Makefile.binsensor_common

##***************************************************************************/

# Deletes all generated files (relocatables, downloadables,
# dependency files and the generated C and headerfiles from ZUIB)
clean:
ifeq ($(OS), Windows_NT)
	@cmd /c "if exist build\NUL rd build /s /q"
else
	@deltree /Y $(subst \,\\,build)
endif

##***************************************************************************/

# Catch all targets which we do not handle and tell world that
%:
	@echo Could not find rule for target '$@'
	@echo Some of the known targets are :
	@echo   Bin_Sensor_ZW020x     : ZW020x targets
	@echo   Bin_Sensor_ZW030x     : ZW030x targets
	@echo   Bin_Sensor_ZW0x0x     : All targets


# All targets that shouldn't result in a file/directory
# with the same name as the target should be listed here.
# Otherwise the above rule will make a directory with
# the targetname
.PHONY: dirs clean depend all default Makefile.TestTargets Makefile.Testdefines Makefile.SecureTargets\
 ZW0x0x ZW020x ZW030x\
 Bin_Sensor_ZW0x0x\
 Bin_Sensor_ZW020x\
 Bin_Sensor_ZW030x\
 Bin_Sensor_noflirs_nomr_ZW0x0x\
 Bin_Sensor_noflirs_nomr_ZW020x\
 Bin_Sensor_noflirs_nomr_ZW030x\
 Bin_Sensor_232_ZW0x0x\
 Bin_Sensor_232_ZW020x\
 Bin_Sensor_232_ZW030x\
 Bin_Sensor_Battery_ZW0x0x\
 Bin_Sensor_Battery_ZW020x\
 Bin_Sensor_Battery_ZW030x\
 Bin_Sensor_Battery_232_ZW0x0x\
 Bin_Sensor_Battery_232_ZW020x\
 Bin_Sensor_Battery_232_ZW030x\
 ZW0x0x_NOSEC\
 ZW020x_NOSEC\
 ZW030x_NOSEC\
 ZW020x_EU\
 ZW020x_US\
 ZW020x_ANZ\
 ZW020x_HK\
 ZW020x_MY\
 ZW020x_IN\
 ZW020x_RU\
 ZW020x_IL\
 ZW030x_EU\
 ZW030x_US\
 ZW030x_ANZ\
 ZW030x_HK\
 ZW030x_MY\
 ZW030x_IN\
 ZW030x_RU\
 ZW030x_IL
