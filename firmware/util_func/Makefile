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
##    Makefile for the ZW0x0x util_func
##    This makefile defines the targets built
##
## Author:   Erik Friis Harck
##
## Last Changed By:  $Author: jfr $
## Revision:         $Revision: 16741 $
## Last Changed:     $Date: 2010-02-22 15:12:27 +0100 (ma, 22 feb 2010) $
##
##***************************************************************************/
# Find the current directory
# used as a base for all other paths.
BASEDIR=$(shell cd)

# The command to make dependencies from the headerfiles
# included by a file.
INCDEP=python $(subst \,\\,$(TOOLSDIR)\IncDep\incdep.py)

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

## Set the type of the non-volatile memory technology. Now it an external EEPROM
#  The ext_nvm directory contains external non-volatile memory drivers:
#
#  ZW_at25128a_spi_if.obj – Atmel SPI Serial EEPROM AT25128A
#  ZW_m25pe10_spi_if.obj – STMicroelectronics Serial Flash M25PE10
#
ifeq ($(NVM_TYPE),)
NVM_TYPE:=at25128a
endif

# Export the variables declared above to the other makefiles.
export BASEDIR INCDEP ZWLIBROOT UNDERSTAND_C NVM_TYPE

BDIRS=build

ZW0x0x: ZW030x

ZW030x:

##***************************************************************************/

# Include test targets (if any)
-include Makefile.TestTargets
-include Makefile.TestDefines

dirs: $(BDIRS)

MAKEOPTS=--no-print-directory -C

##***************************************************************************/

###########################################################################
## ZW030x targets for FLASH code space in normal mode


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
	@echo   ZW030x     : ZW030x targets
	@echo   ZW030x_tf  : ZW030x test frequency targets
	@echo   clean      : Erase all generated files

# All targets that shouldn't result in a file/directory
# with the same name as the target should be listed here.
# Otherwise the above rule will make a directory with
# the targetname

.PHONY: dirs clean depend all default\
 ZW0x0x\
 ZW0x0x_tf\
 ZW030x\
 ZW030x_tf\
 ZW030x_normal\
 ZW030x_devmode\
 Makefile.TestTargets\
 Makefile.TestDefines
