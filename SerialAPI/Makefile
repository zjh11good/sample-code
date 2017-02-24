#****************************************************************************
#
#  Copyright (c) 2001-2013
#  Sigma Designs, Inc.
#  All Rights Reserved
#
#----------------------------------------------------------------------------
#
# Description:
#    Makefile for the ZW0x0x development kit SerialAPI
#    This makefile defines the targets built
#
# Author:   Erik Friis Harck
#
# Last Changed By:  $Author: jsi $
# Revision:         $Revision: 29270 $
# Last Changed:     $Date: 2014-07-02 14:59:40 +0200 (Wed, 02 Jul 2014) $
#
#****************************************************************************

#---------------------------------------------------------------------------------------------------------
# HOW-TO: Specifying which subset of "everything", you wan't to build:
#
# You can specify a single frequency to build for in 3 different ways:
# 1. By specifying the frequency in your command line, like:
#    > make FREQUENCY=EU ....
# 2. By uncommenting and correcting the following line:
#FREQUENCY:=US
# 3. Alternativly you can do the same by setting your environment with:
#    > SET FREQUENCY=EU
#    Remember to UNSET this when you jump to work on other things.
#
# You can specify a single code memory mode to build for in 3 different ways:
# 1. By specifying the code memory mode in your command line, like:
#    > make CODE_MEMORY_MODE=banking ....
# 2. By uncommenting and correcting the following line:
#CODE_MEMORY_MODE:=banking
# 3. Alternativly you can do the same by setting your environment with:
#    > SET CODE_MEMORY_MODE=banking
#    Remember to UNSET this when you jump to work on other things.
#
# You can specify a single library to build for in 3 different ways:
# 1. By specifying the library in your command line, like:
#    > make LIBRARY=controller_portable ....
# 2. By uncommenting and correcting the following line:
#LIBRARY:=controller_static
# 3. Alternativly you can do the same by setting your environment with:
#    > SET LIBRARY=controller_portable
#    Remember to UNSET this when you jump to work on other things.
#
# You can specify a single host interface to build for in 3 different ways:
# 1. By specifying the host interface in your command line, like:
#    > make HOST_INTERFACE=UART ....
# 2. By uncommenting and correcting the following line:
#HOST_INTERFACE:=UART
# 3. Alternativly you can do the same by setting your environment with:
#    > SET HOST_INTERFACE=UART
#    Remember to UNSET this when you jump to work on other things.
#
# You can specify a single sensor type to build for in 3 different ways:
# 1. By specifying the sensor type in your command line, like:
#    > make SENSOR_TYPE=NON_BATT ....
# 2. By uncommenting and correcting the following line:
SENSOR_TYPE:=NON_BATT
# 3. Alternativly you can do the same by setting your environment with:
#    > SET SENSOR_TYPE=NON_BATT
#    Remember to UNSET this when you jump to work on other things.
#
# You can specify a single board type to build for in 3 different ways:
# 1. By specifying the sensor type in your command line, like:
#    > make BOARD=ZDP03A .... UZB
# 2. By uncommenting and correcting the following line:
#BOARD=ZDP03A
# 3. Alternativly you can do the same by setting your environment with:
#    > SET BOARD=ZDP03A
#    Remember to UNSET this when you jump to work on other things.
#
# And yes... You can combine these methods in any way.
#
# If you do nothing of the above, you will be building target code for all combinations of frequencies and
# code memory modes in the lists below.
#
# And yes... You can of course manipulate the lists below, if you have special needs for ranges of targets.
#---------------------------------------------------------------------------------------------------------

# Full list of all supported libraries can be found in $(ZWLIBROOT)\Common\Makefile.common
LIST_OF_LIBRARY:=\
 controller_bridge\
 controller_portable\
 controller_static\
 controller_static_norep\
 controller_static_single\
 slave_enhanced_232\
 slave_routing

# Full list of all supported host interfaces can be found in $(ZWLIBROOT)\Common\Makefile.common
# Downgraded list of supported host interfaces for SerialAPI.
LIST_OF_HOST_INTERFACE:=UART USBVCP

#---------------------------------------------------------------------------------------------------------

# If not supplied by application here, the name of the application will be set equal to it's directory name.
APP_NAME:=serialapi

CHIPPACK:=ZW050x

##***************************************************************************/
# Include test board targets (if any)
-include Makefile.TestBoards

ifneq ($(TESTBOARD),1)
 ifeq ($(BOARD),ZM5202)
  # Set the driver for external serial non-volatile memory for use of SPI port 1, and P0.4 as chip select.
  NVM_TYPE:=generic_p0_4_spi1
  CHIPPACK:=ZM5202
 else
  # We are building for the normal general ZW050x chip.
  CHIPPACK:=$(CHIP)
 endif
endif
ifneq ($(BOOTLOADER_ENABLED),)
ifeq ($(LIBRARY),slave_routing)
LIBRARY:=slave_enhanced_232
override LIB:=slave_enhanced_232
endif
endif

ifneq ($(IF),)
IFNAME:=_$(IF)
endif

# Target name (Name of your target directory and base name of your target files)
TARGET:=$(APP)_$(LIB)
ifneq ($(BOOTLOADER_ENABLED),)
TARGET:=$(TARGET)_OTW
endif
ifneq ($(IMA_ENABLED),)
TARGET:=$(TARGET)_IMA
LIST_OF_LIBRARY:=controller_bridge controller_static controller_static_norep
endif
TARGET:=$(TARGET)_$(CHIPPACK)$(IFNAME)_$(COUNTRY)$(TEST)


#---------------------------------------------------------------------------------------------------------

# List of all application modules.

ifeq ($(BOOTLOADER),)
RELFILES:=\
 serialappl.obj\
 conhandle.obj\
 led_control.obj\
 eeprom.obj

ifeq ($(TESTBOARD),1)
RELFILES+=\
   prodtest_vars.obj
endif

endif


#---------------------------------------------------------------------------------------------------------

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

# Utility location
ifeq ($(ZWUTILDIR),)
ZWUTILDIR:=..\util_func
endif

-include $(ZWUTILDIR)\Makefile.common_bootloader

include $(ZWUTILDIR)\Makefile.common

# Filter out unsupported frequencies for the Z-Wave application.
# Full list of all supported frequencies can be found in $(ZWUTILDIR)\Makefile.common
# (this filter-out must be done after/below the inclusion of $(ZWUTILDIR)\Makefile.common)
LIST_OF_FREQUENCY:=$(filter-out ALL ALL_3CH,$(LIST_OF_FREQUENCY))

#---------------------------------------------------------------------------------------------------------
# ADDITION TO THE VARIANTS TO BUILD
#---------------------------------------------------------------------------------------------------------
# Add a dimension "IMA" to the build array
#
# IMA variant must be added in front of LIST_OF_VARIANTS to be handled last,
# and for the filtering below in this Makefile to work. (LIST_OF_VARIANTS+= IMA # does not work correctly)
LIST_OF_VARIANTS:=IMA $(LIST_OF_VARIANTS)
# A full list of values for each VariantName must be assigned.
LIST_OF_IMA:=nonIMA IMA_ENABLED
# A default value for each VariantName must be assigned. They are used in the help texts.
IMA_DEFAULT:=nonIMA
# An option variable for each VariantName must be assigned to be considered during "make",
# and of course, the application source code must be prepared to distinguish between variants.
ifeq ($(IMA),IMA_ENABLED)
IMA_OPTION:=IMA_ENABLED=1
IMA_ENABLED:=1
endif
#---------------------------------------------------------------------------------------------------------

#---------------------------------------------------------------------------------------------------------
# ADDITION TO THE VARIANTS TO BUILD
#---------------------------------------------------------------------------------------------------------
# Add a dimension "BOOT" to the build array
#
# BOOT variant must be added in front of LIST_OF_VARIANTS to be handled last,
# and for the filtering below in this Makefile to work. (LIST_OF_VARIANTS+= BOOT # does not work correctly)
LIST_OF_VARIANTS:=BOOT $(LIST_OF_VARIANTS)
# A full list of values for each VariantName must be assigned.
LIST_OF_BOOT:=nonBOOT BOOTLOADER BOOTLOADER_ENABLED
# A default value for each VariantName must be assigned. They are used in the help texts.
BOOT_DEFAULT:=nonBOOT
# An option variable for each VariantName must be assigned to be considered during "make",
# and of course, the application source code must be prepared to distinguish between variants.
ifeq ($(BOOT),BOOTLOADER)
BOOT_OPTION:=BOOTLOADER=1
BOOTLOADER:=1
endif
ifeq ($(BOOT),BOOTLOADER_ENABLED)
BOOT_OPTION:=BOOTLOADER_ENABLED=1
BOOTLOADER_ENABLED:=1
endif
#---------------------------------------------------------------------------------------------------------

#---------------------------------------------------------------------------------------------------------
# ADDITION TO THE VARIANTS TO BUILD
#---------------------------------------------------------------------------------------------------------
# Add dimension "LIB16" to the build array
#
# LIB16 variant must be added in front of LIST_OF_VARIANTS to be handled last,
# and for the filtering below in this Makefile to work. (LIST_OF_VARIANTS+= LIB16 # does not work correctly)
LIST_OF_VARIANTS:=LIB16 $(LIST_OF_VARIANTS)
# A full list of values for each VariantName must be assigned.
LIST_OF_LIB16:=nonLIB16 LIB16_ENABLED
# A default value for each VariantName must be assigned. They are used in the help texts.
LIB16_DEFAULT:=nonLIB16
# An option variable for each VariantName must be assigned to be considered during "make",
# and of course, the application source code must be prepared to distinguish between variants.
ifeq ($(LIB16),LIB16_ENABLED)
LIB16_OPTION:=LIB16_ENABLED=1
LIB16_ENABLED:=1
endif
#---------------------------------------------------------------------------------------------------------


#---------------------------------------------------------------------------------------------------------
# Filtering out targets, which have no meaning, or are just not needed.
#
# The controlling variant name must be listed before the controlled variant name in the LIST_OF_VARIANTS
# for this to work.
#
ifneq ($(LIB16_ENABLED),)
LIST_OF_BOOT:=nonBOOT
LIST_OF_LIBRARY:=controller_bridge_16\
		 controller_portable_16\
		 controller_static_16\
		 controller_static_norep_16\
		 controller_static_single_16\
		 slave_enhanced_232_16
ifneq ($(IMA_ENABLED),)
LIST_OF_LIBRARY:=controller_bridge_16 controller_static_16 controller_static_norep_16
endif
ifeq ($(BOARD),ZM5202)
# This ZM5202 module does not support USB
LIST_OF_HOST_INTERFACE:=UART
endif
ifeq ($(TESTBOARD),1)
ifneq ($(IMA_ENABLED),)
LIST_OF_LIBRARY:=controller_bridge_16 controller_static_16 controller_static_norep_16
else
LIST_OF_LIBRARY:=controller_bridge_16 controller_static_16 controller_static_norep_16 controller_static_single_16
endif
endif

else	# ifneq($LIB16_ENABLED))

#---------------------------------------------------------------------------------------------------------
# Filtering out targets, which have no meaning, or are just not needed.
#
# The controlling variant name must be listed before the controlled variant name in the LIST_OF_VARIANTS
# for this to work.
#
ifneq ($(IMA_ENABLED),)
LIST_OF_LIBRARY:=controller_bridge controller_static controller_static_norep
endif
ifeq ($(BOARD),ZM5202)
# This ZM5202 module does not support USB
LIST_OF_HOST_INTERFACE:=UART
endif
ifeq ($(TESTBOARD),1)
ifneq ($(IMA_ENABLED),)
LIST_OF_LIBRARY:=controller_bridge controller_static controller_static_norep
else
LIST_OF_LIBRARY:=controller_bridge controller_static controller_static_norep controller_static_single
endif
endif
#---------------------------------------------------------------------------------------------------------

endif	# ifneq($LIB16_ENABLED))

#---------------------------------------------------------------------------------------------------------
# Filtering out targets, which have no meaning, or are just not needed.
#
# The controlling variant name must be listed before the controlled variant name in the LIST_OF_VARIANTS
# for this to work.
#
ifneq ($(BOOTLOADER_ENABLED),)
LIST_OF_LIBRARY:=\
 controller_bridge\
 controller_portable\
 controller_static\
 controller_static_norep\
 slave_enhanced_232
endif
#---------------------------------------------------------------------------------------------------------

#---------------------------------------------------------------------------------------------------------
# Filtering out targets, which have no meaning, or are just not needed.
#
# The controlling variant name must be listed before the controlled variant name in the LIST_OF_VARIANTS
# for this to work.
#
#LIST_OF_VARIANTS:=BOARD HOST_INTERFACE LIBRARY FREQUENCY CODE_MEMORY_MODE SENSOR_TYPE CHIP
ifneq ($(BOOTLOADER),)
LIST_OF_IMA:=nonIMA
LIST_OF_HOST_INTERFACE:=UART
LIST_OF_LIBRARY:=nolib
LIST_OF_FREQUENCY:=EU
LIST_OF_CODE_MEMORY_MODE=nonbanking
LIST_OF_MAKESCHEME:=NO_SCHEME
endif
#---------------------------------------------------------------------------------------------------------

UVBATHEADERFILES:=
UVBATHEADERFILESPLUS:=
# "uVision" project creation is optional.
# use "make UVISION=1 ZWxxxxx...."
# if $(UVISION) equals 1 or 2
ifeq ($(UVISION),$(filter $(UVISION),1 2))

# If we are building bootloader then do not generate lib_supported_func.h
ifeq ($(BOOTLOADER),)
# If we are building a patch, then don't generate lib_supported_func.h
ifeq ($(PATCH),)
UVBATHEADERFILES:=$(LIB)_supported_func.huvbat
UVBATHEADERFILESPLUS:=$(UVBATHEADERFILES) +
endif
endif
$(LIB)_supported_func.huvbat: $(LIB)_supported_func.h
	@echo $(UVISIONPROJECTGENERATOR) TARGET="$(TARGET)" FREQUENCY="$(FREQUENCY)" RELSDIR="$(RELSDIR)" LISTDIR="$(LISTDIR)" CREATE_SUPPORTED_FUNC="$(LIBFILE)"  > $@

endif

-include Makefile.common_ZW0x0x_supported_functions

# Generate SerialAPI defines for supported functions based on what exists in library.
lib_supported_func.h: $(LIB)_supported_func.h $(UVBATHEADERFILES)
	@echo #include "$(LIB)_supported_func.h" > $(subst \,\\,$(RELSDIRFULL)\lib_supported_func.h)
$(LIB)_supported_func.h: $(LIBFILE)
	make-supported-functions-include.bat $(subst \,\\,$(LIBFILE)) $(subst \,\\,$(TARGETBASEDIRFULL)\$(LIB)_supported_func.h)

# Make sure lib_supported_func.h is generated before we generate dependencies
Makefile.d: lib_supported_func.h

CDEFINES+=,\
 ZW_ID_SERVER,\
 SUC_SUPPORT

CDEFINES+=,\
 $(BOARD)

ifeq ($(IF),USBVCP)
CDEFINES+=,\
 $(IF)
endif

## Missing C defines common for all ZW0102 based controller libraries should be defined below
CDEFINES+=,\
 ZW_EXPLORE

# ifeq ("$(APP)$(CHIP)", "bridgeZW040x")
# CDEFINES+=,REDUCE_BRIDGE
# else
ifeq ("$(APP)$(CHIP)", "bridgeZW030x")
CDEFINES+=,REDUCE_BRIDGE
else
ifeq ("$(APP)$(CHIP)", "bridgeZW020x")
CDEFINES+=,REDUCE_BRIDGE
else
ifeq ("$(APP)", "controller_single")
else
CDEFINES+=,ENABLE_LEDS
endif
endif
endif
# endif

ifeq ($(BOARD),ZM4102)
CDEFINES+=,\
 ZM4102
ADEFINES+=,\
 ZM4102
endif

ifeq ($(TESTBOARD),1)
CDEFINES+=,\
 APPL_PROD_TEST
ADEFINES+=,\
 APPL_PROD_TEST
endif

ifneq ($(IMA_ENABLED),)
CDEFINES+=,\
 IMA_ENABLED
endif

.PHONY:\
 Makefile.TestBoards
