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
 * Author:   Thomas Roll
 *
 * Last Changed By:  $Author: tro $
 * Revision:         $Revision: 23236 $
 * Last Changed:     $Date: 2012-08-17 14:04:49 +0200 (fr, 17 aug 2012) $
 *
 ****************************************************************************/
#ifndef _EEPROM_H_
#define _EEPROM_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <serialappl.h>
#include <ZW_nvm_addr.h>
#include <ZW_nvm_descriptor.h>

/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/

/* NVM allocation definitions */

#if defined(ZW_CONTROLLER) || defined(ZW_SLAVE_32)
/* NVM descriptor lies at the end of defined NVM data */
/* NVM Descriptor size is 12Bytes */
#define NVM_DESCRIPTOR_SIZE     sizeof(t_nvmDescriptor)
/* Setup the minimum size of HOST available dataspace - 64Bytes is allocated for SerialAPI local use */
#define NVM_SERIALAPI_SIZE   0x40
#ifdef NVM_16KB
/* NVM is 16KB and protocol usage is 12KB -> 4KB - 64B - 12 = 4020Bytes for HOST */
#define NVM_SERIALAPI_HOST_SIZE (0x4000 - NVM_LIB_SIZE - NVM_SERIALAPI_SIZE - NVM_DESCRIPTOR_SIZE)
#else
/* Size is based on lowest common Application NVM space */
/* - for NVM sizes bigger than 32KB the HOST can address to the full 64KB range, which is */
/* possible with the SerialAPI NVM Memory functionality, but HOST must have total knowledge */
/* of the NVM size and how much can be used because if HOST tries to address outside of */
/* the physical NVM then writes/reads will wrap around and potentially destroy protocol data */
/* Assuming NVM is minimum 32KB and protocol usage is 24KB -> 8KB - 64B - 12B = 8116Bytes minimum for HOST */
#define NVM_SERIALAPI_HOST_SIZE (0x8000 - NVM_LIB_SIZE - NVM_SERIALAPI_SIZE - NVM_DESCRIPTOR_SIZE)
#endif
#else
/* For routing slaves the total number of data byte available for Application is 64Bytes out of 255Bytes */
/* 48Bytes allocated for SerialAPI local usage */
#define NVM_SERIALAPI_SIZE   0x30
/* Out of these are 48Bytes allocated for SerialAPI local usage -> 64B - 48B = 16Bytes for HOST */
#define NVM_SERIALAPI_HOST_SIZE (0xFF - NVM_LIB_SIZE - NVM_SERIALAPI_SIZE)
#endif

/* Define EEPROM offsets for embedded application data */
extern BYTE far EEOFFSET_MAGIC_far;
#ifdef ZW_SLAVE
extern BYTE far EEOFFSET_LISTENING_far;
extern BYTE far EEOFFSET_GENERIC_far;
extern BYTE far EEOFFSET_SPECIFIC_far;
extern BYTE far EEOFFSET_CMDCLASS_LEN_far;
#else
extern BYTE far EEOFFSET_CMDCLASS_LEN_far;
#endif
extern BYTE far EEOFFSET_CMDCLASS_far[APPL_NODEPARM_MAX];
extern BYTE far EEOFFSET_WATCHDOG_STARTED_far;

extern BYTE far EEOFFSET_SERIALAPI_DUMMY_BUFFER_far[];

extern BYTE far EEOFFSET_HOST_OFFSET_START_far[];

#define MAGIC_VALUE       0x42

#endif /* _EEPROM_H_ */
