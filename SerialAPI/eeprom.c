/****************************************************************************
 *
 * Copyright (c) 2001-2011
 * Sigma Designs, Inc.
 * All Rights Reserved
 *
 *---------------------------------------------------------------------------
 *
 * Description: NVM layout for all node types
 *
 * Author:  Thomas Roll
 *
 * Last Changed By:  $Author: efh $
 * Revision:         $Revision: 11509 $
 * Last Changed:     $Date: 2012-03-05 10:45:45 +0200 (Wed, 5 Mar 2012) $
 *
 ****************************************************************************/

/* Make sure compiler won't shuffle around with these variables,            */
/* as there are external dependencies.                                      */
/* All these variables needs to initialized, because the compiler groups    */
/* the variables into different classes for uninitialized/initialized       */
/* when ordering them. To keep them in order... Keep them all initialized.  */
#pragma ORDER

/****************************************************************************/
/*                              INCLUDE FILES                               */
#include <ZW_basis_api.h>
#include <eeprom.h>


/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/

/***********************/
/*      NVM layout     */
/***********************/
BYTE far EEOFFSET_MAGIC_far;
#ifdef ZW_SLAVE
BYTE far EEOFFSET_LISTENING_far;
BYTE far EEOFFSET_GENERIC_far;
BYTE far EEOFFSET_SPECIFIC_far;
BYTE far EEOFFSET_CMDCLASS_LEN_far;
#else
BYTE far EEOFFSET_CMDCLASS_LEN_far;
#endif
BYTE far EEOFFSET_CMDCLASS_far[APPL_NODEPARM_MAX];
BYTE far EEOFFSET_WATCHDOG_STARTED_far;

#ifdef ZW_SLAVE
BYTE far EEOFFSET_SERIALAPI_DUMMY_BUFFER_far[NVM_SERIALAPI_SIZE - (5 + APPL_NODEPARM_MAX + 1)];
#else
BYTE far EEOFFSET_SERIALAPI_DUMMY_BUFFER_far[NVM_SERIALAPI_SIZE - (2 + APPL_NODEPARM_MAX + 1)];
#endif

BYTE far EEOFFSET_HOST_OFFSET_START_far[NVM_SERIALAPI_HOST_SIZE];
