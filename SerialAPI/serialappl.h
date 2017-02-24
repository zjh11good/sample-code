/****************************************************************************
 *
 * Copyright (c) 2001-2013
 * Sigma Designs, Inc.
 * All Rights Reserved
 *
 *---------------------------------------------------------------------------
 *
 * Description:       Header file for Serial API implementation
 *                    Contains various application definitions and SerialAPI
 *                    functionality support definitions
 *
 * Last Changed By:  $Author: jsi $
 * Revision:         $Revision: 29065 $
 * Last Changed:     $Date: 2014-06-24 16:16:19 +0200 (Tue, 24 Jun 2014) $
 *
 ****************************************************************************/
#ifndef _SERIALAPPL_H_
#define _SERIALAPPL_H_


/* Z-Wave library functionality support definitions */
#include <lib_supported_func.h>
#include "config_app.h"
#include <ZW_SerialAPI.h>

/* Serial API application manufacturer_id */
#define SERIALAPI_MANUFACTURER_ID1 0  /* MSB */
#define SERIALAPI_MANUFACTURER_ID2 0  /* LSB */
/* Serial API application manufacturer product type */
#define SERIALAPI_MANUFACTURER_PRODUCT_TYPE1 0 /* MSB */
#define SERIALAPI_MANUFACTURER_PRODUCT_TYPE2 1 /* LSB */
/* Serial API application manufacturer product id */
#define SERIALAPI_MANUFACTURER_PRODUCT_ID1 0 /* MSB */
#define SERIALAPI_MANUFACTURER_PRODUCT_ID2 1 /* LSB */

/* Serial API version */
#define SERIAL_API_VER 5

/* Max number of times a frame will be transmitted to PC */
#define MAX_SERIAL_RETRY 3

/* Number of bytes in a homeID */
#define HOMEID_LENGTH 4

/* Max number of nodes in a multi cast (group) */
#define MAX_GROUP_NODES 64

/* Macro for accessing the byte in byte_array at the index indx */
#define BYTE_IN_AR(byte_array, indx) (*(byte_array + indx))

/* Macro for getting HIGH BYTE in wVar WORD variable */
#define BYTE_GET_HIGH_BYTE_IN_WORD(wVar) (BYTE)*((BYTE*)&wVar)

/* Macro for getting LOW BYTE in wVar WORD variable */
#define BYTE_GET_LOW_BYTE_IN_WORD(wVar) (BYTE)*((BYTE*)&wVar + 1)

/* Macro for setting HIGH BYTE and LOW BYTE in wVar WORD variable */
#define WORD_SET_HIGH_LOW_BYTES(wVar, bHIGHByte, bLOWByte) BYTE_GET_HIGH_BYTE_IN_WORD(wVar) = bHIGHByte; \
  					                                               BYTE_GET_LOW_BYTE_IN_WORD(wVar)  = bLOWByte

/* States for ApplicationPoll function */
enum
{
  stateIdle,
  stateTxSerial,
  stateFrameParse,
  stateCbTxSerial
};

/* params used by ApplicationNodeInformation */
#define APPL_NODEPARM_MAX       35
#define APPL_SLAVENODEPARM_MAX  APPL_NODEPARM_MAX

#ifdef ZW_SLAVE_ROUTING
/* SerialAPI only used state - used when ZW_RequestNodeInfo transmit fails */
/* It is then assumed that the destination node did not receive the request. */
#define UPDATE_STATE_NODE_INFO_REQ_FAILED   0x81
#endif

/* SerialAPI functionality support definitions */
#ifdef IMA_ENABLED
#define SUPPORT_SEND_DATA_TIMING                        1
#define SUPPORT_ADD_REMOVE_PROTECT                      1
#else
#define SUPPORT_SEND_DATA_TIMING                        0
#define SUPPORT_ADD_REMOVE_PROTECT                      0
#endif

/* Definitions for SerialAPI startup */
typedef enum
{
  SERIALAPI_CONFIG_STARTUP_NOTIFICATION_ENABLED = 1,
  SERIALAPI_CONFIG_UNDEFINED = 0xFE
} SERIALAPI_CONFIG_T;

#ifdef IMA_ENABLED
/* Enable support for SerialAPI Startup Notification */
#define SUPPORT_SERIAL_API_STARTUP_NOTIFICATION         1
#else
#define SUPPORT_SERIAL_API_STARTUP_NOTIFICATION         0
#endif

/* Bootloader Firmware update functionality support definitions */

/* Enum definitions for Firmware Update functionality selector; firmwareUpdateFunction */
typedef enum
{
  FIRMWARE_UPDATE_NVM_INIT = 0,
  FIRMWARE_UPDATE_NVM_SET_NEW_IMAGE = 1,
  FIRMWARE_UPDATE_NVM_GET_NEW_IMAGE = 2,
  FIRMWARE_UPDATE_NVM_UPDATE_CRC16 = 3,
  FIRMWARE_UPDATE_NVM_IS_VALID_CRC16 = 4,
  FIRMWARE_UPDATE_NVM_WRITE = 5,
  FIRMWARE_UPDATE_NVM_UNKNOWN = 0xFF
} FIRMWARE_UPDATE_NVM_T;

#ifdef BOOTLOADER_ENABLED
#define SUPPORT_ZW_FIRMWARE_UPDATE_NVM			            1
#else
#define SUPPORT_ZW_FIRMWARE_UPDATE_NVM			            0
#endif

/* Common SerialAPI functionality support definitions */
#define SUPPORT_SERIAL_API_GET_INIT_DATA                1
#define SUPPORT_SERIAL_API_APPL_NODE_INFORMATION        1
#ifdef ZW_CONTROLLER_BRIDGE
#define SUPPORT_APPLICATION_COMMAND_HANDLER_BRIDGE      1
#define SUPPORT_APPLICATION_COMMAND_HANDLER             0
#else
#define SUPPORT_APPLICATION_COMMAND_HANDLER_BRIDGE      0
#define SUPPORT_APPLICATION_COMMAND_HANDLER             1
#endif

#define SUPPORT_SERIAL_API_SET_TIMEOUTS                 1

#define SUPPORT_SERIAL_API_GET_CAPABILITIES             1
#define SUPPORT_SERIAL_API_SOFT_RESET                   1

#ifdef ZW_CONTROLLER_SINGLE
#define SUPPORT_SERIAL_API_POWER_MANAGEMENT             0
#define SUPPORT_SERIAL_API_READY                        0
#else
#define SUPPORT_SERIAL_API_POWER_MANAGEMENT             1
#define SUPPORT_SERIAL_API_READY                        1
#endif

#define SUPPORT_SERIAL_API_EXT                          1

#ifdef ZW_ENABLE_RTC
#define SUPPORT_CLOCK_SET                               1
#define SUPPORT_CLOCK_GET                               1
#define SUPPORT_CLOCK_CMP                               1
#define SUPPORT_RTC_TIMER_CREATE                        1
#define SUPPORT_RTC_TIMER_READ                          1
#define SUPPORT_RTC_TIMER_DELETE                        1
#define SUPPORT_RTC_TIMER_CALL                          1
#else
#define SUPPORT_CLOCK_SET                               0
#define SUPPORT_CLOCK_GET                               0
#define SUPPORT_CLOCK_CMP                               0
#define SUPPORT_RTC_TIMER_CREATE                        0
#define SUPPORT_RTC_TIMER_READ                          0
#define SUPPORT_RTC_TIMER_DELETE                        0
#define SUPPORT_RTC_TIMER_CALL                          0
#endif

#define SUPPORT_ZW_AUTO_PROGRAMMING                     1

#ifdef TIMER_SUPPORT
#define SUPPORT_TIMER_START                             1
#define SUPPORT_TIMER_RESTART                           1
#define SUPPORT_TIMER_CANCEL                            1
#define SUPPORT_TIMER_CALL                              1
#else
#define SUPPORT_TIMER_START                             0
#define SUPPORT_TIMER_RESTART                           0
#define SUPPORT_TIMER_CANCEL                            0
#define SUPPORT_TIMER_CALL                              0
#endif

#if defined(NUNIT_TEST) && !defined(ZW_CONTROLLER_BRIDGE)
#define SUPPORT_ZW_NUNIT                                1
#else
#define SUPPORT_ZW_NUNIT                                0
#endif

#ifdef PORT_STATUS
#define SUPPORT_ZW_PORT_STATUS                          1
#else
#define SUPPORT_ZW_PORT_STATUS                          0
#endif
/* ZW_EnableSUC() no longer exists in the library */

/* */
#define SUPPORT_SERIAL_API_GET_APPL_HOST_MEMORY_OFFSET  0

#undef SUPPORT_ZW_SET_LEARN_NODE_STATE
#define SUPPORT_ZW_SET_LEARN_NODE_STATE                 0 /* ZW_SetLearnNodeState */

#ifdef ZW_CONTROLLER
/**************************************************************************/
/* Common for all Controllers */
/* SerialAPI functionality support definitions */
#define SUPPORT_ZW_APPLICATION_CONTROLLER_UPDATE        1
#define SUPPORT_ZW_APPLICATION_UPDATE                   1


/**************************************************************************/
/* Controller */
/* Specific SerialAPI functionality support definitions */
#if !defined(ZW_CONTROLLER_STATIC) && !defined(ZW_INSTALLER)
#define SUPPORT_GET_TX_COUNTER                          0
#define SUPPORT_RESET_TX_COUNTER                        0

#define SUPPORT_SERIAL_API_APPL_SLAVE_NODE_INFORMATION  0
#define SUPPORT_APPLICATION_SLAVE_COMMAND_HANDLER       0
#define SUPPORT_ZW_SEND_SLAVE_NODE_INFORMATION          0

#ifdef ZW_CONTROLLER_SINGLE
#define SUPPORT_SERIAL_API_TEST                         0
#endif

/* Not supported by any controllers except controller portable and installer libs */

#endif /* !ZW_CONTROLLER_STATIC) && !ZW_INSTALLER */


/**************************************************************************/
/* Installer Controller */
/* Specific SerialAPI functionality support definitions */
#if defined(ZW_INSTALLER)

#define SUPPORT_GET_TX_COUNTER                          1
#define SUPPORT_RESET_TX_COUNTER                        1

#define SUPPORT_SERIAL_API_APPL_SLAVE_NODE_INFORMATION  0
#define SUPPORT_APPLICATION_SLAVE_COMMAND_HANDLER       0
#define SUPPORT_ZW_SEND_SLAVE_NODE_INFORMATION          0

#ifdef ZW_CONTROLLER_SINGLE
#define SUPPORT_SERIAL_API_TEST                         0
#endif

/* Not supported by any controllers except controller portable and installer libs */

#endif  /* ZW_INSTALLER */


/**************************************************************************/
/* Static Controller */
/* specific SerialAPI functionality support definitions */
#if defined(ZW_CONTROLLER_STATIC) && !defined(ZW_CONTROLLER_BRIDGE)

#define SUPPORT_GET_TX_COUNTER                          0
#define SUPPORT_RESET_TX_COUNTER                        0

#define SUPPORT_SERIAL_API_APPL_SLAVE_NODE_INFORMATION  0
#define SUPPORT_APPLICATION_SLAVE_COMMAND_HANDLER       0
#define SUPPORT_ZW_SEND_SLAVE_NODE_INFORMATION          0

#ifdef ZW_CONTROLLER_SINGLE
#define SUPPORT_SERIAL_API_TEST                         1
#endif

/* Not supported by any controllers except controller portable and installer libs */

#endif  /* ZW_CONTROLLER_STATIC) && !ZW_CONTROLLER_BRIDGE */


/**************************************************************************/
/* Bridge Controller */
/* specific SerialAPI functionality support definitions */
#if defined(ZW_CONTROLLER_BRIDGE)

#undef SUPPORT_ZW_SEND_DATA
#define SUPPORT_ZW_SEND_DATA                            1 /* ZW_SendData */

#define SUPPORT_GET_TX_COUNTER                          0
#define SUPPORT_RESET_TX_COUNTER                        0

#define SUPPORT_SERIAL_API_APPL_SLAVE_NODE_INFORMATION  1
#if SUPPORT_APPLICATION_COMMAND_HANDLER_BRIDGE
#define SUPPORT_APPLICATION_SLAVE_COMMAND_HANDLER       0
#else
#define SUPPORT_APPLICATION_SLAVE_COMMAND_HANDLER       1
#endif
#define SUPPORT_ZW_SEND_SLAVE_NODE_INFORMATION          1

#ifdef ZW_CONTROLLER_SINGLE
#define SUPPORT_SERIAL_API_TEST                         0
#endif

/* Not supported by any controllers except controller portable and installer libs */

#endif  /* ZW_CONTROLLER_BRIDGE */

#endif  /* ZW_CONTROLLER */


#ifdef ZW_SLAVE
/**************************************************************************/
/* Common for all slaves */
/* SerialAPI functionality support definitions */
#define SUPPORT_ZW_APPLICATION_UPDATE                   1
#define SUPPORT_APPLICATION_RF_NOTIFY                   0

#ifdef ZW_SLAVE_32
/**************************************************************************/
/* Slave enhanced */
/* Specific SerialAPI functionality support definitions */

#define SUPPORT_ZW_APPLICATION_CONTROLLER_UPDATE        0

#define SUPPORT_GET_TX_COUNTER                          0
#define SUPPORT_RESET_TX_COUNTER                        0

#define SUPPORT_SERIAL_API_APPL_SLAVE_NODE_INFORMATION  0
#define SUPPORT_APPLICATION_SLAVE_COMMAND_HANDLER       0
#define SUPPORT_ZW_SEND_SLAVE_NODE_INFORMATION          0

#ifdef ZW_CONTROLLER_SINGLE
#define SUPPORT_SERIAL_API_TEST                         0
#endif

#endif  /* ZW_SLAVE_32 */


#if !defined(ZW_SLAVE_32) && defined(ZW_SLAVE_ROUTING)
/**************************************************************************/
/* Slave routing */
/* Specific SerialAPI functionality support definitions */

#define SUPPORT_ZW_APPLICATION_CONTROLLER_UPDATE        0

#define SUPPORT_GET_TX_COUNTER                          0
#define SUPPORT_RESET_TX_COUNTER                        0

#define SUPPORT_SERIAL_API_APPL_SLAVE_NODE_INFORMATION  0
#define SUPPORT_APPLICATION_SLAVE_COMMAND_HANDLER       0
#define SUPPORT_ZW_SEND_SLAVE_NODE_INFORMATION          0

#endif  /* !ZW_SLAVE_32 && ZW_SLAVE_ROUTING */


#if !defined(ZW_SLAVE_ROUTING) && !defined(ZW_SLAVE_32)
/**************************************************************************/
/* Slave */
/* Specific SerialAPI functionality support definitions */

#define SUPPORT_ZW_APPLICATION_CONTROLLER_UPDATE        0

#define SUPPORT_GET_TX_COUNTER                          0
#define SUPPORT_RESET_TX_COUNTER                        0

#define SUPPORT_SERIAL_API_APPL_SLAVE_NODE_INFORMATION  0
#define SUPPORT_APPLICATION_SLAVE_COMMAND_HANDLER       0
#define SUPPORT_ZW_SEND_SLAVE_NODE_INFORMATION          0

#ifdef ZW_CONTROLLER_SINGLE
#define SUPPORT_SERIAL_API_TEST                         0
#endif

#endif  /* !ZW_SLAVE_ROUTING && !ZW_SLAVE_32 */

#endif /* ZW_SLAVE */


#define FID_BM_OFFS(FUNCID) ((FUNCID - 1) / 8)
#define FID_BM(FUNCID) (1 << ((FUNCID - 1) & 0x07))
#define CAP_FID(OFFSET, FUNCID) (((FUNCID != 0) && (FID_BM_OFFS(FUNCID) == OFFSET)) ? FID_BM(FUNCID) : 0)
#ifdef ZW_CONTROLLER_SINGLE
#define CAP_BM(OFFSET) (CAP_FID(OFFSET, (SUPPORT_SERIAL_API_POWER_MANAGEMENT == 0) ? 0 : FUNC_ID_SERIAL_API_POWER_MANAGEMENT) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_READY == 0) ? 0 : FUNC_ID_SERIAL_API_READY) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_GET_INIT_DATA == 0) ? 0 : FUNC_ID_SERIAL_API_GET_INIT_DATA) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_APPL_NODE_INFORMATION == 0) ? 0 : FUNC_ID_SERIAL_API_APPL_NODE_INFORMATION) | \
                        CAP_FID(OFFSET, (SUPPORT_APPLICATION_COMMAND_HANDLER_BRIDGE == 0) ? 0 : FUNC_ID_APPLICATION_COMMAND_HANDLER_BRIDGE) | \
                        CAP_FID(OFFSET, (SUPPORT_APPLICATION_COMMAND_HANDLER == 0) ? 0 : FUNC_ID_APPLICATION_COMMAND_HANDLER) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_CONTROLLER_CAPABILITIES == 0) ? 0 : FUNC_ID_ZW_GET_CONTROLLER_CAPABILITIES) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_SET_TIMEOUTS == 0) ? 0 : FUNC_ID_SERIAL_API_SET_TIMEOUTS) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_GET_CAPABILITIES == 0) ? 0 : FUNC_ID_SERIAL_API_GET_CAPABILITIES) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_SOFT_RESET == 0) ? 0 : FUNC_ID_SERIAL_API_SOFT_RESET) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_STARTUP_NOTIFICATION == 0) ? 0 : FUNC_ID_SERIALAPI_STARTED) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_PROTOCOL_VERSION == 0) ? 0 : FUNC_ID_ZW_GET_PROTOCOL_VERSION) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_RF_RECEIVE_MODE == 0) ? 0 : FUNC_ID_ZW_SET_RF_RECEIVE_MODE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_SLEEP_MODE == 0) ? 0 : FUNC_ID_ZW_SET_SLEEP_MODE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_NODE_INFORMATION == 0) ? 0 : FUNC_ID_ZW_SEND_NODE_INFORMATION) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA == 0) ? 0 : FUNC_ID_ZW_SEND_DATA) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_BRIDGE == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_BRIDGE)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_META_BRIDGE == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_META_BRIDGE)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_MULTI == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_MULTI) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_MULTI_BRIDGE == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_MULTI_BRIDGE)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_VERSION == 0) ? 0 : FUNC_ID_ZW_GET_VERSION) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_ABORT == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_ABORT) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_RF_POWER_LEVEL_SET == 0) ? 0 : FUNC_ID_ZW_RF_POWER_LEVEL_SET) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_RF_POWER_LEVEL_GET == 0) ? 0 : FUNC_ID_ZW_RF_POWER_LEVEL_GET) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_TYPE_LIBRARY == 0) ? 0 : FUNC_ID_ZW_TYPE_LIBRARY) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_PROTOCOL_STATUS == 0) ? 0 : FUNC_ID_ZW_GET_PROTOCOL_STATUS) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_WATCHDOG_ENABLE == 0) ? 0 : FUNC_ID_ZW_WATCHDOG_ENABLE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_WATCHDOG_DISABLE == 0) ? 0 : FUNC_ID_ZW_WATCHDOG_DISABLE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_WATCHDOG_KICK == 0) ? 0 : FUNC_ID_ZW_WATCHDOG_KICK) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_WATCHDOG_START == 0) ? 0 : FUNC_ID_ZW_WATCHDOG_START)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_WATCHDOG_STOP == 0) ? 0 : FUNC_ID_ZW_WATCHDOG_STOP)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_WUT_TIMEOUT == 0) ? 0 : FUNC_ID_ZW_SET_WUT_TIMEOUT) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_IS_WUT_KICKED == 0) ? 0 : FUNC_ID_ZW_IS_WUT_KICKED) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_EXT_INT_LEVEL == 0) ? 0 : FUNC_ID_ZW_SET_EXT_INT_LEVEL) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_META == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_META) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_TEST_FRAME == 0) ? 0 : FUNC_ID_ZW_SEND_TEST_FRAME) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_RANDOM == 0) ? 0 : FUNC_ID_ZW_GET_RANDOM) | \
                        CAP_FID(OFFSET, (SUPPORT_MEMORY_GET_ID == 0) ? 0 : FUNC_ID_MEMORY_GET_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_MEMORY_GET_BYTE == 0) ? 0 : FUNC_ID_MEMORY_GET_BYTE) | \
                        CAP_FID(OFFSET, (SUPPORT_MEMORY_PUT_BYTE == 0) ? 0 : FUNC_ID_MEMORY_PUT_BYTE) | \
                        CAP_FID(OFFSET, (SUPPORT_MEMORY_GET_BUFFER == 0) ? 0 : FUNC_ID_MEMORY_GET_BUFFER) | \
                        CAP_FID(OFFSET, (SUPPORT_MEMORY_PUT_BUFFER == 0) ? 0 : FUNC_ID_MEMORY_PUT_BUFFER) | \
                        CAP_FID(OFFSET, (SUPPORT_NVM_GET_ID == 0) ? 0 : FUNC_ID_NVM_GET_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_NVM_EXT_READ_LONG_BYTE == 0) ? 0 : FUNC_ID_NVM_EXT_READ_LONG_BYTE) | \
                        CAP_FID(OFFSET, (SUPPORT_NVM_EXT_WRITE_LONG_BYTE == 0) ? 0 : FUNC_ID_NVM_EXT_WRITE_LONG_BYTE) | \
                        CAP_FID(OFFSET, (SUPPORT_NVM_EXT_READ_LONG_BUFFER == 0) ? 0 : FUNC_ID_NVM_EXT_READ_LONG_BUFFER) | \
                        CAP_FID(OFFSET, (SUPPORT_NVM_EXT_WRITE_LONG_BUFFER == 0) ? 0 : FUNC_ID_NVM_EXT_WRITE_LONG_BUFFER) | \
                        CAP_FID(OFFSET, (SUPPORT_CLOCK_SET == 0) ? 0 : FUNC_ID_CLOCK_SET) | \
                        CAP_FID(OFFSET, (SUPPORT_CLOCK_GET == 0) ? 0 : FUNC_ID_CLOCK_GET) | \
                        CAP_FID(OFFSET, (SUPPORT_CLOCK_CMP == 0) ? 0 : FUNC_ID_CLOCK_CMP) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_NVR_GET_VALUE == 0) ? 0 : FUNC_ID_NVR_GET_VALUE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_AUTO_PROGRAMMING == 0) ? 0 : FUNC_ID_AUTO_PROGRAMMING) | \
                        CAP_FID(OFFSET, (SUPPORT_PWR_SETSTOPMODE == 0) ? 0 : FUNC_ID_PWR_SETSTOPMODE) | \
                        CAP_FID(OFFSET, (SUPPORT_PWR_CLK_PD == 0) ? 0 : FUNC_ID_PWR_CLK_PD) | \
                        CAP_FID(OFFSET, (SUPPORT_PWR_CLK_PUP == 0) ? 0 : FUNC_ID_PWR_CLK_PUP) | \
                        CAP_FID(OFFSET, (SUPPORT_PWR_SELECT_CLK == 0) ? 0 : FUNC_ID_PWR_SELECT_CLK) | \
                        CAP_FID(OFFSET, (SUPPORT_RTC_TIMER_CREATE == 0) ? 0 : FUNC_ID_RTC_TIMER_CREATE) | \
                        CAP_FID(OFFSET, (SUPPORT_RTC_TIMER_READ == 0) ? 0 : FUNC_ID_RTC_TIMER_READ) | \
                        CAP_FID(OFFSET, (SUPPORT_RTC_TIMER_DELETE == 0) ? 0 : FUNC_ID_RTC_TIMER_DELETE) | \
                        CAP_FID(OFFSET, (SUPPORT_RTC_TIMER_CALL == 0) ? 0 : FUNC_ID_RTC_TIMER_CALL) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_FIRMWARE_UPDATE_NVM == 0) ? 0 : FUNC_ID_ZW_FIRMWARE_UPDATE_NVM) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_LEARN_NODE_STATE == 0) ? 0 : FUNC_ID_ZW_SET_LEARN_NODE_STATE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_NODE_PROTOCOL_INFO == 0) ? 0 : FUNC_ID_ZW_GET_NODE_PROTOCOL_INFO) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_DEFAULT == 0) ? 0 : FUNC_ID_ZW_SET_DEFAULT) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_NEW_CONTROLLER == 0) ? 0 : FUNC_ID_ZW_NEW_CONTROLLER) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REPLICATION_COMMAND_COMPLETE == 0) ? 0 : FUNC_ID_ZW_REPLICATION_COMMAND_COMPLETE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REPLICATION_SEND_DATA == 0) ? 0 : FUNC_ID_ZW_REPLICATION_SEND_DATA) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_ASSIGN_RETURN_ROUTE == 0) ? 0 : FUNC_ID_ZW_ASSIGN_RETURN_ROUTE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_DELETE_RETURN_ROUTE == 0) ? 0 : FUNC_ID_ZW_DELETE_RETURN_ROUTE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REQUEST_NODE_NEIGHBOR_UPDATE == 0) ? 0 : FUNC_ID_ZW_REQUEST_NODE_NEIGHBOR_UPDATE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_NEIGHBOR_COUNT == 0) ? 0 : FUNC_ID_ZW_GET_NEIGHBOR_COUNT) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_ARE_NODES_NEIGHBOURS == 0) ? 0 : FUNC_ID_ZW_ARE_NODES_NEIGHBOURS) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_APPLICATION_CONTROLLER_UPDATE == 0) ? 0 : FUNC_ID_ZW_APPLICATION_UPDATE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_ADD_NODE_TO_NETWORK == 0) ? 0 : FUNC_ID_ZW_ADD_NODE_TO_NETWORK) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REMOVE_NODE_FROM_NETWORK == 0) ? 0 : FUNC_ID_ZW_REMOVE_NODE_FROM_NETWORK) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_CREATE_NEW_PRIMARY == 0) ? 0 : FUNC_ID_ZW_CREATE_NEW_PRIMARY) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_CONTROLLER_CHANGE == 0) ? 0 : FUNC_ID_ZW_CONTROLLER_CHANGE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REQUEST_NODE_INFO == 0) ? 0 : FUNC_ID_ZW_REQUEST_NODE_INFO) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REMOVE_FAILED_NODE_ID == 0) ? 0 : FUNC_ID_ZW_REMOVE_FAILED_NODE_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_IS_FAILED_NODE_ID == 0) ? 0 : FUNC_ID_ZW_IS_FAILED_NODE_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REPLACE_FAILED_NODE == 0) ? 0 : FUNC_ID_ZW_REPLACE_FAILED_NODE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_ROUTING_MAX == 0) ? 0 : FUNC_ID_ZW_GET_ROUTING_MAX) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_ROUTING_MAX == 0) ? 0 : FUNC_ID_ZW_SET_ROUTING_MAX) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_IS_PRIMARY_CTRL == 0) ? 0 : FUNC_ID_ZW_IS_PRIMARY_CTRL) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_AES_ECB == 0) ? 0 : FUNC_ID_ZW_AES_ECB) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_APPLICATION_UPDATE == 0) ? 0 : FUNC_ID_ZW_APPLICATION_UPDATE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_LEARN_MODE == 0) ? 0 : FUNC_ID_ZW_SET_LEARN_MODE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_EXPLORE_REQUEST_INCLUSION == 0) ? 0 : FUNC_ID_ZW_EXPLORE_REQUEST_INCLUSION) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_ASSIGN_SUC_RETURN_ROUTE == 0) ? 0 : FUNC_ID_ZW_ASSIGN_SUC_RETURN_ROUTE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_ENABLE_SUC == 0) ? 0 : FUNC_ID_ZW_ENABLE_SUC) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REQUEST_NETWORK_UPDATE == 0) ? 0 : FUNC_ID_ZW_REQUEST_NETWORK_UPDATE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_SUC_NODE_ID == 0) ? 0 : FUNC_ID_ZW_SET_SUC_NODE_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_DELETE_SUC_RETURN_ROUTE == 0) ? 0 : FUNC_ID_ZW_DELETE_SUC_RETURN_ROUTE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_SUC_NODE_ID == 0) ? 0 : FUNC_ID_ZW_GET_SUC_NODE_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_SUC_ID == 0) ? 0 : FUNC_ID_ZW_SEND_SUC_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_TIMER_START == 0) ? 0 : FUNC_ID_TIMER_START) | \
                        CAP_FID(OFFSET, (SUPPORT_TIMER_RESTART == 0) ? 0 : FUNC_ID_TIMER_RESTART) | \
                        CAP_FID(OFFSET, (SUPPORT_TIMER_CANCEL == 0) ? 0 : FUNC_ID_TIMER_CANCEL) | \
                        CAP_FID(OFFSET, (SUPPORT_TIMER_CALL == 0) ? 0 : FUNC_ID_TIMER_CALL) | \
                        CAP_FID(OFFSET, (SUPPORT_GET_ROUTING_TABLE_LINE == 0) ? 0 : FUNC_ID_GET_ROUTING_TABLE_LINE) | \
                        CAP_FID(OFFSET, (SUPPORT_GET_TX_COUNTER == 0) ? 0 : FUNC_ID_GET_TX_COUNTER) | \
                        CAP_FID(OFFSET, (SUPPORT_RESET_TX_COUNTER == 0) ? 0 : FUNC_ID_RESET_TX_COUNTER) | \
                        CAP_FID(OFFSET, (SUPPORT_STORE_NODEINFO == 0) ? 0 : FUNC_ID_STORE_NODEINFO) | \
                        CAP_FID(OFFSET, (SUPPORT_STORE_HOMEID == 0) ? 0 : FUNC_ID_STORE_HOMEID) | \
                        CAP_FID(OFFSET, (SUPPORT_LOCK_ROUTE_RESPONSE == 0) ? 0 : FUNC_ID_LOCK_ROUTE_RESPONSE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_LAST_WORKING_ROUTE == 0) ? 0 : FUNC_ID_ZW_GET_LAST_WORKING_ROUTE)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_LAST_WORKING_ROUTE == 0) ? 0 : FUNC_ID_ZW_SET_LAST_WORKING_ROUTE)|\
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_APPL_SLAVE_NODE_INFORMATION == 0) ? 0 : FUNC_ID_SERIAL_API_APPL_SLAVE_NODE_INFORMATION) | \
                        CAP_FID(OFFSET, (SUPPORT_APPLICATION_SLAVE_COMMAND_HANDLER == 0) ? 0 : FUNC_ID_APPLICATION_SLAVE_COMMAND_HANDLER) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_SLAVE_NODE_INFORMATION == 0) ? 0 : FUNC_ID_ZW_SEND_SLAVE_NODE_INFORMATION) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_SLAVE_DATA == 0) ? 0 : FUNC_ID_ZW_SEND_SLAVE_DATA) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_SLAVE_LEARN_MODE == 0) ? 0 : FUNC_ID_ZW_SET_SLAVE_LEARN_MODE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_VIRTUAL_NODES == 0) ? 0 : FUNC_ID_ZW_GET_VIRTUAL_NODES) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_IS_VIRTUAL_NODE == 0) ? 0 : FUNC_ID_ZW_IS_VIRTUAL_NODE) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_TEST == 0) ? 0 : FUNC_ID_SERIAL_API_TEST) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_EXT == 0) ? 0 : FUNC_ID_SERIAL_API_EXT) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_GET_APPL_HOST_MEMORY_OFFSET == 0) ? 0 : FUNC_ID_SERIAL_API_GET_APPL_HOST_MEMORY_OFFSET) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_NUNIT == 0) ? 0 : FUNC_ID_ZW_NUNIT_CMD) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_NUNIT == 0) ? 0 : FUNC_ID_ZW_NUNIT_INIT) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_NUNIT == 0) ? 0 : FUNC_ID_ZW_NUNIT_LIST) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_NUNIT == 0) ? 0 : FUNC_ID_ZW_NUNIT_RUN) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_NUNIT == 0) ? 0 : FUNC_ID_ZW_NUNIT_END) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_PORT_STATUS == 0) ? 0 : FUNC_ID_IO_PORT_STATUS)| \
                        CAP_FID(OFFSET, (SUPPORT_ZW_PORT_STATUS == 0) ? 0 : FUNC_ID_IO_PORT) \
                       )
#else
#define CAP_BM(OFFSET) (CAP_FID(OFFSET, (SUPPORT_SERIAL_API_POWER_MANAGEMENT == 0) ? 0 : FUNC_ID_SERIAL_API_POWER_MANAGEMENT) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_READY == 0) ? 0 : FUNC_ID_SERIAL_API_READY) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REDISCOVERY_NEEDED == 0) ? 0 : FUNC_ID_ZW_REDISCOVERY_NEEDED) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_GET_INIT_DATA == 0) ? 0 : FUNC_ID_SERIAL_API_GET_INIT_DATA) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_APPL_NODE_INFORMATION == 0) ? 0 : FUNC_ID_SERIAL_API_APPL_NODE_INFORMATION) | \
                        CAP_FID(OFFSET, (SUPPORT_APPLICATION_COMMAND_HANDLER_BRIDGE == 0) ? 0 : FUNC_ID_APPLICATION_COMMAND_HANDLER_BRIDGE) | \
                        CAP_FID(OFFSET, (SUPPORT_APPLICATION_COMMAND_HANDLER == 0) ? 0 : FUNC_ID_APPLICATION_COMMAND_HANDLER) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_CONTROLLER_CAPABILITIES == 0) ? 0 : FUNC_ID_ZW_GET_CONTROLLER_CAPABILITIES) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_SET_TIMEOUTS == 0) ? 0 : FUNC_ID_SERIAL_API_SET_TIMEOUTS) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_GET_CAPABILITIES == 0) ? 0 : FUNC_ID_SERIAL_API_GET_CAPABILITIES) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_SOFT_RESET == 0) ? 0 : FUNC_ID_SERIAL_API_SOFT_RESET) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_STARTUP_NOTIFICATION == 0) ? 0 : FUNC_ID_SERIALAPI_STARTED) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_PROTOCOL_VERSION == 0) ? 0 : FUNC_ID_ZW_GET_PROTOCOL_VERSION) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_RF_RECEIVE_MODE == 0) ? 0 : FUNC_ID_ZW_SET_RF_RECEIVE_MODE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_SLEEP_MODE == 0) ? 0 : FUNC_ID_ZW_SET_SLEEP_MODE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_NODE_INFORMATION == 0) ? 0 : FUNC_ID_ZW_SEND_NODE_INFORMATION) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA == 0) ? 0 : FUNC_ID_ZW_SEND_DATA) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_BRIDGE == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_BRIDGE)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_META_BRIDGE == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_META_BRIDGE)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_MULTI == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_MULTI) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_MULTI_BRIDGE == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_MULTI_BRIDGE)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_VERSION == 0) ? 0 : FUNC_ID_ZW_GET_VERSION) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_ABORT == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_ABORT) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_RF_POWER_LEVEL_SET == 0) ? 0 : FUNC_ID_ZW_RF_POWER_LEVEL_SET) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_RF_POWER_LEVEL_GET == 0) ? 0 : FUNC_ID_ZW_RF_POWER_LEVEL_GET) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_TYPE_LIBRARY == 0) ? 0 : FUNC_ID_ZW_TYPE_LIBRARY) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_PROTOCOL_STATUS == 0) ? 0 : FUNC_ID_ZW_GET_PROTOCOL_STATUS) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_WATCHDOG_ENABLE == 0) ? 0 : FUNC_ID_ZW_WATCHDOG_ENABLE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_WATCHDOG_DISABLE == 0) ? 0 : FUNC_ID_ZW_WATCHDOG_DISABLE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_WATCHDOG_KICK == 0) ? 0 : FUNC_ID_ZW_WATCHDOG_KICK) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_WATCHDOG_START == 0) ? 0 : FUNC_ID_ZW_WATCHDOG_START)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_WATCHDOG_STOP == 0) ? 0 : FUNC_ID_ZW_WATCHDOG_STOP)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_WUT_TIMEOUT == 0) ? 0 : FUNC_ID_ZW_SET_WUT_TIMEOUT) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_IS_WUT_KICKED == 0) ? 0 : FUNC_ID_ZW_IS_WUT_KICKED) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_EXT_INT_LEVEL == 0) ? 0 : FUNC_ID_ZW_SET_EXT_INT_LEVEL) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_DATA_META == 0) ? 0 : FUNC_ID_ZW_SEND_DATA_META) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_TEST_FRAME == 0) ? 0 : FUNC_ID_ZW_SEND_TEST_FRAME) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_RANDOM == 0) ? 0 : FUNC_ID_ZW_GET_RANDOM) | \
                        CAP_FID(OFFSET, (SUPPORT_MEMORY_GET_ID == 0) ? 0 : FUNC_ID_MEMORY_GET_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_MEMORY_GET_BYTE == 0) ? 0 : FUNC_ID_MEMORY_GET_BYTE) | \
                        CAP_FID(OFFSET, (SUPPORT_MEMORY_PUT_BYTE == 0) ? 0 : FUNC_ID_MEMORY_PUT_BYTE) | \
                        CAP_FID(OFFSET, (SUPPORT_MEMORY_GET_BUFFER == 0) ? 0 : FUNC_ID_MEMORY_GET_BUFFER) | \
                        CAP_FID(OFFSET, (SUPPORT_MEMORY_PUT_BUFFER == 0) ? 0 : FUNC_ID_MEMORY_PUT_BUFFER) | \
                        CAP_FID(OFFSET, (SUPPORT_NVM_GET_ID == 0) ? 0 : FUNC_ID_NVM_GET_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_NVM_EXT_READ_LONG_BYTE == 0) ? 0 : FUNC_ID_NVM_EXT_READ_LONG_BYTE) | \
                        CAP_FID(OFFSET, (SUPPORT_NVM_EXT_WRITE_LONG_BYTE == 0) ? 0 : FUNC_ID_NVM_EXT_WRITE_LONG_BYTE) | \
                        CAP_FID(OFFSET, (SUPPORT_NVM_EXT_READ_LONG_BUFFER == 0) ? 0 : FUNC_ID_NVM_EXT_READ_LONG_BUFFER) | \
                        CAP_FID(OFFSET, (SUPPORT_NVM_EXT_WRITE_LONG_BUFFER == 0) ? 0 : FUNC_ID_NVM_EXT_WRITE_LONG_BUFFER) | \
                        CAP_FID(OFFSET, (SUPPORT_CLOCK_SET == 0) ? 0 : FUNC_ID_CLOCK_SET) | \
                        CAP_FID(OFFSET, (SUPPORT_CLOCK_GET == 0) ? 0 : FUNC_ID_CLOCK_GET) | \
                        CAP_FID(OFFSET, (SUPPORT_CLOCK_CMP == 0) ? 0 : FUNC_ID_CLOCK_CMP) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_NVR_GET_VALUE == 0) ? 0 : FUNC_ID_NVR_GET_VALUE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_AUTO_PROGRAMMING == 0) ? 0 : FUNC_ID_AUTO_PROGRAMMING) | \
                        CAP_FID(OFFSET, (SUPPORT_PWR_SETSTOPMODE == 0) ? 0 : FUNC_ID_PWR_SETSTOPMODE) | \
                        CAP_FID(OFFSET, (SUPPORT_PWR_CLK_PD == 0) ? 0 : FUNC_ID_PWR_CLK_PD) | \
                        CAP_FID(OFFSET, (SUPPORT_PWR_CLK_PUP == 0) ? 0 : FUNC_ID_PWR_CLK_PUP) | \
                        CAP_FID(OFFSET, (SUPPORT_PWR_SELECT_CLK == 0) ? 0 : FUNC_ID_PWR_SELECT_CLK) | \
                        CAP_FID(OFFSET, (SUPPORT_RTC_TIMER_CREATE == 0) ? 0 : FUNC_ID_RTC_TIMER_CREATE) | \
                        CAP_FID(OFFSET, (SUPPORT_RTC_TIMER_READ == 0) ? 0 : FUNC_ID_RTC_TIMER_READ) | \
                        CAP_FID(OFFSET, (SUPPORT_RTC_TIMER_DELETE == 0) ? 0 : FUNC_ID_RTC_TIMER_DELETE) | \
                        CAP_FID(OFFSET, (SUPPORT_RTC_TIMER_CALL == 0) ? 0 : FUNC_ID_RTC_TIMER_CALL) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_FIRMWARE_UPDATE_NVM == 0) ? 0 : FUNC_ID_ZW_FIRMWARE_UPDATE_NVM) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_LEARN_NODE_STATE == 0) ? 0 : FUNC_ID_ZW_SET_LEARN_NODE_STATE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_NODE_PROTOCOL_INFO == 0) ? 0 : FUNC_ID_ZW_GET_NODE_PROTOCOL_INFO) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_DEFAULT == 0) ? 0 : FUNC_ID_ZW_SET_DEFAULT) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_NEW_CONTROLLER == 0) ? 0 : FUNC_ID_ZW_NEW_CONTROLLER) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REPLICATION_COMMAND_COMPLETE == 0) ? 0 : FUNC_ID_ZW_REPLICATION_COMMAND_COMPLETE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REPLICATION_SEND_DATA == 0) ? 0 : FUNC_ID_ZW_REPLICATION_SEND_DATA) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_ASSIGN_RETURN_ROUTE == 0) ? 0 : FUNC_ID_ZW_ASSIGN_RETURN_ROUTE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_DELETE_RETURN_ROUTE == 0) ? 0 : FUNC_ID_ZW_DELETE_RETURN_ROUTE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REQUEST_NODE_NEIGHBOR_UPDATE == 0) ? 0 : FUNC_ID_ZW_REQUEST_NODE_NEIGHBOR_UPDATE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_APPLICATION_CONTROLLER_UPDATE == 0) ? 0 : FUNC_ID_ZW_APPLICATION_UPDATE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_ADD_NODE_TO_NETWORK == 0) ? 0 : FUNC_ID_ZW_ADD_NODE_TO_NETWORK) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REMOVE_NODE_FROM_NETWORK == 0) ? 0 : FUNC_ID_ZW_REMOVE_NODE_FROM_NETWORK) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_CREATE_NEW_PRIMARY == 0) ? 0 : FUNC_ID_ZW_CREATE_NEW_PRIMARY) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_CONTROLLER_CHANGE == 0) ? 0 : FUNC_ID_ZW_CONTROLLER_CHANGE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REQUEST_NODE_INFO == 0) ? 0 : FUNC_ID_ZW_REQUEST_NODE_INFO) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REMOVE_FAILED_NODE_ID == 0) ? 0 : FUNC_ID_ZW_REMOVE_FAILED_NODE_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_IS_FAILED_NODE_ID == 0) ? 0 : FUNC_ID_ZW_IS_FAILED_NODE_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REPLACE_FAILED_NODE == 0) ? 0 : FUNC_ID_ZW_REPLACE_FAILED_NODE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_ROUTING_MAX == 0) ? 0 : FUNC_ID_ZW_GET_ROUTING_MAX) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_ROUTING_MAX == 0) ? 0 : FUNC_ID_ZW_SET_ROUTING_MAX) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_IS_PRIMARY_CTRL == 0) ? 0 : FUNC_ID_ZW_IS_PRIMARY_CTRL) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_AES_ECB == 0) ? 0 : FUNC_ID_ZW_AES_ECB) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_APPLICATION_UPDATE == 0) ? 0 : FUNC_ID_ZW_APPLICATION_UPDATE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_LEARN_MODE == 0) ? 0 : FUNC_ID_ZW_SET_LEARN_MODE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_EXPLORE_REQUEST_INCLUSION == 0) ? 0 : FUNC_ID_ZW_EXPLORE_REQUEST_INCLUSION) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_ASSIGN_SUC_RETURN_ROUTE == 0) ? 0 : FUNC_ID_ZW_ASSIGN_SUC_RETURN_ROUTE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_ENABLE_SUC == 0) ? 0 : FUNC_ID_ZW_ENABLE_SUC) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_REQUEST_NETWORK_UPDATE == 0) ? 0 : FUNC_ID_ZW_REQUEST_NETWORK_UPDATE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_SUC_NODE_ID == 0) ? 0 : FUNC_ID_ZW_SET_SUC_NODE_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_DELETE_SUC_RETURN_ROUTE == 0) ? 0 : FUNC_ID_ZW_DELETE_SUC_RETURN_ROUTE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_SUC_NODE_ID == 0) ? 0 : FUNC_ID_ZW_GET_SUC_NODE_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_SUC_ID == 0) ? 0 : FUNC_ID_ZW_SEND_SUC_ID) | \
                        CAP_FID(OFFSET, (SUPPORT_TIMER_START == 0) ? 0 : FUNC_ID_TIMER_START) | \
                        CAP_FID(OFFSET, (SUPPORT_TIMER_RESTART == 0) ? 0 : FUNC_ID_TIMER_RESTART) | \
                        CAP_FID(OFFSET, (SUPPORT_TIMER_CANCEL == 0) ? 0 : FUNC_ID_TIMER_CANCEL) | \
                        CAP_FID(OFFSET, (SUPPORT_TIMER_CALL == 0) ? 0 : FUNC_ID_TIMER_CALL) | \
                        CAP_FID(OFFSET, (SUPPORT_GET_ROUTING_TABLE_LINE == 0) ? 0 : FUNC_ID_GET_ROUTING_TABLE_LINE) | \
                        CAP_FID(OFFSET, (SUPPORT_GET_TX_COUNTER == 0) ? 0 : FUNC_ID_GET_TX_COUNTER) | \
                        CAP_FID(OFFSET, (SUPPORT_RESET_TX_COUNTER == 0) ? 0 : FUNC_ID_RESET_TX_COUNTER) | \
                        CAP_FID(OFFSET, (SUPPORT_STORE_NODEINFO == 0) ? 0 : FUNC_ID_STORE_NODEINFO) | \
                        CAP_FID(OFFSET, (SUPPORT_STORE_HOMEID == 0) ? 0 : FUNC_ID_STORE_HOMEID) | \
                        CAP_FID(OFFSET, (SUPPORT_LOCK_ROUTE_RESPONSE == 0) ? 0 : FUNC_ID_LOCK_ROUTE_RESPONSE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_LAST_WORKING_ROUTE == 0) ? 0 : FUNC_ID_ZW_GET_LAST_WORKING_ROUTE)|\
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_LAST_WORKING_ROUTE == 0) ? 0 : FUNC_ID_ZW_SET_LAST_WORKING_ROUTE)|\
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_APPL_SLAVE_NODE_INFORMATION == 0) ? 0 : FUNC_ID_SERIAL_API_APPL_SLAVE_NODE_INFORMATION) | \
                        CAP_FID(OFFSET, (SUPPORT_APPLICATION_SLAVE_COMMAND_HANDLER == 0) ? 0 : FUNC_ID_APPLICATION_SLAVE_COMMAND_HANDLER) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_SLAVE_NODE_INFORMATION == 0) ? 0 : FUNC_ID_ZW_SEND_SLAVE_NODE_INFORMATION) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SEND_SLAVE_DATA == 0) ? 0 : FUNC_ID_ZW_SEND_SLAVE_DATA) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_SLAVE_LEARN_MODE == 0) ? 0 : FUNC_ID_ZW_SET_SLAVE_LEARN_MODE) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_GET_VIRTUAL_NODES == 0) ? 0 : FUNC_ID_ZW_GET_VIRTUAL_NODES) | \
                        CAP_FID(OFFSET, (SUPPORT_ZW_IS_VIRTUAL_NODE == 0) ? 0 : FUNC_ID_ZW_IS_VIRTUAL_NODE) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_EXT == 0) ? 0 : FUNC_ID_SERIAL_API_EXT) | \
                        CAP_FID(OFFSET, (SUPPORT_SERIAL_API_GET_APPL_HOST_MEMORY_OFFSET == 0) ? 0 : FUNC_ID_SERIAL_API_GET_APPL_HOST_MEMORY_OFFSET) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_SET_PROMISCUOUS_MODE == 0) ? 0 : FUNC_ID_ZW_SET_PROMISCUOUS_MODE) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_NUNIT == 0) ? 0 : FUNC_ID_ZW_NUNIT_CMD) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_NUNIT == 0) ? 0 : FUNC_ID_ZW_NUNIT_INIT) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_NUNIT == 0) ? 0 : FUNC_ID_ZW_NUNIT_LIST) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_NUNIT == 0) ? 0 : FUNC_ID_ZW_NUNIT_RUN) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_NUNIT == 0) ? 0 : FUNC_ID_ZW_NUNIT_END) |\
                        CAP_FID(OFFSET, (SUPPORT_ZW_PORT_STATUS == 0) ? 0 : FUNC_ID_IO_PORT_STATUS)| \
                        CAP_FID(OFFSET, (SUPPORT_ZW_PORT_STATUS == 0) ? 0 : FUNC_ID_IO_PORT) \
                       )
#endif  /* ZW_CONTROLLER_SINGLE */

#ifdef SUPPORT_SERIAL_API_GET_CAPABILITIES
#ifdef WORK_PATCH
#ifndef __IAR_SYSTEMS_ICC__
extern const XBYTE SERIALAPI_CAPABILITIES[8+32];
#else
extern BYTE SERIALAPI_CAPABILITIES[8+32];
#endif
#else /*WORK_PATCH*/
#ifndef __IAR_SYSTEMS_ICC__
extern const XBYTE SERIALAPI_CAPABILITIES[];
#else
extern BYTE SERIALAPI_CAPABILITIES[];
#endif
#endif /* MAKE_PATCH_CODE */

#endif

extern void ZCB_ComplHandler_ZW_netWork_Management(
  BYTE bStatus                           /* IN   Transmit completion status  */
);
extern void ZCB_ComplHandler_ZW_RemoveFailedNodeID(
  BYTE bStatus                           /* IN   Transmit completion status  */
);
extern void ZCB_ComplHandler_ZW_ReplaceFailedNode(
  BYTE bStatus                           /* IN   Transmit completion status  */
);
extern void DoRespond_workbuf(
  BYTE cnt);



#ifdef UZB

extern void set_state(
  BYTE st
);

#endif	 //#ifdef UZB


extern BOOL Request(
  BYTE cmd,             /*IN   Command                  */
  XBYTE *pData,         /*IN   pointer to data          */
  BYTE len              /*IN   Length of data           */
);
extern void Respond(
  BYTE cmd,             /*IN   Command                  */
  XBYTE *pData,         /*IN   pointer to data          */
  BYTE len              /*IN   Length of data           */
);
extern void DoRespond( void );

extern void PopCallBackQueue(void);

extern BYTE GetCallbackCnt(void);

extern void SaveApplicationSettings(void);

#ifdef ZW_CONTROLLER
extern void SetupNodeManagement(void);
#endif /* ZW_CONTROLLER */

#if SUPPORT_ZW_ASSIGN_RETURN_ROUTE
extern void ZCB_ComplHandler_ZW_AssignReturnRoute(
  BYTE bStatus                          /* IN   Transmit completion status  */
);
#endif /* SUPPORT_ZW_ASSIGN_RETURN_ROUTE */

#if SUPPORT_ZW_DELETE_RETURN_ROUTE
extern void ZCB_ComplHandler_ZW_DeleteReturnRoute(
  BYTE bStatus                          /* IN   Transmit completion status  */
);
#endif /* SUPPORT_ZW_DELETE_RETURN_ROUTE */

#if SUPPORT_ZW_SEND_SUC_ID
extern void ZCB_ComplHandler_ZW_SendSUC_ID(
BYTE bStatus);
#endif /* SUPPORT_ZW_SEND_SUC_ID */

#if SUPPORT_ZW_REDISCOVERY_NEEDED
void ZCB_ComplHandler_ZW_RediscoveryNeeded(BYTE status);
#endif /* SUPPORT_ZW_REDISCOVERY_NEEDED */

#ifdef ZW_CONTROLLER_SINGLE
#if SUPPORT_SERIAL_API_TEST

extern void
SendTestReport(
  BYTE txStatus);

extern void
TestStartRound(void);

extern void
SendTestRoundReport(
  BYTE txStatus);

extern void
ZCB_TestDelayNextSendTimeout(void);

extern void
ZCB_TestDelayTimeout(void);

extern BOOL
TestFindNextNode(void);

extern void
ZCB_TestSendComplete(
  BYTE bStatus);

extern void
TestSend(void);

#endif /* SUPPORT_SERIAL_API_TEST */
#endif /* ZW_CONTROLLER_SINGLE */

#if SUPPORT_SERIAL_API_POWER_MANAGEMENT
extern void
ZCB_PowerManagementWakeUpOnExternalActive(void);

extern void
ZCB_PowerManagementWakeUpOnTimerHandler(void);

extern void
ZCB_powerManagementPoweredUpPinActive(void);

extern void
PowerManagementSetPowerDown(void);

extern void
PowerManagementSetPowerUp(void);

extern void
PowerManagementCheck(void);

extern void
PurgeCallbackQueue(void);
#endif /* SUPPORT_SERIAL_API_POWER_MANAGEMENT */

extern void ZCB_ComplHandler_ZW_SendNodeInformation(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_SendData(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_SendDataMeta(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_SetSUCNodeID(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_SendDataMulti(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_RequestNodeInfo(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_RequestNodeNeighborUpdate(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_SetDefault(void);

extern void ZCB_ComplHandler_ZW_NodeManagement(struct _LEARN_INFO_ *learnNodeInfo);

extern void ZCB_ComplHandler_ZW_ReplicationSendData(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_SendTestFrame(BYTE txStatus);

extern void ZCB_ComplHandler_MemoryPutBuffer(void);

extern void ZCB_ComplHandler_ZW_SendData_Bridge(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_SendDataMeta_Bridge(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_SendDataMulti_Bridge(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_SendSlaveNodeInformation(BYTE txStatus);

extern void ZCB_ComplHandler_ZW_SetSlaveLearnMode(BYTE bStatus, BYTE orgID, BYTE newID);

extern void ZCB_ComplHandler_ZW_SetLearnMode(BYTE bStatus, BYTE bNodeID);

extern void ZCB_ComplHandler_ZW_StoreNodeInfo(void);


#endif /*_SERIALAPPL_H_*/
