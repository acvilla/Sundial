// This file is generated by Ember Desktop.  Please do not edit manually.
//
//

// Enclosing macro to prevent multiple inclusion
#ifndef __CONNECT_CONFIG__
#define __CONNECT_CONFIG__




// Top level macros
#define EMBER_AF_DEVICE_NAME "sink_ezr32"


// Generated setup headers that are included automatically
#include "connect-debug-print.h"


// Generated plugin macros

// Use this macro to check if Command Interpreter plugin is included
#define EMBER_AF_PLUGIN_COMMAND_INTERPRETER

// Use this macro to check if Debug Print plugin is included
#define EMBER_AF_PLUGIN_DEBUG_PRINT
// User options for plugin Debug Print
#define EMBER_AF_DEBUG_PRINT_USE_PORT

// Use this macro to check if Diagnostic plugin is included
#define EMBER_AF_PLUGIN_DIAGNOSTIC

// Use this macro to check if HAL library plugin is included
#define EMBER_AF_PLUGIN_HAL_EZR32

// Use this macro to check if Heartbeat plugin is included
#define EMBER_AF_PLUGIN_HEARTBEAT
// User options for plugin Heartbeat
#define EMBER_AF_PLUGIN_HEARTBEAT_LED 1
#define EMBER_AF_PLUGIN_HEARTBEAT_BLINK_QS 1

// Use this macro to check if Main plugin is included
#define EMBER_AF_PLUGIN_MAIN

// Use this macro to check if EzRadio PHY plugin is included
#define EMBER_AF_PLUGIN_PHY_EZRADIO
// User options for plugin EzRadio PHY
#define EMBER_RADIO_CCA_THRESHOLD -65

// Use this macro to check if Serial plugin is included
#define EMBER_AF_PLUGIN_SERIAL

// Use this macro to check if Simulated EEPROM version 1 Library plugin is included
#define EMBER_AF_PLUGIN_SIM_EEPROM1

// Use this macro to check if Security AES plugin is included
#define EMBER_AF_PLUGIN_STACK_AES_SECURITY

// Use this macro to check if Stack Common plugin is included
#define EMBER_AF_PLUGIN_STACK_COMMON
// User options for plugin Stack Common
#define EMBER_HEAP_SIZE 4000

// Use this macro to check if Stack packet counters plugin is included
#define EMBER_AF_PLUGIN_STACK_COUNTERS

// Use this macro to check if Form and Join plugin is included
#define EMBER_AF_PLUGIN_STACK_FORM_AND_JOIN

// Use this macro to check if MAC Packet Queue plugin is included
#define EMBER_AF_PLUGIN_STACK_MAC_QUEUE
// User options for plugin MAC Packet Queue
#define EMBER_MAC_OUTGOING_QUEUE_SIZE 8

// Use this macro to check if Parent Support plugin is included
#define EMBER_AF_PLUGIN_STACK_PARENT_SUPPORT
// User options for plugin Parent Support
#define EMBER_CHILD_TABLE_SIZE 16
#define EMBER_INDIRECT_QUEUE_SIZE 8
#define EMBER_INDIRECT_TRANSMISSION_TIMEOUT_MS 8000

// Use this macro to check if Security XXTEA Stub plugin is included
#define EMBER_AF_PLUGIN_STACK_XXTEA_SECURITY_STUB


// Generated API headers

// API command-interpreter2 from Command Interpreter plugin
#define EMBER_AF_API_COMMAND_INTERPRETER2 "../../../SiliconLabs/SiliconLabsConnect/connect/plugins/command-interpreter/command-interpreter.h"

// API debug-print from Debug Print plugin
#define EMBER_AF_API_DEBUG_PRINT "../../../SiliconLabs/SiliconLabsConnect/connect/plugins/debug-print/debug-print.h"

// API diagnostic from Diagnostic plugin
#define EMBER_AF_API_DIAGNOSTIC "../../../SiliconLabs/SiliconLabsConnect/submodules/base/hal/micro/diagnostic.h"

// API diagnostic-cortexm3 from Diagnostic plugin
#define EMBER_AF_API_DIAGNOSTIC_CORTEXM3 "../../../SiliconLabs/SiliconLabsConnect/submodules/base/hal/micro/cortexm3/diagnostic.h"

// API serial from Serial plugin
#define EMBER_AF_API_SERIAL "../../../SiliconLabs/SiliconLabsConnect/connect/plugins/serial/serial.h"

// API sim-eeprom from Simulated EEPROM version 1 Library plugin
#define EMBER_AF_API_SIM_EEPROM "../../../SiliconLabs/SiliconLabsConnect/submodules/base/hal/micro/sim-eeprom.h"


// Custom macros
#ifdef EMBER_AF_RADIO
#undef EMBER_AF_RADIO
#endif
#define EMBER_AF_RADIO SI4460

#ifdef EMBER_AF_RADIO_FULL
#undef EMBER_AF_RADIO_FULL
#endif
#define EMBER_AF_RADIO_FULL

#ifdef EMBER_AF_RADIO_REVISION
#undef EMBER_AF_RADIO_REVISION
#endif
#define EMBER_AF_RADIO_REVISION REVC2A

#ifdef EMBER_AF_MCU
#undef EMBER_AF_MCU
#endif
#define EMBER_AF_MCU EZR32

#ifdef EMBER_AF_MCU_FULL
#undef EMBER_AF_MCU_FULL
#endif
#define EMBER_AF_MCU_FULL

#ifdef EMBER_AF_MCU_MCU
#undef EMBER_AF_MCU_MCU
#endif
#define EMBER_AF_MCU_MCU WG330

#ifdef EMBER_AF_MCU_FLASH
#undef EMBER_AF_MCU_FLASH
#endif
#define EMBER_AF_MCU_FLASH 256K

#ifdef EMBER_AF_MCU_RADIO_CHIP
#undef EMBER_AF_MCU_RADIO_CHIP
#endif
#define EMBER_AF_MCU_RADIO_CHIP SI4460REVC2A

#ifdef EMBER_AF_BOARD_TYPE
#undef EMBER_AF_BOARD_TYPE
#endif
#define EMBER_AF_BOARD_TYPE BRD4502C



#endif // __CONNECT_CONFIG__