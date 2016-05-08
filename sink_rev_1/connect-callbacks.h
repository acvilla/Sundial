// This file is generated by Ember Desktop.  Please do not edit manually.
//
//

// Enclosing macro to prevent multiple inclusion
#ifndef __CONNECT_CALLBACKS__
#define __CONNECT_CALLBACKS__


#include PLATFORM_HEADER
#include CONFIGURATION_HEADER
#include "stack/include/ember.h"
#include "command-interpreter/command-interpreter.h"
#include "heartbeat/heartbeat.h"
#include "hal/hal.h"
#include "serial/serial.h"


/** @brief Main Application Entry Point
 *
 * This is the main application entry point. All applications
 * must implement this function.
 */
int main(MAIN_FUNCTION_PARAMETERS);

// The Simulated EEPROM callback function, implemented by the
// application.
void halSimEepromCallback(EmberStatus status);

// Handler called whenever the radio is powered on.
void halRadioPowerUpHandler(void);

// Handler called whenever the radio is powered off.
void halRadioPowerDownHandler(void);

/** @brief Main Init
 *
 * This function is called when the application starts and can be used to
 * perform any additional initialization required at system startup.
 */
void emberAfMainInitCallback(void);

/** @brief Main Tick
 *
 * This function is called in each iteration of the main application loop and
 * can be used to perform periodic functions.  The frequency with which this
 * function is called depends on how quickly the main loop runs.  If the
 * application blocks at any time during the main loop, this function will not
 * be called until execution resumes. Sleeping and idling will block.
 */
void emberAfMainTickCallback(void);

/** @brief Stack Status
 *
 * This function is called when the stack status changes.  This callbacks
 * provides applications an opportunity to be notified of changes to the stack
 * status and take appropriate action.
 *
 * @param status   Ver.: always
 */
void emberAfStackStatusCallback(EmberStatus status);

/** @brief Incoming Message
 *
 * This function is called when a message is received.
 *
 * @param message   Ver.: always
 */
void emberAfIncomingMessageCallback(EmberIncomingMessage *message);

/** @brief Message Sent
 *
 * This function is called to indicate whether an outgoing message was 
 * successfully transmitted or to indicate the reason of failure.
 *
 * @param status    Ver.: always
 * @param message   Ver.: always
 */
void emberAfMessageSentCallback(EmberStatus status, 
                                EmberOutgoingMessage *message);

/** @brief Child Join
 *
 * This function is called when a node has joined the network.
 *
 * @param nodeType   Ver.: always
 * @param nodeId     Ver.: always
 */
void emberAfChildJoinCallback(EmberNodeType nodeType,
                              EmberNodeId nodeId);

/** @brief Active Scan Complete
 *
 * This function is called when a node has completed an active scan.
 */
void emberAfActiveScanCompleteCallback(void);

/** @brief Child Join
 *
 * This function is called when a node has joined the network.
 *
 * @param mean       Ver.: always
 * @param min        Ver.: always
 * @param max        Ver.: always
 * @param variance   Ver.: always
 */
void emberAfEnergyScanCompleteCallback(int8_t mean,
                                       int8_t min,
                                       int8_t max,
                                       uint16_t variance);

/** @brief Incoming Beacon
 *
 * This function is called when a node is performing an active scan and a beacon
 * is received.
 *
 * @param panId          Ver.: always
 * @param nodeId         Ver.: always
 * @param payloadLength  Ver.: always
 * @param payload        Ver.: always
 */
void emberAfIncomingBeaconCallback(EmberPanId panId,
                                   EmberNodeId nodeId,
                                   uint8_t payloadLength,
                                   uint8_t *payload);

/** @brief Mark Application Buffers
 *
 * This function is called when the application must mark its buffers.  Buffers
 * that are not marked will be reclaimed by the stack.
 */
void emberAfMarkApplicationBuffersCallback(void);

void emberIncomingMessageHandler(EmberIncomingMessage *message);

void emberMessageSentHandler(EmberStatus status, EmberOutgoingMessage *message);

void emberStackStatusHandler(EmberStatus status);

void emberStackIsrHandler(void);

void emberMarkApplicationBuffersHandler(void);

void emberChildJoinHandler(EmberNodeType nodeType,
                           EmberNodeId nodeId);

void emberIncomingBeaconHandler(EmberPanId panId,
                                EmberNodeId nodeId,
                                uint8_t payloadLength,
                                uint8_t *payload);

void emberActiveScanCompleteHandler(void);

void emberEnergyScanCompleteHandler(int8_t mean,
                                    int8_t min,
                                    int8_t max,
                                    uint16_t variance);

#endif // __CONNECT_CALLBACKS__
