// Sink sample application.
//
// Copyright 2013 Silicon Laboratories, Inc.                                *80*



#include PLATFORM_HEADER
#include CONFIGURATION_HEADER

#include "em_device.h"
#include "em_chip.h"
#include "bsp.h"

#include "stack/include/ember.h"
#include "heartbeat/heartbeat.h"
#include "hal/hal.h"
#include "phy/phy.h"
#include "command-interpreter/command-interpreter.h"
#include "debug-print/debug-print.h"

#include "em_gpio.h"
#include "em_cmu.h"

#if (defined EMBER_AF_PLUGIN_FREE_RTOS)
#include "FreeRTOS.h"
#include "task.h"
#include EMBER_AF_API_FREE_RTOS
extern long stack_blocked_deep_sleep;
extern long stack_allowed_deep_sleep;
#endif

#define SENSOR_SINK_TX_POWER    0
#define SENSOR_SINK_PAN_ID      0x01FF
#define SENSOR_SINK_PROTOCOL_ID 0xC00F

#define SENSOR_SINK_PROTOCOL_ID_OFFSET  0
#define SENSOR_SINK_COMMAND_ID_OFFSET   2
#define SENSOR_SINK_EUI64_OFFSET        3
#define SENSOR_SINK_NODE_ID_OFFSET      11
#define SENSOR_SINK_DATA_OFFSET         13
#define SENSOR_SINK_MAXIMUM_DATA_LENGTH 10
#define SENSOR_SINK_MINIMUM_LENGTH      SENSOR_SINK_DATA_OFFSET
#define SENSOR_SINK_MAXIMUM_LENGTH      (SENSOR_SINK_MINIMUM_LENGTH \
                                         + SENSOR_SINK_MAXIMUM_DATA_LENGTH)

#define SINK_ADVERTISEMENT_PERIOD_MS (1 * MILLISECOND_TICKS_PER_SECOND)
#define SENSOR_REPORT_PERIOD_MS      (1 * MILLISECOND_TICKS_PER_SECOND)
#define SENSOR_TIMEOUT_MS            (60 * MILLISECOND_TICKS_PER_SECOND)
#define SINK_DATA_DUMP_PERIOD_MS     (1 * MILLISECOND_TICKS_PER_SECOND)

#define NETWORK_UP_LED               BOARDLED0

#define SENSOR_SINK_SECURITY_KEY    {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,       \
                                     0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,       \
                                     0xAA, 0xAA, 0xAA, 0xAA}

#define SENSOR_TABLE_SIZE 5


enum {
  SENSOR_SINK_COMMAND_ID_ADVERTISE_REQUEST = 0,
  SENSOR_SINK_COMMAND_ID_ADVERTISE         = 1,
  SENSOR_SINK_COMMAND_ID_PAIR_REQUEST      = 2,
  SENSOR_SINK_COMMAND_ID_PAIR_CONFIRM      = 3,
  SENSOR_SINK_COMMAND_ID_DATA              = 4,
};
typedef uint8_t SensorSinkCommandId;

typedef struct{
		uint16_t Light_1_Power[96];
		uint16_t Light_2_Power[96];
		uint16_t USB_1_Power[96];
		uint16_t USB_2_Power[96];
		uint16_t Solar_Power[96];
		uint32_t EUnodeId; //unique address of node
		uint32_t Sample_Time[96]; //Gives the time that the sample was taken. the index corresponds to the sample number
		uint8_t SampleNumber; //Index of samples, there are 96 samples per day
}Node_Power;
	uint16_t nodes_array_size = 0;
	static Node_Power nodes_array[SENSOR_TABLE_SIZE];//nodes_array will be the final buffer used to write all of the aggregated data to the sd card, either when power is low
	//or when enough samples have been generated
typedef struct {
  EmberNodeId nodeId;
  EmberEUI64  nodeEui64;
  uint8_t       reportedData[SENSOR_SINK_MAXIMUM_DATA_LENGTH];
  uint8_t       reportedDataLength;
  uint32_t      lastReportMs;
} Sensor;
static EmberKeyData securityKey = SENSOR_SINK_SECURITY_KEY;

static void sinkInit(void);
static EmberStatus send(EmberNodeId nodeId,
                        SensorSinkCommandId commandId,
                        uint8_t *buffer,
                        uint8_t bufferLength);
static uint16_t fetchLowHighInt16u(uint8_t *contents);
static void storeLowHighInt16u(uint8_t *contents, uint16_t value);
uint16_t nodePush(Node_Power* array, uint16_t size);
void updateNodesArray(uint16_t *measurements, uint32_t EUnodeId, EmberIncomingMessage *message);
EmberEventControl advertiseControl;
EmberEventControl dataReportControl;

static uint8_t message[SENSOR_SINK_MAXIMUM_LENGTH];
static EmberMessageLength messageLength;
static EmberMessageOptions txOptions = EMBER_OPTIONS_ACK_REQUESTED;
static Sensor sensors[SENSOR_TABLE_SIZE];


// An advertisement message consists of the sensor/sink protocol id, the
// advertisement command id, and the long and short ids of the sink.  Each sink
// on the network periodically broadcasts its advertisement to all other nodes.

void updateNodesArray(uint16_t *measurements, uint32_t EUnodeId, EmberIncomingMessage *message){
	    	uint8_t nodeInTable = 0; //1 if node is found in the table
	    	int n = 0;
	        for(n = 0; n < SENSOR_TABLE_SIZE; n++){
	        	if(nodes_array[n].EUnodeId == EUnodeId)//if the 64-bit IEEE address of tempNode matches the node at index n then add in the sampled data
	        	{
	        		nodeInTable = 1;
	        		nodes_array[n].Light_1_Power[nodes_array[n].SampleNumber] = measurements[0];
	        		nodes_array[n].Light_2_Power[nodes_array[n].SampleNumber] = measurements[1];
	        		nodes_array[n].USB_1_Power[nodes_array[n].SampleNumber] = measurements[2];
	        		nodes_array[n].USB_2_Power[nodes_array[n].SampleNumber] = measurements[3];
	        		nodes_array[n].Solar_Power[nodes_array[n].SampleNumber] = measurements[4];
	        		nodes_array[n].Sample_Time[nodes_array[n].SampleNumber] =  measurements[5];//get current time
	        		nodes_array[n].SampleNumber= (nodes_array[n].SampleNumber + 1) % 96; //increment the sample index

	        	}
	        }
	        if (nodeInTable == 0)                            //if sample isn't in table append a node struct to the end of the nodes_array
	        {

	        	uint16_t new_size = nodePush(nodes_array, nodes_array_size);
	        	nodes_array_size = new_size;


	        	nodes_array[nodes_array_size - 1].SampleNumber = 0;
	        	nodes_array[nodes_array_size - 1].Light_1_Power[0] = measurements[0];
	    		nodes_array[nodes_array_size - 1].Light_2_Power[0] = measurements[1];
	    		nodes_array[nodes_array_size - 1].USB_1_Power[0] = measurements[2];
	    		nodes_array[nodes_array_size - 1].USB_2_Power[0] = measurements[3];
	    		nodes_array[nodes_array_size - 1].Solar_Power[0] = measurements[4];
	    		nodes_array[nodes_array_size - 1].Sample_Time[0] = measurements[5];//get current time
	    		nodes_array[nodes_array_size - 1].SampleNumber = 1; //increment the sample index
	    		nodes_array[nodes_array_size - 1].EUnodeId = EUnodeId;          //set the EUnodeId of the new Node_Power struct

	        }
}

uint16_t nodePush(Node_Power* array, uint16_t size)
{
	//if node not already present in array, push it to the end
    uint16_t new_size = size + 1;
    if (new_size <= SENSOR_TABLE_SIZE)
    	{
      	//Node_Power* new_addr = (array + 1);//not needed but can use instead of returning size
    	return new_size;
    	}
    else
    	{
    	return NULL;
    	}
}

void advertiseHandler(void)
{
  // If the sink is not on the network, the periodic event is cancelled and
  // advertisements are not set.
  if (!emberStackIsUp()) {
    emberEventControlSetInactive(advertiseControl);
  } else {
    EmberStatus status = send(EMBER_BROADCAST_ADDRESS,
                              SENSOR_SINK_COMMAND_ID_ADVERTISE,
                              NULL,
                              0);
    //emberAfCorePrintln("TX: Advertise to 0x%2x: 0x%x",
                          //EMBER_BROADCAST_ADDRESS,
                         // status);
    emberEventControlSetDelayMS(advertiseControl,
                                SINK_ADVERTISEMENT_PERIOD_MS);
  }
}


// Here we print out the first two bytes reported by the sinks as a little
// endian 16-bits decimal.
/*
typedef struct {
  EmberNodeId nodeId;
  EmberEUI64  nodeEui64;
  uint8_t       reportedData[SENSOR_SINK_MAXIMUM_DATA_LENGTH];
  uint8_t       reportedDataLength;
  uint32_t      lastReportMs;
} Sensor;
*/
void dataReportHandler(void)
{
//emberAfCorePrintln("EVENT: data received");
	//stack is not up
  // If the sink is not on the network, the periodic event is cancelled and
  // sensors data is no longer printed out.
  if (!emberStackIsUp()) {
    //emberEventControlSetInactive(dataReportControl);
  } else {
    uint8_t i;

    for (i = 0; i < SENSOR_TABLE_SIZE; i++) {

      if ((sensors[i].nodeId != EMBER_NULL_NODE_ID)
          && (sensors[i].reportedDataLength >= 2)) {

    	/*
    	uint16_t measurements[5];
    	int LowByte = 0; //LowByte and HighByte are reset every call to dataReportHandler
    	int HighByte = 1;
    	for(int j=0; j<5; j++){
    		measurements[j] = (sensors[i].reportedData[LowByte]|(sensors[i].reportedData[HighByte] << 8));
    		LowByte += 2;  // 0 2 4 6 8 - Increment the low byte and high byte to parse in the next power measurement
    		HighByte += 2; //  1 3 5 7 9
    	}

    	unsigned long long EUnodeId = (sensors[i].nodeEui64[3] << 24)| (sensors[i].nodeEui64[2] << 16)|
    	                              (sensors[i].nodeEui64[1] << 8)|  (sensors[i].nodeEui64[0]);  //set the nodeEUi64 address of temp node

    	updateNodesArray(measurements, EUnodeId);
    	*/



      }
    }
    emberEventControlSetDelayMS(dataReportControl,
                                SINK_DATA_DUMP_PERIOD_MS);
  }
}

void emberAfIncomingMessageCallback(EmberIncomingMessage *message)
{
  if (message->length < SENSOR_SINK_MINIMUM_LENGTH
      || (fetchLowHighInt16u(message->payload + SENSOR_SINK_PROTOCOL_ID_OFFSET)
          != SENSOR_SINK_PROTOCOL_ID)) {
    return;
  }
  //emberAfCorePrintln("payload is: %x", message->payload[SENSOR_SINK_COMMAND_ID_OFFSET]);
  switch (message->payload[SENSOR_SINK_COMMAND_ID_OFFSET]) {
  case SENSOR_SINK_COMMAND_ID_ADVERTISE_REQUEST:
  {
    EmberStatus status;
    //emberAfCorePrintln("RX: Advertise Request from 0x%2x", message->source);

    // We received an advertise request from a sensor, unicast back an advertise
    // command.
    status = send(message->source,
                  SENSOR_SINK_COMMAND_ID_ADVERTISE,
                  NULL,
                  0);
    //emberAfCorePrintln("SENSOR_SINK_COMMAND_ID_ADVERTISE_REQUEST");
  }
    break;
  case SENSOR_SINK_COMMAND_ID_ADVERTISE:
    //emberAfCorePrintln("RX: Advertise from 0x%2x", message->source);
    //emberAfCorePrintln("SENSOR_SINK_COMMAND_ID_ADVERTISE");
    break;
  case SENSOR_SINK_COMMAND_ID_PAIR_REQUEST:
    {
      uint8_t i;
      emberAfCorePrintln("RX: Pair Request from 0x%2x", message->source);
      // Check whether the sensor is already present in the table first.
      for (i = 0; i < SENSOR_TABLE_SIZE; i++) {
        if (sensors[i].nodeId != EMBER_NULL_NODE_ID
            && MEMCOMPARE(sensors[i].nodeEui64,
                          message->payload + SENSOR_SINK_EUI64_OFFSET,
                          EUI64_SIZE) == 0) {
        	emberAfCorePrintln("found in sensor table");
          break;
        }
      }

      // Find an empty entry in the table. The node was not there before
      if (i == SENSOR_TABLE_SIZE) {
        for (i = 0; i < SENSOR_TABLE_SIZE; i++) {
          if (sensors[i].nodeId == EMBER_NULL_NODE_ID) {
        	  //sensors[i].nodeId = message->source;		//added something so it would do something with the empty entry
        	  emberAfCorePrintln("something else");
        	//emberAfCorePrintln("current value of i: %d", i);
            break;
          }
        }
      }

      // Add or update the entry in the table. I don't think this is working as planned
      if (i < SENSOR_TABLE_SIZE) {
        EmberStatus status = send(message->source,
                                  SENSOR_SINK_COMMAND_ID_PAIR_CONFIRM,
                                  NULL,
                                  0);
        if (status == EMBER_SUCCESS) {
          sensors[i].nodeId = message->source;

          MEMCOPY(sensors[i].nodeEui64,
                  message->payload + SENSOR_SINK_EUI64_OFFSET,
                  EUI64_SIZE);
          emberAfCorePrintln("placing in sensor table");
          sensors[i].lastReportMs = halCommonGetInt32uMillisecondTick();
        }
      }
    }
    emberAfCorePrintln("SENSOR_SINK_COMMAND_ID_PAIR_REQUEST");
    break;
  case SENSOR_SINK_COMMAND_ID_PAIR_CONFIRM:
    emberAfCorePrintln("RX: Pair Confirm from 0x%2x", message->source);
    //emberAfCorePrintln("SENSOR_SINK_COMMAND_ID_PAIR_CONFIRM");
    break;

  case SENSOR_SINK_COMMAND_ID_DATA:
    {
      uint8_t i, j;
      for (i = 0; i < SENSOR_TABLE_SIZE; i++) {

    	 /*
    	  uint32_t EUnodeId = (sensors[i].nodeEui64[3] << 24)| (sensors[i].nodeEui64[2] << 16)|
          	                  (sensors[i].nodeEui64[1] << 8)|  (sensors[i].nodeEui64[0]);  //set the nodeEUi64 address of temp node
          if(nodes_array[i].EUnodeId == EUnodeId){
    	  */

    	  //when the sink resets, this if statement prevents printing.
        if (MEMCOMPARE(sensors[i].nodeEui64,
                       message->payload + SENSOR_SINK_EUI64_OFFSET,
                       EUI64_SIZE) == 0) {

          sensors[i].reportedDataLength = message->length - SENSOR_SINK_DATA_OFFSET;

          MEMCOPY(sensors[i].reportedData,
                  message->payload + SENSOR_SINK_DATA_OFFSET,
                  sensors[i].reportedDataLength);

          sensors[i].lastReportMs = halCommonGetInt32uMillisecondTick();

          //-------------------------------------------------------MODIFIED-CODE--------------------------------------------------------------------
          uint16_t measurements[6];
          	int LowByte = 0; //LowByte and HighByte are reset every call to dataReportHandler
          	int HighByte = 1;
          	for(int j=0; j<5; j++){
          		measurements[j] = (sensors[i].reportedData[LowByte]|(sensors[i].reportedData[HighByte] << 8));
          		LowByte += 2;  // 0 2 4 6 8 - Increment the low byte and high byte to parse in the next power measurement
          		HighByte += 2; //  1 3 5 7 9
          	}
          	measurements[5] = halCommonGetInt32uMillisecondTick();

          	uint32_t EUnodeId = (sensors[i].nodeEui64[3] << 24)| (sensors[i].nodeEui64[2] << 16)|
          	                              (sensors[i].nodeEui64[1] << 8)|  (sensors[i].nodeEui64[0]);  //set the nodeEUi64 address of temp node

          	updateNodesArray(measurements, EUnodeId,message);

          	emberAfCorePrintln("------------------------------NODE %d---------------------------------------", i);
          	emberAfCorePrintln("EuNodeId: %d", nodes_array[i].EUnodeId);
          	emberAfCorePrintln("");
          	emberAfCorePrintln("TIME:\n %d - YY\n %d - MM\n %d - DD\n %d - MSEC", 0, 0, nodes_array[i].Sample_Time[nodes_array[i].SampleNumber] << 16, nodes_array[i].Sample_Time[nodes_array[i].SampleNumber]);



          	        //here it should check whether nodes array is close to full, or if power is low and should write nodes_array to the SD card
          	        //the function that writes to the sd card using fatfs should add strings and formatting that specify what each value means

          	  //Print Statements
          	  emberAfCorePrintln("Node %d Light_1_Power[%d]: %d mW", i, nodes_array[i].SampleNumber, nodes_array[i].Light_1_Power[nodes_array[i].SampleNumber]);
              emberAfCorePrintln("Node %d Light_2_Power[%d]: %d mW", i, nodes_array[i].SampleNumber, nodes_array[i].Light_2_Power[nodes_array[i].SampleNumber]);
      	      emberAfCorePrintln("Node %d USB_1_Power[%d]: %d mW", i, nodes_array[i].SampleNumber, nodes_array[i].USB_1_Power[nodes_array[i].SampleNumber]);
         	  emberAfCorePrintln("Node %d USB_2_Power[%d]: %d mW", i, nodes_array[i].SampleNumber, nodes_array[i].USB_2_Power[nodes_array[i].SampleNumber]);
          	  emberAfCorePrintln("Node %d Solar_Power[%d]: %d mW", i, nodes_array[i].SampleNumber, nodes_array[i].Solar_Power[nodes_array[i].SampleNumber]);
          break;
        }
      }
    }

    break;
  default:
    //emberAfCorePrintln("RX: Unknown from 0x%2x", message->source);
    break;
  }
}

void emberAfMessageSentCallback(EmberStatus status,
                                EmberOutgoingMessage *message)
{
  if (status != EMBER_SUCCESS) {
    emberAfCorePrintln("TX: 0x%x", status);
  }
}

void emberAfStackStatusCallback(EmberStatus status)
{
  switch (status) {
  case EMBER_NETWORK_UP:
    emberAfCorePrintln("Network up");
    emberEventControlSetActive(advertiseControl);
    emberEventControlSetActive(dataReportControl);
    break;
  case EMBER_NETWORK_DOWN:
	//emberResetNetworkState();
    emberAfCorePrintln("Network down");
    sinkInit();
    break;
  case EMBER_JOIN_FAILED:
    emberAfCorePrintln("Join failed");
    break;
  default:
    emberAfCorePrintln("Stack status: 0x%x", status);
    break;
  }
}

// This callback is called when the application starts and can be used to
// perform any additional initialization required at system startup.
void emberAfMainInitCallback(void)
{
  emberAfCorePrintln("INIT: %p", EMBER_AF_DEVICE_NAME);
  emberAfCorePrintln("\n%p>", EMBER_AF_DEVICE_NAME);

  // Initialize the list of sensors from which we receive reports.
  sinkInit();
  EmberNetworkParameters parameters;
  EmberStatus form_status;

  static EmberKeyData securityKey = SENSOR_SINK_SECURITY_KEY;
  emberSetSecurityKey(&securityKey);
  MEMSET(&parameters, 0, sizeof(EmberNetworkParameters));
  parameters.radioTxPower = 0;//                                (1) --------------------------SET PARAMETERS----------------------------------//
  parameters.radioChannel = 1;
  parameters.panId = SENSOR_SINK_PAN_ID;

  EmberStatus status = emberNetworkInit();//                    (2) Try to rejoin pre-existing network if possible
  emberEventControlSetActive(dataReportControl);


         if (status == EMBER_NOT_JOINED){
		   form_status = emberFormNetwork(&parameters);//       (3) If emberNetworkInit fails then form a network on channel 1 using above parameters
		   emberAfCorePrintln("form 0x%x", form_status);
	 	 }
         //	 	 	 	 	 	 	 	 	 	 	 	 	 	(4) Else the network has already been re-initialized and nothing else needs to be done in this function
}
// This callback is called in each iteration of the main application loop and
// can be used to perform periodic functions.
void emberAfMainTickCallback(void)
{
  // Time out sensors that have not reported in a while.
  uint32_t nowMs = halCommonGetInt32uMillisecondTick();
  uint8_t i;
  for (i = 0; i < SENSOR_TABLE_SIZE; i++) {
    if (sensors[i].nodeId != EMBER_NULL_NODE_ID) {
      if (SENSOR_TIMEOUT_MS
          < elapsedTimeInt32u(sensors[i].lastReportMs, nowMs)) {
        emberAfCorePrintln("EVENT: timed out sensor 0x%2x",
                              sensors[i].nodeId);
        sensors[i].nodeId = EMBER_NULL_NODE_ID;
      }
    }
  }

  if (emberStackIsUp())		//i dont think this is needed
    halSetLed(NETWORK_UP_LED);
  else
    halClearLed(NETWORK_UP_LED);
}

static void sinkInit(void)
{
  uint8_t i;
  for (i = 0; i < SENSOR_TABLE_SIZE; i++) {
    sensors[i].nodeId = EMBER_NULL_NODE_ID;
  }
}

static EmberStatus send(EmberNodeId nodeId,
                        SensorSinkCommandId commandId,
                        uint8_t *buffer,
                        uint8_t bufferLength)
{
  messageLength = 0;
  storeLowHighInt16u(message + messageLength, SENSOR_SINK_PROTOCOL_ID);
  messageLength += 2;
  message[messageLength++] = commandId;
  MEMCOPY(message + messageLength, emberGetEui64(), EUI64_SIZE);
  messageLength += EUI64_SIZE;
  storeLowHighInt16u(message + messageLength, emberGetNodeId());
  messageLength += 2;
  if (bufferLength != 0) {
    MEMCOPY(message + messageLength, buffer, bufferLength);
    messageLength += bufferLength;
  }
  return emberMessageSend(nodeId,
                          0, // endpoint
                          0, // messageTag
                          messageLength,
                          message,
                          txOptions);
}

static uint16_t fetchLowHighInt16u(uint8_t *contents)
{
  return HIGH_LOW_TO_INT(contents[1], contents[0]);
}

static void storeLowHighInt16u(uint8_t *contents, uint16_t value)
{
  contents[0] = LOW_BYTE(value);
  contents[1] = HIGH_BYTE(value);
}

//------------------------------------------------------------------------------
// CLI commands

void formCommand(void)
{
  EmberStatus status;
  EmberNetworkParameters parameters;

  // Initialize the security key to the default key prior to forming the
  // network.
  emberSetSecurityKey(&securityKey);

  MEMSET(&parameters, 0, sizeof(EmberNetworkParameters));
  parameters.radioTxPower = SENSOR_SINK_TX_POWER;
  parameters.radioChannel = emberUnsignedCommandArgument(0);
  parameters.panId = SENSOR_SINK_PAN_ID;

  status = emberFormNetwork(&parameters);

  emberAfCorePrintln("form 0x%x", status);
}

void pjoinCommand(void)
{
  uint8_t duration = (uint8_t)emberUnsignedCommandArgument(0);
  emberPermitJoining(duration);
}

void setTxOptionsCommand(void)
{
  txOptions = emberUnsignedCommandArgument(0);
  emberAfCorePrintln("TX options set: MAC acks %s, security %s, priority %s",
                        ((txOptions & EMBER_OPTIONS_ACK_REQUESTED) ? "enabled" : "disabled"),
                        ((txOptions & EMBER_OPTIONS_SECURITY_ENABLED) ? "enabled" : "disabled"),
                        ((txOptions & EMBER_OPTIONS_HIGH_PRIORITY) ? "enabled" : "disabled"));
}

void setSecurityKeyCommand(void)
{
  EmberKeyData key;
  emberCopyKeyArgument(0, &key);

  if (emberSetSecurityKey(&key) == EMBER_SUCCESS) {
    uint8_t i;

    emberAfCorePrint("Security key set {");
    for(i=0; i<EMBER_ENCRYPTION_KEY_SIZE; i++) {
      emberAfCorePrint("%x", key.contents[i]);
    }
    emberAfCorePrintln("}");
  } else {
    emberAfCorePrintln("Security key set failed");
  }
}

void advertiseCommand(void)
{
  emberEventControlSetActive(advertiseControl);
}

void sensorsCommand(void)
{
  uint8_t i;
  emberAfCorePrintln("### Sensors table ###");
  for (i = 0; i < SENSOR_TABLE_SIZE; i++) {
    if (sensors[i].nodeId != EMBER_NULL_NODE_ID) {
      emberAfCorePrintln("entry:%d id:0x%2x eui64:(>)%x%x%x%x%x%x%x%x last report:0x%4x",
                            (uint16_t)i,
                            sensors[i].nodeId,
                            sensors[i].nodeEui64[7], sensors[i].nodeEui64[6],
                            sensors[i].nodeEui64[5], sensors[i].nodeEui64[4],
                            sensors[i].nodeEui64[3], sensors[i].nodeEui64[2],
                            sensors[i].nodeEui64[1], sensors[i].nodeEui64[0],
                            sensors[i].lastReportMs);
    }
  }
}

void infoCommand(void)
{
  emberAfCorePrintln("%p:", EMBER_AF_DEVICE_NAME);
  emberAfCorePrintln("  network state: 0x%x", emberNetworkState());
  emberAfCorePrintln("      node type: 0x%x", emberGetNodeType());
  emberAfCorePrintln("          eui64: >%x%x%x%x%x%x%x%x",
                        emberGetEui64()[7],
                        emberGetEui64()[6],
                        emberGetEui64()[5],
                        emberGetEui64()[4],
                        emberGetEui64()[3],
                        emberGetEui64()[2],
                        emberGetEui64()[1],
                        emberGetEui64()[0]);
  emberAfCorePrintln("        node id: 0x%2x", emberGetNodeId());
  emberAfCorePrintln("         pan id: 0x%2x", emberGetPanId());
  emberAfCorePrintln("        channel: %d", (uint16_t)emberGetRadioChannel());
  emberAfCorePrintln("          power: %d", (uint16_t)emberGetRadioPower());

#ifdef EMBER_AF_PLUGIN_FREE_RTOS
  emberAfCorePrintln("rtos deep block: %l", rtosGetDeepSleepBlockCount());
  emberAfCorePrintln(" stk block deep: %l", stack_blocked_deep_sleep);
  emberAfCorePrintln(" stk allow deep: %l", stack_allowed_deep_sleep);
#endif
}

void counterCommand(void)
{
  uint8_t counterType = emberUnsignedCommandArgument(0);
  uint32_t counter;
  EmberStatus status = emberGetCounter(counterType, &counter);

  if (status != EMBER_SUCCESS) {
    emberAfCorePrintln("Get counter failed, status=0x%x", status);
  } else {
    emberAfCorePrintln("Counter type=0x%x: %d", counterType, counter);
  }
}
