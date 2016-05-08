// Sensor sample application.
//
// Copyright 2013 Silicon Laboratories, Inc.                                *80*

#include PLATFORM_HEADER
#include CONFIGURATION_HEADER


#include "em_emu.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "em_dbg.h"
#include "bsp.h"
#include "em_gpio.h"
#include "ustimer.h"
#include "em_timer.h"

#include "stack/include/ember.h"
#include "heartbeat/heartbeat.h"
#include "hal/hal.h"
#include "phy/phy.h"
#include "wstk-sensors/wstk-sensors.h"
#include "poll/poll.h"
#include "command-interpreter/command-interpreter.h"
#include "debug-print/debug-print.h"

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
#define SENSOR_PAIR_TIMEOUT_MS       (5 * MILLISECOND_TICKS_PER_SECOND)

#define NETWORK_UP_LED               BOARDLED0

#define SENSOR_SINK_SECURITY_KEY    {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,       \
                                     0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,       \
                                     0xAA, 0xAA, 0xAA, 0xAA}

///////////////// OUR PROJECT DEFINES ////////////////////////
// calibration constants for voltage and current measurements
#define CAL_FACTOR_CURRENT	0.2
#define CAL_FACTOR_12V		0.2
#define CAL_FACTOR_5V		0.2
#define CAL_FACTOR_SOLAR	0.2

// these are the GPIO control pin numbers on each specific port
#define LIGHT1_PIN 		4
#define LIGHT2_PIN 		5
#define USB1_PIN		14
#define USB2_PIN		6
#define SOLAR_PIN		6
#define VCTRL1_PIN 		1
#define VCTRL2_PIN 		2
#define ADC0_CH0_PIN	0
#define ADC0_CH1_PIN	1
#define ADC0_CH2_PIN	2
#define ADC0_CH3_PIN	3

GPIO_Port_TypeDef Lights_Port 		= gpioPortD;
GPIO_Port_TypeDef Usb1_Port 		= gpioPortA;
GPIO_Port_TypeDef Usb2_Port 		= gpioPortF;
GPIO_Port_TypeDef Solar_Port 		= gpioPortD;
GPIO_Port_TypeDef Voltage_Ctrl_Port = gpioPortF;
GPIO_Port_TypeDef ADC_Port			= gpioPortD;

enum {
  SENSOR_SINK_COMMAND_ID_ADVERTISE_REQUEST = 0,
  SENSOR_SINK_COMMAND_ID_ADVERTISE         = 1,
  SENSOR_SINK_COMMAND_ID_PAIR_REQUEST      = 2,
  SENSOR_SINK_COMMAND_ID_PAIR_CONFIRM      = 3,
  SENSOR_SINK_COMMAND_ID_DATA              = 4,
};
typedef uint8_t SensorSinkCommandId;

static EmberNodeId sinkNodeId = EMBER_NULL_NODE_ID;
static bool confirmed;
static EmberKeyData securityKey = SENSOR_SINK_SECURITY_KEY;
static EmberStatus send(EmberNodeId nodeId,
                        SensorSinkCommandId commandId,
                        uint8_t *buffer,
                        uint8_t bufferLength);
static EmberStatus requestAdvertise(void);
static uint16_t fetchLowHighInt16u(uint8_t *contents);
static void storeLowHighInt16u(uint8_t *contents, uint16_t value);

//////////////// OUR PROJECT FUNCTION PROTOTYPES ////////////////
void initSundialADC(void);
void initSundialGpio(void);
static uint32_t adc0TakeMeasurement(ADC_SingleInput_TypeDef channel);
static uint32_t takeCurrentMeasurement(GPIO_Port_TypeDef gpioPort, unsigned int gpioPin, ADC_SingleInput_TypeDef channel);
static uint32_t takeVoltageMeasurement(uint8_t vctrl1, uint8_t vctrl2);
static uint16_t calculatePower(uint32_t current, uint32_t voltage);
void takeMeasurementSet(uint16_t *power_meas);

static uint8_t message[SENSOR_SINK_MAXIMUM_LENGTH];
static EmberMessageLength messageLength;
static EmberMessageOptions txOptions = EMBER_OPTIONS_ACK_REQUESTED;
EmberEventControl reportControl;

//////////// OUR PROJECT GLOBAL VARIABLES /////////////////////
ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;


void reportHandler(void)
{
  if (!emberStackIsUp() || sinkNodeId == EMBER_NULL_NODE_ID || !confirmed) {
    emberEventControlSetInactive(reportControl);
    emberAfPluginPollEnableShortPolling(FALSE);
  } else {

    uint8_t payload[SENSOR_SINK_MAXIMUM_DATA_LENGTH];
    uint16_t measurements[5];  // array to store the most recent set of measurements
    uint16_t *measptr = measurements;

    // take measurements and fill measurements array
    takeMeasurementSet(measptr);
    // transfer measurements to payload
    int byteidx = 0;
    for (int i = 0; i < 5; i++)
    {
    	uint16_t temp = measurements[i];
    	payload[byteidx] = LOW_BYTE(temp);
    	payload[byteidx + 1] = HIGH_BYTE(temp);
    	byteidx += 2;
    }
    int rand_num = halCommonGetRandom();
    //emberAfCorePrint("rand_num is %d\n",rand_num);

    //USTIMER_Delay(120000*(halCommonGetRandom()%1000)); //delay is between 0 and 120 seconds(2minutes)	//works but not useful for demoing

     EmberStatus status = send(sinkNodeId,
                    SENSOR_SINK_COMMAND_ID_DATA,
                    payload,
                    SENSOR_SINK_MAXIMUM_DATA_LENGTH);
      emberAfCorePrint("TX: Data to 0x%2x:\n", sinkNodeId);

      emberEventControlSetDelayMS(reportControl, SENSOR_REPORT_PERIOD_MS);
  }
}

void initSundialADC(void)
{
	  /* Enable ADC clock */
	  CMU_ClockEnable(cmuClock_ADC0, true);

	  /* Customize ADC base configuration (from ADC_INIT_DEFAULT) */
	  init.timebase = ADC_TimebaseCalc(0);  		// uses current HFPER clock setting
	  init.prescale = ADC_PrescaleCalc(7000000, 0); // uses current HFPER clock setting to scale the desired ADC frequency
	  init.lpfMode =  adcLPFilterRC;				// use the on-chip RC filter
	  init.ovsRateSel = adcOvsRateSel16;			// avg 16 samples per conversion

	 /* Customize ADC Single Measurement configuration (from ADC_INITSINGLE_DEFAULT) */
	 singleInit.reference  	= adcRef2V5;			// use 2.5V for the reference voltage
	 singleInit.input      	= adcSingleInpCh0;		// default to CH0
	 singleInit.resolution 	= adcRes12Bit;			// 12-bit resolution
	 singleInit.acqTime 	= adcAcqTime32;			// 32 clock cycles
	 singleInit.rep			= false;				// not in repetitive mode

	 ADC_Init(ADC0, &init);
	 ADC_InitSingle(ADC0, &singleInit);
}

void initSundialGpio(void)
{
	/* initializes all GPIO pins for our project */

	// outputs (to toggle current measurements)
	/*
	 * GPIO_PinModeSet(GPIO_Port_TypeDef port,
	                     unsigned int pin,
	                     GPIO_Mode_TypeDef mode,
	                     unsigned int out);
	 */
	GPIO_PinModeSet(Lights_Port, LIGHT1_PIN, gpioModePushPull, 0);  // PD4  Note: 0 means output = LOW
	GPIO_PinModeSet(Lights_Port, LIGHT2_PIN, gpioModePushPull, 0);	// PD5
	GPIO_PinModeSet(Usb1_Port, USB1_PIN, gpioModePushPull, 0);		// PA14
	GPIO_PinModeSet(Usb2_Port, USB2_PIN, gpioModePushPull, 0);		// PF6
	GPIO_PinModeSet(Solar_Port, SOLAR_PIN, gpioModePushPull, 0);	// PD6

	// Disables LED0 to prevent interference with PF6
	BSP_LedClear(0);

	// Disables the debug interface (SWDIO, SWO) to prevent interference with PF1 and PF2
	// must wait 3 seconds before disabling the debug interface (see EZR32WG ref manual p.734).
	USTIMER_Delay(3000000);
	DBG_SWOEnable(0);
	GPIO_DbgSWDClkEnable(0);	// PF0
	GPIO_DbgSWDIOEnable(0);		// PF1
	GPIO_DbgSWOEnable(0);		// PF2

	// outputs (to toggle voltage measurements)
	GPIO_PinModeSet(Voltage_Ctrl_Port, VCTRL1_PIN, gpioModePushPull, 0);	// PF1
	GPIO_PinModeSet(Voltage_Ctrl_Port, VCTRL1_PIN, gpioModePushPull, 0);	// PF2

	// inputs (for voltage measurements)
	GPIO_PinModeSet(ADC_Port, ADC0_CH0_PIN, gpioModeInput, 0);		// PD0 - USB measurements
	GPIO_PinModeSet(ADC_Port, ADC0_CH1_PIN, gpioModeInput, 0);		// PD1 - Light measurements
	GPIO_PinModeSet(ADC_Port, ADC0_CH2_PIN, gpioModeInput, 0); 		// PD2 - Solar measurement
	GPIO_PinModeSet(ADC_Port, ADC0_CH3_PIN, gpioModeInput, 0); 		// PD3 - Supply voltages (5V, 12V)
}

static uint32_t adc0TakeMeasurement(ADC_SingleInput_TypeDef channel)
{

	/* Takes a measurement from ADC0 CH# depending on the specified channel */
	uint32_t data;

	// reset the ADC channel selection
	ADC0->SINGLECTRL = ADC_SINGLECTRL_INPUTSEL_DEFAULT;

	switch (channel)
	{
		case adcSingleInpCh0:
			ADC0->SINGLECTRL |= ADC_SINGLECTRL_INPUTSEL_CH0;
			break;

		case adcSingleInpCh1:
			ADC0->SINGLECTRL |= ADC_SINGLECTRL_INPUTSEL_CH1;
			break;

		case adcSingleInpCh2:
			ADC0->SINGLECTRL |= ADC_SINGLECTRL_INPUTSEL_CH2;
			break;

		case adcSingleInpCh3:
			ADC0->SINGLECTRL |= ADC_SINGLECTRL_INPUTSEL_CH3;
			break;
	}

	 while(((ADC0 -> STATUS & ADC_STATUS_WARM) != 1) && (ADC0 -> STATUS & ADC_STATUS_SINGLEREFWARM != 1)){
			//Wait until adc and reference are warmed up
		}
		ADC_Start(ADC0, adcStartSingle);     //Start a single conversion
		while (ADC0 -> STATUS & ADC_STATUS_SINGLEACT){
			//Wait until ADC0 finishes conversion
		}
		if (ADC0 -> STATUS & ADC_STATUS_SINGLEDV){ //Check to see if conversion result is valid
			  //emberAfCorePrintln("Valid Data");
		}
		data = ADC_DataSingleGet(ADC0);  //Get result from ADC data register
		return data;
}

static uint32_t takeCurrentMeasurement(GPIO_Port_TypeDef gpioPort, unsigned int gpioPin, ADC_SingleInput_TypeDef channel)
{

	/* sets the specified control pin high to take a current measurement */
	uint32_t measurement;
	GPIO_PinOutSet(gpioPort, gpioPin);
	USTIMER_Delay(50000);                       // wait 50ms for gpio pin to turn on switch
	measurement = adc0TakeMeasurement(channel); // take actual measurement
	GPIO_PinOutClear(gpioPort, gpioPin);       // reset control pin after measurement
	return measurement;
}

static uint32_t takeVoltageMeasurement(uint8_t vctrl2, uint8_t vctrl1)
{

	/* sets the specified control pins to take a voltage measurement.
	 *
	 * vctrl2 vctrl1
	 * 	0 		0 		= No connection
	 * 	0 		1 		= solar panel
	 * 	1 		0 		= 5V
	 * 	1 		1 		= 12V
	 *
	 */
	uint32_t measurement;

	if (vctrl1) GPIO_PinOutSet(Voltage_Ctrl_Port, VCTRL1_PIN);
	if (vctrl2) GPIO_PinOutSet(Voltage_Ctrl_Port, VCTRL2_PIN);
	USTIMER_Delay(50000);                       // wait 50ms for gpio pin to turn on switch
	measurement = adc0TakeMeasurement(adcSingleInpCh3);
	GPIO_PinOutClear(Voltage_Ctrl_Port, VCTRL1_PIN);
	GPIO_PinOutClear(Voltage_Ctrl_Port, VCTRL2_PIN);  // return voltage control to no connection after measurement
	return measurement;
}

static double calibrateCurrentMeasurement(uint32_t current)
{
	/* these are just function stubs for now */
	return (double) current;
}

static double calibrateVoltageMeasurement(uint32_t voltage, double cal_factor)
{
	/* these are just function stubs for now */
	return cal_factor * ((double) voltage);
}

static uint16_t calculatePower(uint32_t current, uint32_t voltage)
{
	/* Calculates the power consumed by the circuit in mW, given a current and voltage measurement.
	 *
	 * The voltage and current values are raw uint32_t directly from the ADC data register.
	 */
    return (uint16_t) ((0.2 * ((voltage*current*1250) / 4096))/1000);
}

void takeMeasurementSet(uint16_t *power_meas)
{
	/* takes the full suite of power measurements once every 15 minutes.
	 * In total this is 5 measurements.
	 */

	uint32_t current, voltage;

	// lights 1 power measurement
	emberAfCorePrintln("-------- Light1 --------");
	current = takeCurrentMeasurement(Lights_Port, LIGHT1_PIN, adcSingleInpCh1);
	voltage = takeVoltageMeasurement(1, 1);  // 1 1 = 12V
	power_meas[0] = calculatePower(current, voltage);
	emberAfCorePrintln("C:\t%d", current);
	emberAfCorePrintln("V:\t%d", voltage);
	emberAfCorePrintln("P:\t%d", power_meas[0]);

	// lights 2 measurement
	emberAfCorePrintln("-------- Light2 --------");
	current = takeCurrentMeasurement(Lights_Port, LIGHT2_PIN, adcSingleInpCh1);
	power_meas[1] = calculatePower(current, voltage);
	emberAfCorePrintln("C:\t%d", current);
	emberAfCorePrintln("V:\t%d", voltage);
	emberAfCorePrintln("P:\t%d", power_meas[1]);

	// usb 1 measurement
	emberAfCorePrintln("--------- USB1 ---------");
	current = takeCurrentMeasurement(Usb1_Port, USB1_PIN, adcSingleInpCh0);
	voltage = takeVoltageMeasurement(1, 0);  // 1 0 = 5V
	power_meas[2] = calculatePower(current, voltage);
	emberAfCorePrintln("C:\t%d", current);
	emberAfCorePrintln("V:\t%d", voltage);
	emberAfCorePrintln("P:\t%d", power_meas[2]);

	// usb 2 measurement
	emberAfCorePrintln("--------- USB2 ---------");
	current = takeCurrentMeasurement(Usb2_Port, USB2_PIN, adcSingleInpCh0);
	power_meas[3] = calculatePower(current, voltage);
	emberAfCorePrintln("C:\t%d", current);
	emberAfCorePrintln("V:\t%d", voltage);
	emberAfCorePrintln("P:\t%d", power_meas[3]);

	// solar panel measurement
	emberAfCorePrintln("-------- Solar ---------");
	current = takeCurrentMeasurement(Solar_Port, SOLAR_PIN, adcSingleInpCh2);
	voltage = takeVoltageMeasurement(0, 1);  // 0 1 = Solar (~5V)
	power_meas[4] = calculatePower(current, voltage);
	emberAfCorePrintln("C:\t%d", current);
	emberAfCorePrintln("V:\t%d", voltage);
	emberAfCorePrintln("P:\t%d", power_meas[4]);
}

void emberAfIncomingMessageCallback(EmberIncomingMessage *message)
{
  if (message->length < SENSOR_SINK_MINIMUM_LENGTH
      || (fetchLowHighInt16u(message->payload + SENSOR_SINK_PROTOCOL_ID_OFFSET)
          != SENSOR_SINK_PROTOCOL_ID)) {
    return;
  }

  switch (message->payload[SENSOR_SINK_COMMAND_ID_OFFSET]) {
  case SENSOR_SINK_COMMAND_ID_ADVERTISE_REQUEST:
    emberAfCorePrintln("RX: Advertise Request from 0x%2x", message->source);
    emberAfCorePrintln("SENSOR_SINK_COMMAND_ID_ADVERTISE_REQUEST");
    break;
  case SENSOR_SINK_COMMAND_ID_ADVERTISE:
    {
	    emberAfCorePrintln("sinkNodeId:%d", sinkNodeId);
	    emberAfCorePrintln("confirmed: %d",confirmed);
      emberAfCorePrintln("RX: Advertise from 0x%2x", message->source);

      if (sinkNodeId == EMBER_NULL_NODE_ID || !confirmed) {
        EmberStatus status = send(EMBER_COORDINATOR_ADDRESS,
                                  SENSOR_SINK_COMMAND_ID_PAIR_REQUEST,
                                  NULL,
                                  0);
        emberAfCorePrintln("TX: Pair Request to 0x%2x: 0x%x",
                              EMBER_COORDINATOR_ADDRESS,
                              status);
        if (status == EMBER_SUCCESS) {
          sinkNodeId = EMBER_COORDINATOR_ADDRESS;
          confirmed = FALSE;

          // If the node is sleepy switch to short poll mode to get the confirm
          // command.
          if (emberGetNodeType() == EMBER_STAR_SLEEPY_END_DEVICE) {
            emberAfPluginPollEnableShortPolling(TRUE);
          }
        }
      } else {
        emberAfPluginPollEnableShortPolling(FALSE);
      }
      emberAfCorePrintln("SENSOR_SINK_COMMAND_ID_ADVERTISE");
      break;
    }
  case SENSOR_SINK_COMMAND_ID_PAIR_REQUEST:
    emberAfCorePrintln("RX: Pair Request from 0x%2x", message->source);
    emberAfCorePrintln("SENSOR_SINK_COMMAND_ID_PAIR_REQUEST");
    break;
  case SENSOR_SINK_COMMAND_ID_PAIR_CONFIRM:
    {
      emberAfCorePrintln("RX: Pair Confirm from 0x%2x", message->source);
      if (message->source == sinkNodeId) {
        confirmed = TRUE;
        reportHandler();
        emberAfPluginPollEnableShortPolling(FALSE);
      }
      emberAfCorePrintln("SENSOR_SINK_COMMAND_ID_PAIR_CONFIRM");
      break;
    }
  case SENSOR_SINK_COMMAND_ID_DATA:
    {
      uint8_t i;
      emberAfCorePrint("RX: Data from 0x%2x:", message->source);
      for (i = SENSOR_SINK_DATA_OFFSET; i < message->length; i++) {
        emberAfCorePrint(" %x", message->payload[i]);
      }
      emberAfCorePrintln("");
      emberAfCorePrintln("SENSOR_SINK_COMMAND_ID_PAIR_CONFIRM");
      break;
    }
  default:
    emberAfCorePrintln("RX: Unknown from 0x%2x", message->source);
    break;
  }
}

#define MAX_TX_FAILURES     50
static uint8_t txFailureCount = 0;

void emberAfMessageSentCallback(EmberStatus status,
                                EmberOutgoingMessage *message)
{
  if (status != EMBER_SUCCESS) {
    emberAfCorePrintln("TX: 0x%x", status);
    txFailureCount++;
    if (SENSOR_SINK_MINIMUM_LENGTH <= message->length
        && (fetchLowHighInt16u(message->payload + SENSOR_SINK_PROTOCOL_ID_OFFSET)
            == SENSOR_SINK_PROTOCOL_ID
        && (message->payload[SENSOR_SINK_COMMAND_ID_OFFSET]
            == SENSOR_SINK_COMMAND_ID_DATA))
        && txFailureCount >= MAX_TX_FAILURES) {
      emberAfCorePrintln("EVENT: dropped sink 0x%2x", sinkNodeId);
      sinkNodeId = EMBER_NULL_NODE_ID;
    }
  } else {
    // Success: reset the failures count.
    txFailureCount = 0;
  }
}

void emberAfStackStatusCallback(EmberNetworkStatus status)
{
  switch (status) {
  case EMBER_NETWORK_UP:
    emberAfCorePrintln("Network up");

    // Unicast an advertise request to the sink.
    requestAdvertise();
    break;
  case EMBER_NETWORK_DOWN:
    sinkNodeId = EMBER_NULL_NODE_ID;
    emberAfCorePrintln("Network down");
    //emberResetNetworkState();
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
	CMU_ClockEnable(cmuClock_HFPER, true);
	USTIMER_Init();  //calibrate delay timer
	initSundialADC();
	initSundialGpio();

	 emberAfCorePrintln("\n%p>", EMBER_AF_DEVICE_NAME);
	 emberAfCorePrintln("INIT: %p", EMBER_AF_DEVICE_NAME);

	 EmberStatus status = emberNetworkInit();//try to rejoin old network


	 ////different after this line
	 EmberNetworkParameters parameters;
	 /*MEMSET(&parameters, 0, sizeof(EmberNetworkParameters));
	    parameters.radioTxPower = 0;
	    parameters.radioChannel = 1;
	    parameters.panId = 0x01FF;     //Same as sink pan_id
	*/
	    emberAfCorePrintln("ember join status: %x", status);

	   /*
	    while(status == EMBER_NOT_JOINED){
		 emberJoinNetwork(EMBER_STAR_END_DEVICE, &parameters);
		 //emberAfCorePrintln("ember join status: %x", status);
		 //emberAfCorePrintln("infinite loop");
	 }*/
}


// This callback is called in each iteration of the main application loop and
// can be used to perform periodic functions.
void emberAfMainTickCallback(void)
{
	/* DISABLED interaction with ON-BOARD LED
  if (emberStackIsUp())
    halSetLed(NETWORK_UP_LED);
  else
    halClearLed(NETWORK_UP_LED);
    */
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

static EmberStatus requestAdvertise(void)
{
  EmberStatus status = send(EMBER_COORDINATOR_ADDRESS,
                            SENSOR_SINK_COMMAND_ID_ADVERTISE_REQUEST,
                            NULL,
                            0);
  // Enable short poll to get the advertise unicast. This implicitly kicks
  // off the periodic polling.
  if (emberGetNodeType() == EMBER_STAR_SLEEPY_END_DEVICE) {
    emberAfPluginPollEnableShortPolling(TRUE);
  }
  // Use the report event to timeout on the pairing process.
  emberEventControlSetDelayMS(reportControl, SENSOR_PAIR_TIMEOUT_MS);
  emberAfCorePrintln("TX: Advertise Request (unicast), status 0x%x",
                        status);

  return status;
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
// CLI commands handlers

void joinCommand(void)
{
  EmberStatus status;
  EmberNetworkParameters parameters;

  // Initialize the security key to the default key prior to joining the
  // network.
  emberSetSecurityKey(&securityKey);

  MEMSET(&parameters, 0, sizeof(EmberNetworkParameters));
  parameters.radioTxPower = SENSOR_SINK_TX_POWER;
  parameters.radioChannel = emberUnsignedCommandArgument(0);
  parameters.panId = SENSOR_SINK_PAN_ID;
  status = emberJoinNetwork(EMBER_STAR_END_DEVICE, &parameters);
  emberAfCorePrintln("join end device 0x%x", status);
}

void joinSleepyCommand(void)
{
  EmberStatus status;
  EmberNetworkParameters parameters;

  // Initialize the security key to the default key prior to joining the
  // network.
  emberSetSecurityKey(&securityKey);

  MEMSET(&parameters, 0, sizeof(EmberNetworkParameters));
  parameters.radioTxPower = SENSOR_SINK_TX_POWER;
  parameters.radioChannel = emberUnsignedCommandArgument(0);
  parameters.panId = SENSOR_SINK_PAN_ID;
  status = emberJoinNetwork(EMBER_STAR_SLEEPY_END_DEVICE, &parameters);
  emberAfCorePrintln("join sleepy 0x%x", status);
}

void joinRangeExtenderCommand(void)
{
  EmberStatus status;
  EmberNetworkParameters parameters;

  // Initialize the security key to the default key prior to joining the
  // network.
  emberSetSecurityKey(&securityKey);

  MEMSET(&parameters, 0, sizeof(EmberNetworkParameters));
  parameters.radioTxPower = SENSOR_SINK_TX_POWER;
  parameters.radioChannel = emberUnsignedCommandArgument(0);
  parameters.panId = SENSOR_SINK_PAN_ID;
  status = emberJoinNetwork(EMBER_STAR_RANGE_EXTENDER, &parameters);
  emberAfCorePrintln("join range extender 0x%x", status);
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

void advertiseRequestCommand(void)
{
  requestAdvertise();
}

  void dataCommand(void)
{
  emberEventControlSetActive(reportControl);
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
  emberAfCorePrintln("     TX options: MAC acks %s, security %s, priority %s",
                     ((txOptions & EMBER_OPTIONS_ACK_REQUESTED) ? "enabled" : "disabled"),
                     ((txOptions & EMBER_OPTIONS_SECURITY_ENABLED) ? "enabled" : "disabled"),
                     ((txOptions & EMBER_OPTIONS_HIGH_PRIORITY) ? "enabled" : "disabled"));

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
