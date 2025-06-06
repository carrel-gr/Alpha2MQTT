/*
Name:		Definitions.h
Created:	24/Aug/2022
Author:		Daniel Young

This file is part of Alpha2MQTT (A2M) which is released under GNU GENERAL PUBLIC LICENSE.
See file LICENSE or go to https://choosealicense.com/licenses/gpl-3.0/ for full license details.

Notes

Customise these options as per README.txt.  Please read README.txt before continuing.
*/

#ifndef _Definitions_h
#define _Definitions_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


/*************************************************/
/* Start customizations here.                    */
/*************************************************/

// Compiling for ESP8266 or ESP32?
#define MP_ESP32
//#define MP_ESP8266
#define MP_XIAO_ESP32C6

// Display parameters - Set LARGE_DISPLAY for 128x64 oled
// Don't set this if using the ESP8266 OLED Shield 64x48 display.
#define LARGE_DISPLAY

// If your OLED does have an RST pin, set this.
// An OLED Shield compatible with an ESP8266 does have a RESET pin and it is linked to GPIO0 if using an ESP8266.
// If you are using the same OLED Shield with an ESP32, by default for this project it is linked to GIO13.
#define OLED_RST_PIN -1    // No RST pin
//#define OLED_RST_PIN 0
//#define OLED_RST_PIN 13

// Set this to true to set the "retain" flag when publishing to MQTT
// "retain" is also a flag in mqttState.   This is AND-ed with that.
// This does NOT control whether we tell HA to add "retain" to command topics that it publishes back to us.
#define MQTT_RETAIN true
// Set this to 1 or 0
#define MQTT_SUBSCRIBE_QOS 1
// Define this to make Home Assistant be the "authority" for OpMode and related values.
// When HA is the authority, then on reboot, let HA tell us what our OpMode is.
// When HA is not the authority, then on reboot, read the ESS state and tell HA.
// This controls whether or not we tell HA to add "retain" to command topics that it publishes back to us.
#define HA_IS_OP_MODE_AUTHORITY
// Define this to enable the HA force_update option for many (not all) sensors.
//#define MQTT_FORCE_UPDATE

// The ESP8266 has limited memory and so reserving lots of RAM to build a payload and MQTT buffer causes out of memory exceptions.
// 4096 works well given the RAM requirements of Alpha2MQTT.
// If you aren't using an ESP8266 you may be able to increase this.
// Alpha2MQTT on boot will request a buffer size of (MAX_MQTT_PAYLOAD_SIZE + MQTT_HEADER_SIZE) for MQTT, and
// MAX_MQTT_PAYLOAD_SIZE for building payloads.  If these fail and your device doesn't boot, you can assume you've set this too high.
#define MAX_MQTT_PAYLOAD_SIZE 4096
#define MIN_MQTT_PAYLOAD_SIZE 512
#define MQTT_HEADER_SIZE 512

//#define DEBUG_OVER_SERIAL	// Enable debugging msgs over serial port
//#define DEBUG_LEVEL2		// For serial flooding action
//#define DEBUG_OUTPUT_TX_RX	// write RS485 frames to serial debugging port
//#define DEBUG_FREEMEM	// Enable extra debug on display and via MQTT
//#define DEBUG_WIFI	// Enable extra debug on display and via MQTT
//#define DEBUG_CALLBACKS	// Enable extra debug on display and via MQTT
//#define DEBUG_OPS     // Enable extra debug for opModes
//#define DEBUG_RS485	// Enable extra debug on display and via MQTT
//#define DEBUG_NO_RS485	// Fake-out all RS485 comms

// The SOC Target value is a percent value.  Define MIN/MAX range for what HA can use.
#define SOC_TARGET_MAX 100
#define SOC_TARGET_MIN 30

// Set your EMS version.  Either EMS2.5 or EMS3.5/EMS3.6
// If 3.5/3.6 is not set, then you get 2.5
#define EMS_35_36

// Set your inverters MAX power output.
// MAX power is the value that AlphaESS rates that product at.
//	TBD: There should be a way to read this from the ESS.
//	It appears that for the SMILE-SP(B) the 2nd-5th digits of the inverter (not EMS) serial number, are the max power
//	e.g. "79600ABP225R0061"  <- max power is 9600
#define INVERTER_POWER_MAX 9600

// If values for some registers such as voltage or temperatures appear to be out by a decimal place or two, try the following:

// Documentation declares 1V - However Presume 0.1 as result appears to reflect this.  I.e. my voltage reading was 2421, * 0.1 for 242.1
// However EMS3.5/EMS3.6 seems to follow the spec.
#ifdef EMS_35_36
#define GRID_VOLTAGE_MULTIPLIER 1.0
#else // EMS_35_36
#define GRID_VOLTAGE_MULTIPLIER 0.1
#endif // EMS_35_36
// Documentation declares 0.001V - My min cell voltage is reading as 334, so * 0.001 = 0.334V.  I consider the document wrong, think it should be 0.01
// However EMS3.5/EMS3.6 seems to follow the spec.
#ifdef EMS_35_36
#define CELL_VOLTAGE_MULTIPLIER 0.001
#else // EMS_35_36
#define CELL_VOLTAGE_MULTIPLIER 0.01
#endif // EMS_35_36
// Documentation declares 0.1D - Mine returns 2720, so assuming actually multiplied by 0.01 to bring to something realistic
// However EMS3.5/EMS3.6 seems to follow the spec.
#ifdef EMS_35_36
#define INVERTER_TEMP_MULTIPLIER 0.1
#else // EMS_35_36
#define INVERTER_TEMP_MULTIPLIER 0.01
#endif // EMS_35_36
// Documentation declares 0.1kWh - My value was 308695, and according to web interface my total PV is 3086kWh, so multiplier seems wrong
// Note: For EMS3.5/EMS3.6 this seems to use 0.01 and NOT the spec value of 0.1
#define TOTAL_ENERGY_MULTIPLIER 0.01
// Doc declares 0.1 %/bit for REG_TIMING_RW_CHARGE_CUT_SOC but it seems to really be 1.0
#define CHARGE_CUT_SOC_MULTIPLIER 1
// Most all battery SOC values use this multiplier.  Do note that DISPATCH_SOC_MULTIPLIER (below) is different
#define BATTERY_SOC_MULTIPLIER 0.1
#define BATTERY_KWH_MULTIPLIER 0.1
#define BATTERY_TEMP_MULTIPLIER 0.1
#define FREQUENCY_MULTIPLIER 0.01

// I beg to differ on this, I'd say it's more 0.396 based on my tests
// However make it easily customisable here
#define DISPATCH_SOC_MULTIPLIER 0.4
//#define DISPATCH_SOC_MULTIPLIER 0.396

// After some liaison with a user of Alpha2MQTT on a 115200 baud rate, this fixed inconsistent retrieval
// such as sporadic NO-RESP.  It works by introducing a delay between requests sent to the inverter meaning it
// has time to 'breathe'
// 80ms is a default starting point which is 1/12 of a second.  If it corrects the issue try reducing the delay to 60, 40, etc until you find a happy place.
// If you want to make use of it, uncomment the next line and change 80 as necessary
//#define REQUIRED_DELAY_DUE_TO_INCONSISTENT_RETRIEVAL 80

// The device name is used as the MQTT base topic and presence on the network.
// You can have more than one Alpha2MQTT on your network without changing this.
#define DEVICE_NAME "Alpha2MQTT"

// Default address of inverter is 0x55 as per Alpha Modbus documentation.  If you have altered it, reflect that change here.
#define ALPHA_SLAVE_ID 0x55

// x 50mS to wait for RS485 input chars.  300ms as per Modbus documentation, but I got timeouts on that.  However 400ms works without issue
#define RS485_TRIES 8 // 16

// A user informed me that their router leverages leases on network connections which can't be disabled.
// I.e. when lease expires, WiFi doesn't drop but data stops.
// If FORCE_RESTART_HOURS is defined, then the system will auto-reboot every X many hours as configured in FORCE_RESTART_HOURS
//#define FORCE_RESTART_HOURS 49

/*************************************************/
/* Shouldn't need to change anything below this. */
/*************************************************/

#if defined MP_XIAO_ESP32C6 && ! defined MP_ESP32
#define MP_ESP32
#endif // MP_XIAO_ESP32C6 && ! MP_ESP32

#ifdef MP_XIAO_ESP32C6
#define BUTTON_PIN 9 // BOOT/GPIO9
#endif // MP_XIAO_ESP32C6

#if (!defined MP_ESP8266) && (!defined MP_ESP32)
#error You must specify the microprocessor in use
#endif
#if (defined MP_ESP8266) && (defined MP_ESP32)
#error You must only select one microprocessor from the list
#endif

#ifdef LARGE_DISPLAY
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#else // LARGE_DISPLAY
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 48
#endif // LARGE_DISPLAY
#define SCREEN_ADDRESS 0x3C

// Handled Registers as per 1.23 documentation
// Network meter - configuration
#define REG_GRID_METER_RW_GRID_METER_CT_ENABLE										0x0000	// 1/bit								// 2 Bytes		// Unsigned Short
#define REG_GRID_METER_RW_GRID_METER_CT_RATE										0x0001	// 1/bit								// 2 Bytes		// Unsigned Short

// Network meter - operation						
#define REG_GRID_METER_R_TOTAL_ENERGY_FEED_TO_GRID_1								0x0010	// 0.01kWh/bit							// 4 Bytes		// Unsigned Integer
//#define REG_GRID_METER_R_TOTAL_ENERGY_FEED_TO_GRID_2								0x0011	// 0.01kWh/bit						
#define REG_GRID_METER_R_TOTAL_ENERGY_CONSUMED_FROM_GRID_1							0x0012	// 0.01kWh/bit							// 4 Bytes		// Unsigned Integer
//#define REG_GRID_METER_R_TOTAL_ENERGY_CONSUMED_FROM_GRID_2							0x0013	// 0.01kWh/bit						
#define REG_GRID_METER_R_VOLTAGE_OF_A_PHASE											0x0014	// 1V									// 2 Bytes		// Unsigned Short
#define REG_GRID_METER_R_VOLTAGE_OF_B_PHASE											0x0015	// 1V									// 2 Bytes		// Unsigned Short
#define REG_GRID_METER_R_VOLTAGE_OF_C_PHASE											0x0016	// 1V									// 2 Bytes		// Unsigned Short
#define REG_GRID_METER_R_CURRENT_OF_A_PHASE											0x0017	// 0.1A									// 2 Bytes		// Short
#define REG_GRID_METER_R_CURRENT_OF_B_PHASE											0x0018	// 0.1A									// 2 Bytes		// Short
#define REG_GRID_METER_R_CURRENT_OF_C_PHASE											0x0019	// 0.1A									// 2 Bytes		// Short
#define REG_GRID_METER_R_FREQUENCY													0x001A	// 0.01Hz								// 2 Bytes		// Unsigned Short
#define REG_GRID_METER_R_ACTIVE_POWER_OF_A_PHASE_1									0x001B	// 1W/bit								// 4 Bytes		// Integer
//#define REG_GRID_METER_R_ACTIVE_POWER_OF_A_PHASE_2									0x001C	// 1W/bit						
#define REG_GRID_METER_R_ACTIVE_POWER_OF_B_PHASE_1									0x001D	// 1W/bit								// 4 Bytes		// Integer
//#define REG_GRID_METER_R_ACTIVE_POWER_OF_B_PHASE_2									0x001E	// 1W/bit						
#define REG_GRID_METER_R_ACTIVE_POWER_OF_C_PHASE_1									0x001F	// 1W/bit								// 4 Bytes		// Integer
//#define REG_GRID_METER_R_ACTIVE_POWER_OF_C_PHASE_2									0x0020	// 1W/bit						
#define REG_GRID_METER_R_TOTAL_ACTIVE_POWER_1										0x0021	// 1W/bit								// 4 Bytes		// Integer
//#define REG_GRID_METER_R_TOTAL_ACTIVE_POWER_2										0x0022	// 1W/bit						
#define REG_GRID_METER_R_REACTIVE_POWER_OF_A_PHASE_1								0x0023	// 1var									// 4 Bytes		// Integer
//#define REG_GRID_METER_R_REACTIVE_POWER_OF_A_PHASE_2								0x0024	// 1var						
#define REG_GRID_METER_R_REACTIVE_POWER_OF_B_PHASE_1								0x0025	// 1var									// 4 Bytes		// Integer
//#define REG_GRID_METER_R_REACTIVE_POWER_OF_B_PHASE_2								0x0026	// 1var						
#define REG_GRID_METER_R_REACTIVE_POWER_OF_C_PHASE_1								0x0027	// 1var									// 4 Bytes		// Integer
//#define REG_GRID_METER_R_REACTIVE_POWER_OF_C_PHASE_2								0x0028	// 1var						
#define REG_GRID_METER_R_TOTAL_REACTIVE_POWER_1										0x0029	// 1var									// 4 Bytes		// Integer
//#define REG_GRID_METER_R_TOTAL_REACTIVE_POWER_2										0x002A	// 1var						
#define REG_GRID_METER_R_APPARENT_POWER_OF_A_PHASE_1								0x002B	// 1VA									// 4 Bytes		// Integer
//#define REG_GRID_METER_R_APPARENT_POWER_OF_A_PHASE_2								0x002C	// 1VA						
#define REG_GRID_METER_R_APPARENT_POWER_OF_B_PHASE_1								0x002D	// 1VA									// 4 Bytes		// Integer
//#define REG_GRID_METER_R_APPARENT_POWER_OF_B_PHASE_2								0x002E	// 1VA						
#define REG_GRID_METER_R_APPARENT_POWER_OF_C_PHASE_1								0x002F	// 1VA									// 4 Bytes		// Integer
//#define REG_GRID_METER_R_APPARENT_POWER_OF_C_PHASE_2								0x0030	// 1VA						
#define REG_GRID_METER_R_TOTAL_APPARENT_POWER_1										0x0031	// 1VA									// 4 Bytes		// Integer
//#define REG_GRID_METER_R_TOTAL_APPARENT_POWER_2										0x0032	// 1VA						
#define REG_GRID_METER_R_POWER_FACTOR_OF_A_PHASE									0x0033	// 0.01									// 2 Bytes		// Short
#define REG_GRID_METER_R_POWER_FACTOR_OF_B_PHASE									0x0034	// 0.01									// 2 Bytes		// Short
#define REG_GRID_METER_R_POWER_FACTOR_OF_C_PHASE									0x0035	// 0.01									// 2 Bytes		// Short
#define REG_GRID_METER_R_TOTAL_POWER_FACTOR											0x0036	// 0.01									// 2 Bytes		// Short

// PV meter - configuration						
#define REG_PV_METER_RW_PV_METER_CT_ENABLE											0x0080	// 1/bit								// 2 Bytes		// Unsigned Short
#define REG_PV_METER_RW_PV_METER_CT_RATE											0x0081	// 1/bit								// 2 Bytes		// Unsigned Short

// PV meter - operation						
#define REG_PV_METER_R_TOTAL_ENERGY_FEED_TO_GRID_1									0x0090	// 0.01kWh/bit							// 4 Bytes		// Unsigned Integer
//#define REG_PV_METER_R_TOTAL_ENERGY_FEED_TO_GRID_2									0x0091	// 0.01kWh/bit						
#define REG_PV_METER_R_TOTAL_ENERGY_CONSUMED_FROM_GRID_1							0x0092	// 0.01kWh/bit							// 4 Bytes		// Unsigned Integer
//#define REG_PV_METER_R_TOTAL_ENERGY_CONSUMED_FROM_GRID_2							0x0093	// 0.01kWh/bit						
#define REG_PV_METER_R_VOLTAGE_OF_A_PHASE											0x0094	// 1V									// 2 Bytes		// Unsigned Short
#define REG_PV_METER_R_VOLTAGE_OF_B_PHASE											0x0095	// 1V									// 2 Bytes		// Unsigned Short
#define REG_PV_METER_R_VOLTAGE_OF_C_PHASE											0x0096	// 1V									// 2 Bytes		// Unsigned Short
#define REG_PV_METER_R_CURRENT_OF_A_PHASE											0x0097	// 0.1A									// 2 Bytes		// Short
#define REG_PV_METER_R_CURRENT_OF_B_PHASE											0x0098	// 0.1A									// 2 Bytes		// Short
#define REG_PV_METER_R_CURRENT_OF_C_PHASE											0x0099	// 0.1A									// 2 Bytes		// Short
#define REG_PV_METER_R_FREQUENCY													0x009A	// 0.01Hz								// 2 Bytes		// Unsigned Short
#define REG_PV_METER_R_ACTIVE_POWER_OF_A_PHASE_1									0x009B	// 1W/bit								// 4 Bytes		// Integer
//#define REG_PV_METER_R_ACTIVE_POWER_OF_A_PHASE_2									0x009C	// 1W/bit						
#define REG_PV_METER_R_ACTIVE_POWER_OF_B_PHASE_1									0x009D	// 1W/bit								// 4 Bytes		// Integer
//#define REG_PV_METER_R_ACTIVE_POWER_OF_B_PHASE_2									0x009E	// 1W/bit						
#define REG_PV_METER_R_ACTIVE_POWER_OF_C_PHASE_1									0x009F	// 1W/bit								// 4 Bytes		// Integer
//#define REG_PV_METER_R_ACTIVE_POWER_OF_C_PHASE_2									0x00A0	// 1W/bit						
#define REG_PV_METER_R_TOTAL_ACTIVE_POWER_1											0x00A1	// 1W/bit								// 4 Bytes		// Integer
//#define REG_PV_METER_R_TOTAL_ACTIVE_POWER_2											0x00A2	// 1W/bit						
#define REG_PV_METER_R_REACTIVE_POWER_OF_A_PHASE_1									0x00A3	// 1var									// 4 Bytes		// Integer
//#define REG_PV_METER_R_REACTIVE_POWER_OF_A_PHASE_2									0x00A4	// 1var						
#define REG_PV_METER_R_REACTIVE_POWER_OF_B_PHASE_1									0x00A5	// 1var									// 4 Bytes		// Integer
//#define REG_PV_METER_R_REACTIVE_POWER_OF_B_PHASE_2									0x00A6	// 1var						
#define REG_PV_METER_R_REACTIVE_POWER_OF_C_PHASE_1									0x00A7	// 1var									// 4 Bytes		// Integer
//#define REG_PV_METER_R_REACTIVE_POWER_OF_C_PHASE_2									0x00A8	// 1var						
#define REG_PV_METER_R_TOTAL_REACTIVE_POWER_1										0x00A9	// 1var									// 4 Bytes		// Integer
//#define REG_PV_METER_R_TOTAL_REACTIVE_POWER_2										0x00AA	// 1var						
#define REG_PV_METER_R_APPARENT_POWER_OF_A_PHASE_1									0x00AB	// 1VA									// 4 Bytes		// Integer
//#define REG_PV_METER_R_APPARENT_POWER_OF_A_PHASE_2									0x00AC	// 1VA						
#define REG_PV_METER_R_APPARENT_POWER_OF_B_PHASE_1									0x00AD	// 1VA									// 4 Bytes		// Integer
//#define REG_PV_METER_R_APPARENT_POWER_OF_B_PHASE_2									0x00AE	// 1VA						
#define REG_PV_METER_R_APPARENT_POWER_OF_C_PHASE_1									0x00AF	// 1VA									// 4 Bytes		// Integer
//#define REG_PV_METER_R_APPARENT_POWER_OF_C_PHASE_2									0x00B0	// 1VA						
#define REG_PV_METER_R_TOTAL_APPARENT_POWER_1										0x00B1	// 1VA									// 4 Bytes		// Integer
//#define REG_PV_METER_R_TOTAL_APPARENT_POWER_2										0x00B2	// 1VA						
#define REG_PV_METER_R_POWER_FACTOR_OF_A_PHASE										0x00B3	// 0.01									// 2 Bytes		// Short
#define REG_PV_METER_R_POWER_FACTOR_OF_B_PHASE										0x00B4	// 0.01									// 2 Bytes		// Short
#define REG_PV_METER_R_POWER_FACTOR_OF_C_PHASE										0x00B5	// 0.01									// 2 Bytes		// Short
#define REG_PV_METER_R_TOTAL_POWER_FACTOR											0x00B6	// 0.01									// 2 Bytes		// Short

// Battery - HOME Series						
#define REG_BATTERY_HOME_R_VOLTAGE													0x0100	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_CURRENT													0x0101	// 0.1A/bit								// 2 Bytes		// Short
#define REG_BATTERY_HOME_R_SOC														0x0102	// 0.1/bit								// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_STATUS													0x0103	// <<Note1 - BATTERY STATUS LOOKUP>>	// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_RELAY_STATUS												0x0104	// <<Note2 - BATTERY RELAY STATUS LU>>	// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_PACK_ID_OF_MIN_CELL_VOLTAGE								0x0105	// 0.001V/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_CELL_ID_OF_MIN_CELL_VOLTAGE								0x0106	// 0.001V/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_MIN_CELL_VOLTAGE											0x0107	// 0.001V/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_PACK_ID_OF_MAX_CELL_VOLTAGE								0x0108	// 0.001V/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_CELL_ID_OF_MAX_CELL_VOLTAGE								0x0109	// 0.001V/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_MAX_CELL_VOLTAGE											0x010A	// 0.001V/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_PACK_ID_OF_MIN_CELL_TEMPERATURE							0x010B	// 0.001D/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_CELL_ID_OF_MIN_CELL_TEMPERATURE							0x010C	// 0.001D/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_MIN_CELL_TEMPERATURE										0x010D	// 0.001D/bit							// 2 Bytes		// Short
#define REG_BATTERY_HOME_R_PACK_ID_OF_MAX_CELL_TEMPERATURE							0x010E	// 0.001D/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_CELL_ID_OF_MAX_CELL_TEMPERATURE							0x010F	// 0.001D/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_MAX_CELL_TEMPERATURE										0x0110	// 0.001D/bit							// 2 Bytes		// Short
#define REG_BATTERY_HOME_R_MAX_CHARGE_CURRENT										0x0111	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_MAX_DISCHARGE_CURRENT									0x0112	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_CHARGE_CUT_OFF_VOLTAGE									0x0113	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_DISCHARGE_CUT_OFF_VOLTAGE								0x0114	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BMU_SOFTWARE_VERSION										0x0115	//										// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_LMU_SOFTWARE_VERSION										0x0116	//										// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_ISO_SOFTWARE_VERSION										0x0117	//										// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_NUMBER											0x0118	// Battery modules number				// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_CAPACITY											0x0119	// 0.1kWh/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_TYPE												0x011A	// <<Note3 - BATTERY TYPE LOOKUP>>		// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_SOH												0x011B	// 0.1/bit								// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_WARNING_1										0x011C	// Reserve								// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_WARNING_2										0x011D	// Reserve
#define REG_BATTERY_HOME_R_BATTERY_FAULT_1											0x011E	// <<Note4 - BATTERY ERROR LOOKUP>>		// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_FAULT_2											0x011F	// <<Note4 - BATTERY ERROR LOOKUP>>
#define REG_BATTERY_HOME_R_BATTERY_CHARGE_ENERGY_1									0x0120	// 0.1kWh/bit							// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_CHARGE_ENERGY_2									0x0121	// 0.1kWh/bit
#define REG_BATTERY_HOME_R_BATTERY_DISCHARGE_ENERGY_1								0x0122	// 0.1kWh/bit							// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_DISCHARGE_ENERGY_2								0x0123	// 0.1kWh/bit
#define REG_BATTERY_HOME_R_BATTERY_ENERGY_CHARGE_FROM_GRID_1						0x0124	// 0.1kWh/bit							// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_ENERGY_CHARGE_FROM_GRID_2						0x0125	// 0.1kWh/bit
#define REG_BATTERY_HOME_R_BATTERY_POWER											0x0126	// 1W/bit, - Charge, + Discharge		// 2 Bytes		// Short
#define REG_BATTERY_HOME_R_BATTERY_REMAINING_TIME									0x0127	// 1 minute/bit							// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_IMPLEMENTATION_CHARGE_SOC						0x0128	// 0.1/bit (Rate_SOC UPS_SOC)			// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_IMPLEMENTATION_DISCHARGE_SOC						0x0129	// 0.1/bit (Rate_SOC UPS_SOC)			// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_REMAINING_CHARGE_SOC								0x012A	// 0.1/bit (Rate_SOC Remain_SOC)		// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_REMAINING_DISCHARGE_SOC							0x012B	// 0.1/bit (Remain_SOC UPS_SOC)			// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_MAX_CHARGE_POWER									0x012C	// 1W/bit								// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_MAX_DISCHARGE_POWER								0x012D	// 1W/bit								// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_RW_BATTERY_MOS_CONTROL										0x012E	// <<BATTERY MOS CONTROL LOOKUP>>		// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_SOC_CALIBRATION									0x012F	// <<BATTERY SOC CALIBRATION LOOKUP>>	// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_SINGLE_CUT_ERROR_CODE							0x0130	// 										// 2 Bytes		// Unsigned Short
#define REG_BATTERY_HOME_R_BATTERY_FAULT_1_1										0x0131	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_FAULT_1_2										0x0132	//
#define REG_BATTERY_HOME_R_BATTERY_FAULT_2_1										0x0133	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_FAULT_2_2										0x0134	//
#define REG_BATTERY_HOME_R_BATTERY_FAULT_3_1										0x0135	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_FAULT_3_2										0x0136	//
#define REG_BATTERY_HOME_R_BATTERY_FAULT_4_1										0x0137	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_FAULT_4_2										0x0138	//
#define REG_BATTERY_HOME_R_BATTERY_FAULT_5_1										0x0139	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_FAULT_5_2										0x013A	//
#define REG_BATTERY_HOME_R_BATTERY_FAULT_6_1										0x013B	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_FAULT_6_2										0x013C	//
#define REG_BATTERY_HOME_R_BATTERY_WARNING_1_1										0x013D	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_WARNING_1_2										0x013E	//
#define REG_BATTERY_HOME_R_BATTERY_WARNING_2_1										0x013F	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_WARNING_2_2										0x0140	//
#define REG_BATTERY_HOME_R_BATTERY_WARNING_3_1										0x0141	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_WARNING_3_2										0x0142	//
#define REG_BATTERY_HOME_R_BATTERY_WARNING_4_1										0x0143	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_WARNING_4_2										0x0144	//
#define REG_BATTERY_HOME_R_BATTERY_WARNING_5_1										0x0145	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_WARNING_5_2										0x0146	//
#define REG_BATTERY_HOME_R_BATTERY_WARNING_6_1										0x0147	// 										// 4 Bytes		// Unsigned Integer
//#define REG_BATTERY_HOME_R_BATTERY_WARNING_6_2										0x0148	//

// Inverter - HOME Series	
#define REG_INVERTER_HOME_R_VOLTAGE_L1												0x0400	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_VOLTAGE_L2												0x0401	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_VOLTAGE_L3												0x0402	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_CURRENT_L1												0x0403	// 0.1A/bit								// 2 Bytes		// Short
#define REG_INVERTER_HOME_R_CURRENT_L2												0x0404	// 0.1A/bit								// 2 Bytes		// Short
#define REG_INVERTER_HOME_R_CURRENT_L3												0x0405	// 0.1A/bit								// 2 Bytes		// Short
#define REG_INVERTER_HOME_R_POWER_L1_1												0x0406	// 1W/bit								// 4 Bytes		// Integer
//#define REG_INVERTER_HOME_R_POWER_L1_2												0x0407	// 1W/bit
#define REG_INVERTER_HOME_R_POWER_L2_1												0x0408	// 1W/bit								// 4 Bytes		// Integer
//#define REG_INVERTER_HOME_R_POWER_L2_2												0x0409	// 1W/bit
#define REG_INVERTER_HOME_R_POWER_L3_1												0x040A	// 1W/bit								// 4 Bytes		// Integer
//#define REG_INVERTER_HOME_R_POWER_L3_2												0x040B	// 1W/bit
#define REG_INVERTER_HOME_R_POWER_TOTAL_1											0x040C	// 1W/bit								// 4 Bytes		// Integer
//#define REG_INVERTER_HOME_R_POWER_TOTAL_2											0x040D	// 1W/bit
#define REG_INVERTER_HOME_R_BACKUP_VOLTAGE_L1										0x040E	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_BACKUP_VOLTAGE_L2										0x040F	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_BACKUP_VOLTAGE_L3										0x0410	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_BACKUP_CURRENT_L1										0x0411	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_BACKUP_CURRENT_L2										0x0412	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_BACKUP_CURRENT_L3										0x0413	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_BACKUP_POWER_L1_1										0x0414	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_BACKUP_POWER_L1_2										0x0415	// 1W/bit
#define REG_INVERTER_HOME_R_BACKUP_POWER_L2_1										0x0416	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_BACKUP_POWER_L2_2										0x0417	// 1W/bit
#define REG_INVERTER_HOME_R_BACKUP_POWER_L3_1										0x0418	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_BACKUP_POWER_L3_2										0x0419	// 1W/bit
#define REG_INVERTER_HOME_R_BACKUP_POWER_TOTAL_1									0x041A	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_BACKUP_POWER_TOTAL_2									0x041B	// 1W/bit
#define REG_INVERTER_HOME_R_FREQUENCY												0x041C	// 0.01Hz/bit							// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV1_VOLTAGE												0x041D	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV1_CURRENT												0x041E	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV1_POWER_1												0x041F	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_PV1_POWER_2												0x0420	// 1W/bit
#define REG_INVERTER_HOME_R_PV2_VOLTAGE												0x0421	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV2_CURRENT												0x0422	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV2_POWER_1												0x0423	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_PV2_POWER_2												0x0424	// 1W/bit
#define REG_INVERTER_HOME_R_PV3_VOLTAGE												0x0425	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV3_CURRENT												0x0426	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV3_POWER_1												0x0427	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_PV3_POWER_2												0x0428	// 1W/bit
#define REG_INVERTER_HOME_R_PV4_VOLTAGE												0x0429	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV4_CURRENT												0x042A	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV4_POWER_1												0x042B	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_PV4_POWER_2												0x042C	// 1W/bit
#define REG_INVERTER_HOME_R_PV5_VOLTAGE												0x042D	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV5_CURRENT												0x042E	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV5_POWER_1												0x042F	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_PV5_POWER_2												0x0430	// 1W/bit
#define REG_INVERTER_HOME_R_PV6_VOLTAGE												0x0431	// 0.1V/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV6_CURRENT												0x0432	// 0.1A/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_PV6_POWER_1												0x0433	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_PV6_POWER_2												0x0434	// 1W/bit
#define REG_INVERTER_HOME_R_INVERTER_TEMP											0x0435	// 0.1D/bit								// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_INVERTER_WARNING_1_1									0x0436	// Reserve								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_INVERTER_WARNING_1_2									0x0437	// Reserve
#define REG_INVERTER_HOME_R_INVERTER_WARNING_2_1									0x0438	// Reserve								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_INVERTER_WARNING_2_2									0x0439	// Reserve
#define REG_INVERTER_HOME_R_INVERTER_FAULT_1_1										0x043A	// Note 26								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_INVERTER_FAULT_1_2										0x043B	// Note26
#define REG_INVERTER_HOME_R_INVERTER_FAULT_2_1										0x043C	// Note 26								// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_INVERTER_FAULT_2_2										0x043D	// Note 26
#define REG_INVERTER_HOME_R_INVERTER_TOTAL_PV_ENERGY_1								0x043E	// 0.1kWh/bit							// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_INVERTER_TOTAL_PV_ENERGY_2								0x043F	// 0.1kWh/bit
#define REG_INVERTER_HOME_R_WORKING_MODE											0x0440	// <<Note5 - INVERTER OPERATION LOOKUP>>// 2 Bytes		// Unsigned Short
#ifdef EMS_35_36
#define REG_INVERTER_HOME_R_INVERTER_BAT_VOLTAGE								0x0441	// 1V/bit							// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_INVERTER_BAT_CURRENT								0x0442	// 0.1A/bit							// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_INVERTER_BAT_POWER									0x0443	// 1W/bit							// 2 Bytes		// Signed Short
#define REG_INVERTER_HOME_R_INVERTER_TOTAL_REACT_POWER_1							0x0444	// 1W/bit							// 4 Bytes		// Signed Int
//#define REG_INVERTER_HOME_R_INVERTER_TOTAL_REACT_POWER_2							0x0445	// 1W/bit
#define REG_INVERTER_HOME_R_INVERTER_TOTAL_APPARENT_POWER_1							0x0446	// 1W/bit							// 4 Bytes		// Signed Int
//#define REG_INVERTER_HOME_R_INVERTER_TOTAL_APPARENT_POWER_2							0x0447	// 1W/bit
#define REG_INVERTER_HOME_R_INVERTER_FREQUENCY									0x0448	// 0.01Hz/bit							// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_INVERTER_BACKUP_FREQUENCY								0x0449	// 0.01Hz/bit							// 2 Bytes		// Unsigned Short
#define REG_INVERTER_HOME_R_INVERTER_POWER_FACTOR								0x044A	// 0.01/bit							// 2 Bytes		// Signed Short
#define REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_1_1								0x044B	// Note 27							// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_1_2								0x044C	// Note 27
#define REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_2_1								0x044D	// Note 27							// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_2_2								0x044E	// Note 27
#define REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_3_1								0x044F	// Reserve							// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_3_2								0x0450	// Reserve
#define REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_4_1								0x0451	// Reserve							// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_4_2								0x0452	// Reserve
#endif // EMS_35_36
#define REG_INVERTER_HOME_R_PV_TOTAL_POWER_1									0x0453	// 1w/bit							// 4 Bytes		// Unsigned Integer
//#define REG_INVERTER_HOME_R_PV_TOTAL_POWER_2									0x0454	// 1w/bit

// Inverter Information
#define REG_INVERTER_INFO_R_MASTER_SOFTWARE_VERSION_1								0x0640	//										// 10 Bytes		// Unsigned Char
//#define REG_INVERTER_INFO_R_MASTER_SOFTWARE_VERSION_2								0x0641	//
//#define REG_INVERTER_INFO_R_MASTER_SOFTWARE_VERSION_3								0x0642	//
//#define REG_INVERTER_INFO_R_MASTER_SOFTWARE_VERSION_4								0x0643	//
//#define REG_INVERTER_INFO_R_MASTER_SOFTWARE_VERSION_5								0x0644	//
#define REG_INVERTER_INFO_R_SLAVE_SOFTWARE_VERSION_1								0x0645	//										// 10 Bytes		// Unsigned Char
//#define REG_INVERTER_INFO_R_SLAVE_SOFTWARE_VERSION_2								0x0646	//
//#define REG_INVERTER_INFO_R_SLAVE_SOFTWARE_VERSION_3								0x0647	//
//#define REG_INVERTER_INFO_R_SLAVE_SOFTWARE_VERSION_4								0x0648	//
//#define REG_INVERTER_INFO_R_SLAVE_SOFTWARE_VERSION_5								0x0649	//
#define REG_INVERTER_INFO_R_SERIAL_NUMBER_1											0x064A	//										// 20 Bytes		// Unsigned Char
//#define REG_INVERTER_INFO_R_SERIAL_NUMBER_2											0x064B	//
//#define REG_INVERTER_INFO_R_SERIAL_NUMBER_3											0x064C	//
//#define REG_INVERTER_INFO_R_SERIAL_NUMBER_4											0x064D	//
//#define REG_INVERTER_INFO_R_SERIAL_NUMBER_5											0x064E	//
//#define REG_INVERTER_INFO_R_SERIAL_NUMBER_6											0x065F	//
//#define REG_INVERTER_INFO_R_SERIAL_NUMBER_7											0x0650	//
//#define REG_INVERTER_INFO_R_SERIAL_NUMBER_8											0x0651	//
//#define REG_INVERTER_INFO_R_SERIAL_NUMBER_9											0x0652	//
//#define REG_INVERTER_INFO_R_SERIAL_NUMBER_10										0x0653	//
#define REG_INVERTER_INFO_R_ARM_SOFTWARE_VERSION_1										0x0654	//										// 10 Bytes		// Unsigned Char
//#define REG_INVERTER_INFO_R_ARM_SOFTWARE_VERSION_2										0x0655	//
//#define REG_INVERTER_INFO_R_ARM_SOFTWARE_VERSION_3										0x0656	//
//#define REG_INVERTER_INFO_R_ARM_SOFTWARE_VERSION_4										0x0657	//
//#define REG_INVERTER_INFO_R_ARM_SOFTWARE_VERSION_5										0x0658	//


// ***********
// Ignore System(Only applicable to HHE MEC)

// ***********
// Ignore Echonet Config (Japan)


// System Information
#define REG_SYSTEM_INFO_RW_FEED_INTO_GRID_PERCENT									0x0700	// 1%/bit						// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_INFO_R_SYSTEM_FAULT											0x0701	// Note 6						// 4 Bytes		// Unsigned Integer
#define REG_SYSTEM_INFO_RW_SYSTEM_TIME_YEAR_MONTH									0x0740	// 0xYYMM, 0x1109 = 2017/09				// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_INFO_RW_SYSTEM_TIME_DAY_HOUR										0x0741	// 0xDDHH, 0x1109 = 17th Day/9th Hour	// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_INFO_RW_SYSTEM_TIME_MINUTE_SECOND								0x0742	// 0xmmss, 0x1109 = 17th Min/9th Sec	// 2 Bytes		// Unsigned Short

#define REG_SYSTEM_INFO_R_EMS_SN_BYTE_1_2											0x0743	// EMS SN: ASCII 0x414C=='AL'			// 2 Bytes		// Unsigned Short
//#define REG_SYSTEM_INFO_R_EMS_SN_BYTE_3_4											0x0744	// EMS SN: ASCII 0x3132=='12'			// 2 Bytes		// Unsigned Short
//#define REG_SYSTEM_INFO_R_EMS_SN_BYTE_5_6											0x0745	// EMS SN: ASCII 0x3132=='12'			// 2 Bytes		// Unsigned Short
//#define REG_SYSTEM_INFO_R_EMS_SN_BYTE_7_8											0x0746	// EMS SN: ASCII 0x3132=='12'			// 2 Bytes		// Unsigned Short
//#define REG_SYSTEM_INFO_R_EMS_SN_BYTE_9_10											0x0747	// EMS SN: ASCII 0x3132=='12'			// 2 Bytes		// Unsigned Short
//#define REG_SYSTEM_INFO_R_EMS_SN_BYTE_11_12											0x0748	// EMS SN: ASCII 0x3132=='12'			// 2 Bytes		// Unsigned Short
//#define REG_SYSTEM_INFO_R_EMS_SN_BYTE_13_14											0x0749	// EMS SN: ASCII 0x3132=='12'			// 2 Bytes		// Unsigned Short
//#define REG_SYSTEM_INFO_R_EMS_SN_BYTE_15_16											0x074A	// EMS SN: ASCII 0x3132=='12'			// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_INFO_R_EMS_VERSION_HIGH											0x074B	// 										// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_INFO_R_EMS_VERSION_MIDDLE										0x074C	// 										// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_INFO_R_EMS_VERSION_LOW											0x074D	// 										// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_INFO_R_PROTOCOL_VERSION											0x074E	// 										// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_INFO_R_EMS_VERSION_LOW_SUFFIX_1										0x074F	// 										// 8 Bytes		// String
//#define REG_SYSTEM_INFO_R_EMS_VERSION_LOW_SUFFIX_2										0x0750
//#define REG_SYSTEM_INFO_R_EMS_VERSION_LOW_SUFFIX_3										0x0751
//#define REG_SYSTEM_INFO_R_EMS_VERSION_LOW_SUFFIX_4										0x0752


// System Configuration
#define REG_SYSTEM_CONFIG_RW_MAX_FEED_INTO_GRID_PERCENT								0x0800	// 1%/bit								// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_CONFIG_RW_PV_CAPACITY_STORAGE_1									0x0801	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_SYSTEM_CONFIG_RW_PV_CAPACITY_STORAGE_2									0x0802	// 1W/bit
#define REG_SYSTEM_CONFIG_RW_PV_CAPACITY_OF_GRID_INVERTER_1							0x0803	// 1W/bit								// 4 Bytes		// Unsigned Integer
//#define REG_SYSTEM_CONFIG_RW_PV_CAPACITY_OF_GRID_INVERTER_2						0x0804	// 1W/bit
#define REG_SYSTEM_CONFIG_RW_SYSTEM_MODE											0x0805	// <<SYSTEM MODE LOOKUP>>				// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_CONFIG_RW_METER_CT_SELECT										0x0806	// <<METER CT SELECT LOOKUP>>			// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_CONFIG_RW_BATTERY_READY											0x0807	// <<BATTERY READY LOOKUP>>				// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_CONFIG_RW_IP_METHOD												0x0808	// <<IP METHOD LOOKUP>>					// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_CONFIG_RW_LOCAL_IP_1												0x0809	// 0xC0A80101 192.168.1.1				// 4 Bytes		// Unsigned Int ** Declared short, believe Int
//#define REG_SYSTEM_CONFIG_RW_LOCAL_IP_2												0x080A	// 0xC0A80101 192.168.1.1
#define REG_SYSTEM_CONFIG_RW_SUBNET_MASK_1											0x080B	// 0xFFFFFF01 255.255.255.0				// 4 Bytes		// Unsigned Int ** Declared short, believe Int
//#define REG_SYSTEM_CONFIG_RW_SUBNET_MASK_2											0x080C	// 0xFFFFFF01 255.255.255.0
#define REG_SYSTEM_CONFIG_RW_GATEWAY_1												0x080D	// 0xC0A80101 192.168.1.1				// 4 Bytes		// Unsigned Int ** Declared short, believe Int
//#define REG_SYSTEM_CONFIG_RW_GATEWAY_2												0x080E	// 0xC0A80101 192.168.1.1
#define REG_SYSTEM_CONFIG_RW_MODBUS_ADDRESS											0x080F	// default 0x55							// 2 Bytes		// Unsigned Short
#define REG_SYSTEM_CONFIG_RW_MODBUS_BAUD_RATE										0x0810	// <<MODBUS BAUD RATE LOOKUP>>			// 2 Bytes		// Unsigned Short


// Timing
#define REG_TIMING_RW_TIME_PERIOD_CONTROL_FLAG										0x084F	// <<TIME PERIOD CONTROL FLAG LOOKUP>>	// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_UPS_RESERVE_SOC												0x0850	// 0.1/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_DISCHARGE_START_TIME_1									0x0851	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_1									0x0852	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_DISCHARGE_START_TIME_2									0x0853	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_2									0x0854	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_CHARGE_CUT_SOC												0x0855	// 0.1/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_CHARGE_START_TIME_1										0x0856	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_CHARGE_STOP_TIME_1										0x0857	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_CHARGE_START_TIME_2										0x0858	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_CHARGE_STOP_TIME_2										0x0859	// 1H/bit								// 2 Bytes		// Unsigned Short
#ifdef EMS_35_36
#define REG_TIMING_RW_TIME_DISCHARGE_START_TIME_1_MIN									0x085A	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_1_MIN									0x085B	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_DISCHARGE_START_TIME_2_MIN									0x085C	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_2_MIN									0x085D	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_CHARGE_START_TIME_1_MIN									0x085E	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_CHARGE_STOP_TIME_1_MIN									0x085F	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_CHARGE_START_TIME_2_MIN									0x0860	// 1H/bit								// 2 Bytes		// Unsigned Short
#define REG_TIMING_RW_TIME_CHARGE_STOP_TIME_2_MIN									0x0861	// 1H/bit								// 2 Bytes		// Unsigned Short
#endif // EMS_35_36

// Dispatch
#define REG_DISPATCH_RW_DISPATCH_START												0x0880	// <<DISPATCH START LOOKUP>>			// 2 Bytes		// Unsigned Short
#define REG_DISPATCH_RW_ACTIVE_POWER_1												0x0881	// 1W/bit Offset: 32000 load<32000		// 4 Bytes		// Int
//#define REG_DISPATCH_RW_ACTIVE_POWER_2												0x0882	// 1W/bit Offset: 32000 load<32000	
#define REG_DISPATCH_RW_REACTIVE_POWER_1											0x0883	// 1Var/bit Offset: 32000 load<32000	// 4 Bytes		// Int
//#define REG_DISPATCH_RW_REACTIVE_POWER_2											0x0884	// 1Var/bit Offset: 32000 load<32000	
#define REG_DISPATCH_RW_DISPATCH_MODE												0x0885	// <<Note7 - DISPATCH MODE LOOKUP>>		// 2 Bytes		// Unsigned Short
#define REG_DISPATCH_RW_DISPATCH_SOC												0x0886	// 0.4%/bit, 95=SOC of 38%				// 2 Bytes		// Unsigned Short
#define REG_DISPATCH_RW_DISPATCH_TIME_1												0x0887	// 1S/bit								// 4 Bytes		// Unsigned Int
//#define REG_DISPATCH_RW_DISPATCH_TIME_2												0x0888	// 1S/bit
#define REG_DISPATCH_RW_DISPATCH_PARA_7												0x0889	// <<Note29>>		// 2 Bytes		// Unsigned Short
#define REG_DISPATCH_RW_DISPATCH_PARA_8												0x088A	// <<Note29>>		// 2 Bytes		// Unsigned Short

// Auxiliary
#define REG_AUXILIARY_W_EMS_DO0														0x08B0	// by-pass control function				// 2 Bytes		// Unsigned Short
#define REG_AUXILIARY_W_EMS_DO1														0x08B1	// System fault output					// 2 Bytes		// Unsigned Short
#define REG_AUXILIARY_R_EMS_DI0														0x08C0	// EPO, BatteryMOS cut off				// 2 Bytes		// Unsigned Short
#define REG_AUXILIARY_R_EMS_DI1														0x08C1	// Reserved								// 2 Bytes		// Unsigned Short

// System - Operational Data
#define REG_SYSTEM_OP_R_PV_INVERTER_ENERGY_1										0x08D0	// 0.1kWh/bit							// 4 Bytes		// Unsigned Int
//#define REG_SYSTEM_OP_R_PV_INVERTER_ENERGY_2										0x08D1	// 0.1kWh/bit
#define REG_SYSTEM_OP_R_SYSTEM_TOTAL_PV_ENERGY_1									0x08D2	// 0.1kWh/bit							// 4 Bytes		// Unsigned Int
//#define REG_SYSTEM_OP_R_SYSTEM_TOTAL_PV_ENERGY_2									0x08D3	// 0.1kWh/bit
#define REG_SYSTEM_OP_R_SYSTEM_FAULT_1												0x08D4	// <<Note6 - SYSTEM ERROR LOOKUP>>		// 4 Bytes		// Unsigned Int
//#define REG_SYSTEM_OP_R_SYSTEM_FAULT_2												0x08D5	// <<Note6 - SYSTEM ERROR LOOKUP>>


// Safety TEST
#define REG_SAFETY_TEST_RW_GRID_REGULATION											0x1000	// <<GRID REGULATION LOOKUP>>			// 2 Bytes		// Unsigned Short


// Custom in Alpha2ESS
#define REG_CUSTOM_LOAD																0xFFFE	// // 1W/bit							// 4 Bytes		// Integer
#define REG_CUSTOM_SYSTEM_DATE_TIME													0xFFFF	// dd/MMM/yyyy HH:mm:ss					// N/A			// Unsigned Char
#define REG_CUSTOM_GRID_CURRENT_A_PHASE												0xFFFD	// 0.1A									// 2 Bytes		// Short
#define REG_CUSTOM_TOTAL_SOLAR_POWER												0xFFFC
// End of Handled Registered







// BATTERY MOS CONTROL LOOKUP
#define BATTERY_MOS_CONTROL_OPEN 0
#define BATTERY_MOS_CONTROL_OPEN_DESC "Open"
#define BATTERY_MOS_CONTROL_CLOSE 1
#define BATTERY_MOS_CONTROL_CLOSE_DESC "Close"

// BATTERY SOC CALIBRATION LOOKUP
#define BATTERY_SOC_CALIBRATION_DISABLE 0
#define BATTERY_SOC_CALIBRATION_DISABLE_DESC "Disable"
#define BATTERY_SOC_CALIBRATION_ENABLE 1
#define BATTERY_SOC_CALIBRATION_ENABLE_DESC "Enable"


// SYSTEM MODE LOOKUP
#define SYSTEM_MODE_AC 1
#define SYSTEM_MODE_AC_DESC "AC mode"
#define SYSTEM_MODE_DC 2
#define SYSTEM_MODE_DC_DESC "DC mode"
#define SYSTEM_MODE_HYBRID 3
#define SYSTEM_MODE_HYBRID_DESC "Hybrid mode"

// METER CT SELECT LOOKUP
#define METER_CT_SELECT_GRID_AND_PV_USE_CT 0
#define METER_CT_SELECT_GRID_AND_PV_USE_CT_DESC "Grid&PV use CT"
#define METER_CT_SELECT_GRID_USE_CT_PV_USE_METER 1
#define METER_CT_SELECT_GRID_USE_CT_PV_USE_METER_DESC "Grid use CT, PV use Meter"
#define METER_CT_SELECT_GRID_USE_METER_PV_USE_CT 2
#define METER_CT_SELECT_GRID_USE_METER_PV_USE_CT_DESC "Grid use meter, PV use CT"
#define METER_CT_SELECT_GRID_AND_PV_USE_METER 3
#define METER_CT_SELECT_GRID_AND_PV_USE_METER_DESC "Grid&PV use meter"


// BATTERY READY LOOKUP
#define BATTERY_READY_OFF 0
#define BATTERY_READY_OFF_DESC "Off"
#define BATTERY_READY_ON 1
#define BATTERY_READY_ON_DESC "On"

// IP METHOD LOOKUP
#define IP_METHOD_DHCP 0
#define IP_METHOD_DHCP_DESC "DHCP"
#define IP_METHOD_STATIC 1
#define IP_METHOD_STATIC_DESC "STATIC"

// MODBUS BAUD RATE LOOKUP
#define MODBUS_BAUD_RATE_9600 0
#define MODBUS_BAUD_RATE_9600_DESC "9600"
#define MODBUS_BAUD_RATE_115200 1
#define MODBUS_BAUD_RATE_115200_DESC "115200 (Only household)"
#define MODBUS_BAUD_RATE_256000 2
#define MODBUS_BAUD_RATE_256000_DESC "256000 (Only household)"
#define MODBUS_BAUD_RATE_19200 3
#define MODBUS_BAUD_RATE_19200_DESC "19200 (Only industry)"

// TIME PERIOD CONTROL FLAG LOOKUP
#define TIME_PERIOD_CONTROL_FLAG_DISABLE 0
#define TIME_PERIOD_CONTROL_FLAG_DISABLE_DESC "Disable Time period control"
#define TIME_PERIOD_CONTROL_FLAG_ENABLE_CHARGE 1
#define TIME_PERIOD_CONTROL_FLAG_ENABLE_CHARGE_DESC "Enable charge Time period control"
#define TIME_PERIOD_CONTROL_FLAG_ENABLE_DISCHARGE 2
#define TIME_PERIOD_CONTROL_FLAG_ENABLE_DISCHARGE_DESC "Enable discharge Time period control"
#define TIME_PERIOD_CONTROL_FLAG_ENABLE 3
#define TIME_PERIOD_CONTROL_FLAG_ENABLE_DESC "Enable Time period control"


// DISPATCH START LOOKUP
#define DISPATCH_START_START 1
#define DISPATCH_START_START_DESC "Start"
#define DISPATCH_START_STOP 0
#define DISPATCH_START_STOP_DESC "Stop"

#define DISPATCH_ACTIVE_POWER_OFFSET 32000


// Note1 - BATTERY STATUS LOOKUP
#define BATTERY_STATUS_CHARGE0_DISCHARGE0 0
#define BATTERY_STATUS_CHARGE0_DISCHARGE0_DESC "Charge (0) / Discharge (0)"
#define BATTERY_STATUS_CHARGE0_DISCHARGE1 1
#define BATTERY_STATUS_CHARGE0_DISCHARGE1_DESC "Charge (0) / Discharge (1)"
#define BATTERY_STATUS_CHARGE1_DISCHARGE0 256
#define BATTERY_STATUS_CHARGE1_DISCHARGE0_DESC "Charge (1) / Discharge (0)"
#define BATTERY_STATUS_CHARGE1_DISCHARGE1 257
#define BATTERY_STATUS_CHARGE1_DISCHARGE1_DESC "Charge (1) / Discharge (1)"
#define BATTERY_STATUS_CHARGE2_DISCHARGE0 512
#define BATTERY_STATUS_CHARGE2_DISCHARGE0_DESC "Charge (2) / Discharge (0)"
#define BATTERY_STATUS_CHARGE2_DISCHARGE1 513
#define BATTERY_STATUS_CHARGE2_DISCHARGE1_DESC "Charge (2) / Discharge (1)"


// Note2 - BATTERY RELAY STATUS LOOKUP
#define BATTERY_RELAY_STATUS_CHARGE_DISCHARGE_RELAYS_NOT_CONNECTED 0
#define BATTERY_RELAY_STATUS_CHARGE_DISCHARGE_RELAYS_NOT_CONNECTED_DESC "Charge and discharge relays are disconnected"
#define BATTERY_RELAY_STATUS_ONLY_DISCHARGE_RELAY_CLOSED 1
#define BATTERY_RELAY_STATUS_ONLY_DISCHARGE_RELAY_CLOSED_DESC "Only the discharge relay is closed"
#define BATTERY_RELAY_STATUS_ONLY_CHARGE_RELAY_CLOSED 2
#define BATTERY_RELAY_STATUS_ONLY_CHARGE_RELAY_CLOSED_DESC "Only the charging relay is closed"
#define BATTERY_RELAY_STATUS_CHARGE_AND_DISCHARGE_RELAYS_CLOSED 3
#define BATTERY_RELAY_STATUS_CHARGE_AND_DISCHARGE_RELAYS_CLOSED_DESC "Charge and discharge relays are closed"


// Note3 - BATTERY TYPE LOOKUP
#define BATTERY_TYPE_M4860 2
#define BATTERY_TYPE_M4860_DESC "M4860"
#define BATTERY_TYPE_M48100 3
#define BATTERY_TYPE_M48100_DESC "M48100"
#define BATTERY_TYPE_48112_P 13
#define BATTERY_TYPE_48112_P_DESC "48112-P"
#define BATTERY_TYPE_SMILE5_BAT 16
#define BATTERY_TYPE_SMILE5_BAT_DESC "Smile5-BAT"
#define BATTERY_TYPE_M4856_P 24
#define BATTERY_TYPE_M4856_P_DESC "M4856-P"
#define BATTERY_TYPE_SMILE_BAT_10_3P 27
#define BATTERY_TYPE_SMILE_BAT_10_3P_DESC "Smile-BAT-10.3P"
#define BATTERY_TYPE_SMILE_BAT_10_1P 30
#define BATTERY_TYPE_SMILE_BAT_10_1P_DESC "Smile-BAT-10.1P"
#define BATTERY_TYPE_SMILE_BAT_5_8P 33
#define BATTERY_TYPE_SMILE_BAT_5_8P_DESC "Smile-BAT-5.8P"
#define BATTERY_TYPE_SMILE_BAT_5_JP 34
#define BATTERY_TYPE_SMILE_BAT_5_JP_DESC "Smile-BAT-JP"
#define BATTERY_TYPE_SMILE_BAT_13_7P 35
#define BATTERY_TYPE_SMILE_BAT_13_7P_DESC "Smile-BAT-13.7P"
#define BATTERY_TYPE_SMILE_BAT_8_2_PHA 77
#define BATTERY_TYPE_SMILE_BAT_8_2_PHA_DESC "Smile-BAT-8.2PHA"

// Note4 - BATTERY ERROR LOOKUP
#ifdef EMS_35_36
#define BATTERY_ERROR_BIT_0 "Temp sensor error"
#define BATTERY_ERROR_BIT_1 "Mos error"
#define BATTERY_ERROR_BIT_2 "Circuit breaker open"
#define BATTERY_ERROR_BIT_3 "Dial switching mode inconsistence"
#define BATTERY_ERROR_BIT_4 "Slave battery communication lost"
#define BATTERY_ERROR_BIT_5 "Sn missing"
#define BATTERY_ERROR_BIT_6 "Master battery communication lost"
#define BATTERY_ERROR_BIT_7 "Firmware versions inconsistence"
#define BATTERY_ERROR_BIT_8 "Multi master error"
#define BATTERY_ERROR_BIT_9 "Mos high temperature"
#define BATTERY_ERROR_BIT_10 "Insulation fault"
#define BATTERY_ERROR_BIT_11 "Total pressureabnormal"
#define BATTERY_ERROR_BIT_12 "Mos feedback failure"
#define BATTERY_ERROR_BIT_13 "Prefi lled failure"
#define BATTERY_ERROR_BIT_14 "17823 communication failure"
#define BATTERY_ERROR_BIT_15 "17841 communication failure"
#define BATTERY_ERROR_BIT_16 "Mos temperature sensor error"
#define BATTERY_ERROR_BIT_17 "UNDEFINED"
#define BATTERY_ERROR_BIT_18 "UNDEFINED"
#define BATTERY_ERROR_BIT_19 "UNDEFINED"
#define BATTERY_ERROR_BIT_20 "UNDEFINED"
#define BATTERY_ERROR_BIT_21 "UNDEFINED"
#define BATTERY_ERROR_BIT_22 "UNDEFINED"
#define BATTERY_ERROR_BIT_23 "UNDEFINED"
#define BATTERY_ERROR_BIT_24 "UNDEFINED"
#define BATTERY_ERROR_BIT_25 "UNDEFINED"
#define BATTERY_ERROR_BIT_26 "UNDEFINED"
#define BATTERY_ERROR_BIT_27 "UNDEFINED"
#define BATTERY_ERROR_BIT_28 "UNDEFINED"
#define BATTERY_ERROR_BIT_29 "UNDEFINED"
#define BATTERY_ERROR_BIT_30 "UNDEFINED"
#define BATTERY_ERROR_BIT_31 "UNDEFINED"
#else // EMS_35_36
#define BATTERY_ERROR_BIT_0 ""
#define BATTERY_ERROR_BIT_1 ""
#define BATTERY_ERROR_BIT_2 "Cell Temp Difference"
#define BATTERY_ERROR_BIT_3 "Balancer Fault"
#define BATTERY_ERROR_BIT_4 "Charge Over Current"
#define BATTERY_ERROR_BIT_5 "Balancer Mos Fault"
#define BATTERY_ERROR_BIT_6 "Discharge Over Current"
#define BATTERY_ERROR_BIT_7 "Pole Over Temp"
#define BATTERY_ERROR_BIT_8 "Cell Over Volts"
#define BATTERY_ERROR_BIT_9 "Cell Volt Difference"
#define BATTERY_ERROR_BIT_10 "Discharge Low Temp"
#define BATTERY_ERROR_BIT_11 "Low Volt Shutdown"
#define BATTERY_ERROR_BIT_12 "Cell Low Volts"
#define BATTERY_ERROR_BIT_13 "ISO Comm Fault"
#define BATTERY_ERROR_BIT_14 "LMU SN Repeat"
#define BATTERY_ERROR_BIT_15 "BMU SN Repeat"
#define BATTERY_ERROR_BIT_16 "IR Fault"
#define BATTERY_ERROR_BIT_17 "LMU Comm Fault"
#define BATTERY_ERROR_BIT_18 "Cell Over Temp"
#define BATTERY_ERROR_BIT_19 "BMU Comm Fault"
#define BATTERY_ERROR_BIT_20 "INV Comm Fault"
#define BATTERY_ERROR_BIT_21 "Charge Low Temp"
#define BATTERY_ERROR_BIT_22 "TOPBMU Comm Fault"
#define BATTERY_ERROR_BIT_23 "Volt Detect Fault"
#define BATTERY_ERROR_BIT_24 "Wire Harness Fault"
#define BATTERY_ERROR_BIT_25 "Cluster Cut Fault"
#define BATTERY_ERROR_BIT_26 "Relay Fault"
#define BATTERY_ERROR_BIT_27 "LMU ID Repeat"
#define BATTERY_ERROR_BIT_28 "LMU ID Discontinuous"
#define BATTERY_ERROR_BIT_29 "Current Sensor Fault"
#define BATTERY_ERROR_BIT_30 ""
#define BATTERY_ERROR_BIT_31 "Temp Sensor Fault"
#endif // EMS_35_36

// Note5 - INVERTER OPERATION LOOKUP
#define INVERTER_OPERATION_MODE_WAIT_MODE 0
#define INVERTER_OPERATION_MODE_WAIT_MODE_DESC "Wait Mode"
#define INVERTER_OPERATION_MODE_ONLINE_MODE 1
#define INVERTER_OPERATION_MODE_ONLINE_MODE_DESC "Online Mode"
#define INVERTER_OPERATION_MODE_UPS_MODE 2
#define INVERTER_OPERATION_MODE_UPS_MODE_DESC "UPS Mode"
#define INVERTER_OPERATION_MODE_BYPASS_MODE 3
#define INVERTER_OPERATION_MODE_BYPASS_MODE_DESC "Bypass Mode"
#define INVERTER_OPERATION_MODE_ERROR_MODE 4
#define INVERTER_OPERATION_MODE_ERROR_MODE_DESC "Error Mode"
#define INVERTER_OPERATION_MODE_DC_MODE 5
#define INVERTER_OPERATION_MODE_DC_MODE_DESC "DC Mode"
#define INVERTER_OPERATION_MODE_SELF_TEST_MODE 6
#define INVERTER_OPERATION_MODE_SELF_TEST_MODE_DESC "Self Test Mode"
#define INVERTER_OPERATION_MODE_CHECK_MODE 7
#define INVERTER_OPERATION_MODE_CHECK_MODE_DESC "Check Mode"
#define INVERTER_OPERATION_MODE_UPDATE_MASTER_MODE 8
#define INVERTER_OPERATION_MODE_UPDATE_MASTER_MODE_DESC "Update Master Mode"
#define INVERTER_OPERATION_MODE_UPDATE_SLAVE_MODE 9
#define INVERTER_OPERATION_MODE_UPDATE_SLAVE_MODE_DESC "Update Slave Mode"
#define INVERTER_OPERATION_MODE_UPDATE_ARM_MODE 10
#define INVERTER_OPERATION_MODE_UPDATE_ARM_MODE_DESC "Update ARM Mode"



// Note6 - SYSTEM ERROR LOOKUP
#define SYSTEM_ERROR_AL_BIT_0 "Network Card_Fault"
#define SYSTEM_ERROR_AL_BIT_1 "Rtc_Fault"
#define SYSTEM_ERROR_AL_BIT_2 "E2prom_Fault"
#define SYSTEM_ERROR_AL_BIT_3 "INV_Comms_Error"
#define SYSTEM_ERROR_AL_BIT_4 "Grid_Meter_Lost"
#define SYSTEM_ERROR_AL_BIT_5 "PV_Meter_Lost"
#define SYSTEM_ERROR_AL_BIT_6 "BMS_Lost"
#define SYSTEM_ERROR_AL_BIT_7 "UPS_Battery_Volt_Low"
#define SYSTEM_ERROR_AL_BIT_8 "Backup_Overload"
#define SYSTEM_ERROR_AL_BIT_9 "INV_Slave_Lost"
#define SYSTEM_ERROR_AL_BIT_10 "INV_Master_Lost"
#define SYSTEM_ERROR_AL_BIT_11 "Parallel_Comm_Error"
#define SYSTEM_ERROR_AL_BIT_12 "Parallel_Mode_Differ"
#define SYSTEM_ERROR_AL_BIT_13 "Flash_Fault"
#define SYSTEM_ERROR_AL_BIT_14 "SDRAM error"
#define SYSTEM_ERROR_AL_BIT_15 "Extension CAN error"
#define SYSTEM_ERROR_AL_BIT_16 "inv type not specified"


// Note6 - SYSTEM ERROR LOOKUP
#define SYSTEM_ERROR_AE_BIT_0 "Inverter disconnected"
#define SYSTEM_ERROR_AE_BIT_1 "Net meter separately"
#define SYSTEM_ERROR_AE_BIT_2 "Battery disconnected"
#define SYSTEM_ERROR_AE_BIT_3 "System not set"
#define SYSTEM_ERROR_AE_BIT_4 "PV meter disconnected"
#define SYSTEM_ERROR_AE_BIT_5 "Counter not set"
#define SYSTEM_ERROR_AE_BIT_6 "Incorrect connection direction of the PV meter"
#define SYSTEM_ERROR_AE_BIT_7 "SD not inserted or SD write error"
#define SYSTEM_ERROR_AE_BIT_8 "RTC error"
#define SYSTEM_ERROR_AE_BIT_9 "SDRAM error"
#define SYSTEM_ERROR_AE_BIT_10 "MMC-Error (CH376)"
#define SYSTEM_ERROR_AE_BIT_11 "Network card error"
#define SYSTEM_ERROR_AE_BIT_12 "Extension CAN Error (MCP2515)"
#define SYSTEM_ERROR_AE_BIT_13 "DRED error"
#define SYSTEM_ERROR_AE_BIT_14 "Android LCD separated"
#define SYSTEM_ERROR_AE_BIT_15 "STS_Lost"
#define SYSTEM_ERROR_AE_BIT_16 "STS_Fault"
#define SYSTEM_ERROR_AE_BIT_17 "PV_INV_Lost:n"
#define SYSTEM_ERROR_AE_BIT_18 "DG_PV_Conflict"
#define SYSTEM_ERROR_AE_BIT_19 "PV_INV_Fault:n"
#define SYSTEM_ERROR_AE_BIT_20 "AirConFault"
#define SYSTEM_ERROR_AE_BIT_21 "Fire_Fault"
#define SYSTEM_ERROR_AE_BIT_22 "FireControllerErr"
#define SYSTEM_ERROR_AE_BIT_23 "GC_Fault"
#define SYSTEM_ERROR_AE_BIT_24 "AirConLost"
#define SYSTEM_ERROR_AE_BIT_25 "OverCurr"
#define SYSTEM_ERROR_AE_BIT_26 "PcsModeFault"
#define SYSTEM_ERROR_AE_BIT_27 "BatEnergyLow"



// Note7 - DISPATCH MODE LOOKUP
#define DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV 1
#define DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV_DESC "Only charge via PV"
// Battery is not allowed to discharge (Pdispatch < 32000).
// After the PV supplies the load, the excess energy is used to charge the
// battery. When the battery is charged, there is surplus power to the grid.
// Honors Dispatch Power but not Dispatch SOC
#define DISPATCH_MODE_STATE_OF_CHARGE_CONTROL 2
#define DISPATCH_MODE_STATE_OF_CHARGE_CONTROL_DESC "SOC control"
// Force charge mode. The charge or discharge process will be
// stopped when it reaches the SOC setting value.
// Honors Dispatch Power and Dispatch SOC
#define DISPATCH_MODE_LOAD_FOLLOWING 3
#define DISPATCH_MODE_LOAD_FOLLOWING_DESC "Load Following"
// The system will be self-consumption mode. Same as mode 5.
// Honors Dispatch Power but not Dispatch SOC
#define DISPATCH_MODE_MAXIMISE_OUTPUT 4
#define DISPATCH_MODE_MAXIMISE_OUTPUT_DESC "Maximize Output"
// If the current PV power can not meet the required inverter AC
// output power, the battery will also discharge.
// Doesn't honors Dispatch Power or Dispatch SOC
#define DISPATCH_MODE_NORMAL_MODE 5
#define DISPATCH_MODE_NORMAL_MODE_DESC "Normal mode"
// The system will be self-consumption mode. Same as mode 3.
// Honors Dispatch Power but not Dispatch SOC
#define DISPATCH_MODE_OPTIMISE_CONSUMPTION 6
#define DISPATCH_MODE_OPTIMISE_CONSUMPTION_DESC "Optimise consumption"
// PV will charge batteries first. If the PV power cannot meet the maximum battery
// charging power, it will also absorb power from the grid to charge the battery.
// Doesn't honors Dispatch Power or Dispatch SOC
#define DISPATCH_MODE_MAXIMISE_CONSUMPTION 7
#define DISPATCH_MODE_MAXIMISE_CONSUMPTION_DESC "Maximise consumption"
// It will only absorb power from the grid to charge the battery.
// Doesn't honors Dispatch Power or Dispatch SOC
#define DISPATCH_MODE_ECO_MODE 8
#define DISPATCH_MODE_ECO_MODE_DESC "ECO Mode"
// Self-consumption mode, but limits charging power.
// Honors Dispatch Power (for charging) but not Dispatch SOC
#define DISPATCH_MODE_FCAS_MODE 9
#define DISPATCH_MODE_FCAS_MODE_DESC "FCAS-Mode"
// Unknown
#define DISPATCH_MODE_PV_POWER_SETTING 10
#define DISPATCH_MODE_PV_POWER_SETTING_DESC "PV power setting"
// Unknown
#define DISPATCH_MODE_NO_BATTERY_CHARGE 19
#define DISPATCH_MODE_NO_BATTERY_CHARGE_DESC "No Battery Charge"
// Battery doesn't charge.  Excess PV power is pushed to the grid.
// Honors Dispatch Power but not Dispatch SOC
#define DISPATCH_MODE_BURNIN_MODE 20
#define DISPATCH_MODE_BURNIN_MODE_DESC "Burnin Mode"
// Unknown


// Note8 - GRID REGULATION LOOKUP
#define GRID_REGULATION_AL_0 0
#define GRID_REGULATION_AL_0_DESC "VDE0126"
#define GRID_REGULATION_AL_1 1
#define GRID_REGULATION_AL_1_DESC "ARN4105/11.18"
#define GRID_REGULATION_AL_2 2
#define GRID_REGULATION_AL_2_DESC "AS4777.2"
#define GRID_REGULATION_AL_3 3
#define GRID_REGULATION_AL_3_DESC "G83_2"
#define GRID_REGULATION_AL_4 4
#define GRID_REGULATION_AL_4_DESC "C10/C11"
#define GRID_REGULATION_AL_5 5
#define GRID_REGULATION_AL_5_DESC "TOR D4"
#define GRID_REGULATION_AL_6 6
#define GRID_REGULATION_AL_6_DESC "EN50438_NL"
#define GRID_REGULATION_AL_7 7
#define GRID_REGULATION_AL_7_DESC "EN50438_DK"
#define GRID_REGULATION_AL_8 8
#define GRID_REGULATION_AL_8_DESC "CEB"
#define GRID_REGULATION_AL_9 9
#define GRID_REGULATION_AL_9_DESC "CEI-021"
#define GRID_REGULATION_AL_10 10
#define GRID_REGULATION_AL_10_DESC "NRS097-2-1"
#define GRID_REGULATION_AL_11 11
#define GRID_REGULATION_AL_11_DESC "VDE0126_GREECE"
#define GRID_REGULATION_AL_12 12
#define GRID_REGULATION_AL_12_DESC "UTE_C15_712"
#define GRID_REGULATION_AL_13 13
#define GRID_REGULATION_AL_13_DESC "IEC61727"
#define GRID_REGULATION_AL_14 14
#define GRID_REGULATION_AL_14_DESC "G59_3"
#define GRID_REGULATION_AL_15 15
#define GRID_REGULATION_AL_15_DESC "RD1699"
#define GRID_REGULATION_AL_16 16
#define GRID_REGULATION_AL_16_DESC "G99"
#define GRID_REGULATION_AL_17 17
#define GRID_REGULATION_AL_17_DESC "Philippines_60HZ"
#define GRID_REGULATION_AL_18 18
#define GRID_REGULATION_AL_18_DESC "Tahiti_60HZ"
#define GRID_REGULATION_AL_19 19
#define GRID_REGULATION_AL_19_DESC "AS4777.2-SA"
#define GRID_REGULATION_AL_20 20
#define GRID_REGULATION_AL_20_DESC "G98"
#define GRID_REGULATION_AL_21 21
#define GRID_REGULATION_AL_21_DESC "EN50549"
#define GRID_REGULATION_AL_22 22
#define GRID_REGULATION_AL_22_DESC "PEA"
#define GRID_REGULATION_AL_23 23
#define GRID_REGULATION_AL_23_DESC "MEA"
#define GRID_REGULATION_AL_24 24
#define GRID_REGULATION_AL_24_DESC "BISI"
#define GRID_REGULATION_AL_25 25
#define GRID_REGULATION_AL_25_DESC "JET-GR Series"
#define GRID_REGULATION_AL_26 26
#define GRID_REGULATION_AL_26_DESC "JET-GR Series"
#define GRID_REGULATION_AL_27 27
#define GRID_REGULATION_AL_27_DESC "Taiwan"
#define GRID_REGULATION_AL_28 28
#define GRID_REGULATION_AL_28_DESC "DEFAULT_50HZ"
#define GRID_REGULATION_AL_29 29
#define GRID_REGULATION_AL_29_DESC "DEFAULT_60HZ"
#define GRID_REGULATION_AL_30 30
#define GRID_REGULATION_AL_30_DESC "WAREHOUSE"
#define GRID_REGULATION_AL_31 31
#define GRID_REGULATION_AL_31_DESC "AS4777.2-NZ"
#define GRID_REGULATION_AL_32 32
#define GRID_REGULATION_AL_32_DESC "Korea"
#define GRID_REGULATION_AL_33 33
#define GRID_REGULATION_AL_33_DESC "G98/G99-IE"
#define GRID_REGULATION_AL_34 34
#define GRID_REGULATION_AL_34_DESC "34EN50549-PL"
#define GRID_REGULATION_AL_35 35
#define GRID_REGULATION_AL_35_DESC "UL 1741"
#define GRID_REGULATION_AL_36 36
#define GRID_REGULATION_AL_36_DESC "UL1741-Rule 21"
#define GRID_REGULATION_AL_37 37
#define GRID_REGULATION_AL_37_DESC "UL1741-Hawaiian"
#define GRID_REGULATION_AL_38 38
#define GRID_REGULATION_AL_38_DESC "EN50549"

#ifdef EMS_35_36
// Note 26: Household Inverter fault codes
#define HOUSEHOLD_INVERTER_FAULT_1_BIT_0 0
#define HOUSEHOLD_INVERTER_FAULT_1_BIT_0_DESC "Grid_OVP"

// Note 27: Household Inverter fault extend codes
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_0 0
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_0_DESC "ac_hct_check_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_1 1
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_1_DESC "dci_consistency_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_2 2
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_2_DESC "gfci_consistency_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_3 3
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_3_DESC "relay_device_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_4 4
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_4_DESC "ac_hct_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_5 5
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_5_DESC "ground_i_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_6 6
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_6_DESC "utility_phase_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_7 7
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_7_DESC "utility_loss"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_8 8
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_8_DESC "internal_fan_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_9 9
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_9_DESC "fac_consistency_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_10 10
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_10_DESC "vac_consistency_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_11 11
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_11_DESC "phase_angle_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_12 12
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_12_DESC "dsp_communications_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_13 13
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_13_DESC "eeprom_rw_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_14 14
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_14_DESC "vac_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_15 15
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_15_DESC "fac_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_16 16
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_16_DESC "external_fan_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_17 17
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_17_DESC "afci_device_failure"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_18 18
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_18_DESC "bus_soft_timeout"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_19 19
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_19_DESC "dc_bus_short"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_20 20
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_20_DESC "inv_soft_timeout"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_21 21
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_21_DESC "grid_load_reverse"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_22 22
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_22_DESC "ipe_reverse"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_23 23
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_23_DESC "ems_sci"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_24 24
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_24_DESC "ems_can"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_25 25
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_25_DESC "sps_12v_ref"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_26 26
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_26_DESC "1p5v_ref"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_27 27
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_27_DESC "0p5v_ref"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_28 28
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_28_DESC "ntc_loss"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_29 29
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_29_DESC "inv_hct"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_30 30
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_30_DESC "load_ct"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_31 31
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_1_BIT_31_DESC "pv1_ct"

#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_0 0
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_0_DESC "pv2_ct"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_1 1
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_1_DESC "bat1_ct"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_2 2
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_2_DESC "bat2_ct"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_3 3
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_3_DESC "bypass_rly"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_4 4
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_4_DESC "load_rly"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_5 5
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_5_DESC "npe_rly"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_6 6
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_6_DESC "dci"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_7 7
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_7_DESC "watchdog"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_8 8
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_8_DESC "inv_open_loop"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_9 9
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_9_DESC "sw_consistency"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_10 10
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_10_DESC "n_n_reverse_lost"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_11 11
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_11_DESC "ini_fault"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_12 12
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_12_DESC "dsp_b_fault"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_13 13
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_13_DESC "inverter_circuit_abnormal"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_14 14
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_14_DESC "boost_circuit_abnormal"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_15 15
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_15_DESC "data_storage_error"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_16 16
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_16_DESC "para_can"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_17 17
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_17_DESC "para_synsignal_wrong"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_18 18
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_18_DESC "para_sw_diff"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_19 19
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_19_DESC "para_module_wrong"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_20 20
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_20_DESC "para_negative_power"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_21 21
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_21_DESC "para_multi_master"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_22 22
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_22_DESC "para_turnon_wrong"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_23 23
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_23_DESC "hw_ver_diff"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_24 24
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_24_DESC "bus_unbalance"
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_25 25
#define HOUSEHOLD_INVERTER_FAULT_EXTEND_2_BIT_25_DESC "inv_line_short"
#endif // EMS_35_36

// Note 28: Battery Warning
#define BATTERY_WARNING_BIT_0 "Temperature imbalance"
#define BATTERY_WARNING_BIT_1 "Over temperature"
#define BATTERY_WARNING_BIT_2 "Discharge low temperature"
#define BATTERY_WARNING_BIT_3 "Charge low temperature"
#define BATTERY_WARNING_BIT_4 "Discharge over current"
#define BATTERY_WARNING_BIT_5 "Charge over current"
#define BATTERY_WARNING_BIT_6 "Cell over voltage"
#define BATTERY_WARNING_BIT_7 "Cell low voltage"
#define BATTERY_WARNING_BIT_8 "sw_inconsistence"
#define BATTERY_WARNING_BIT_9 "mos_temperature_sensor_error"
#define BATTERY_WARNING_BIT_10 "soc_inconsistence"
#define BATTERY_WARNING_BIT_11 "bms_sci_lost"
#define BATTERY_WARNING_BIT_12 "bms_fan_err"

// Frame and Function Codes
#define MAX_FRAME_SIZE_ZERO_INDEXED 63
#define MIN_FRAME_SIZE_ZERO_INDEXED 4
#define MAX_FRAME_SIZE_RESPONSE_WRITE_SUCCESS_ZERO_INDEXED 7

#define MODBUS_FN_READDATAREGISTER 0x03
#define MODBUS_FN_WRITEDATAREGISTER 0x10
#define MODBUS_FN_WRITESINGLEREGISTER 0x06

#define FRAME_POSITION_SLAVE_ID 0
#define FRAME_POSITION_FUNCTION_CODE 1



// Ensure we stick to fixed values by forcing from a selection of values for data type returned
enum modbusReturnDataType
{
	notDefined,
	unsignedInt,
	signedInt,
	unsignedShort,
	signedShort,
	character
};
#define MODBUS_RETURN_DATA_TYPE_NOT_DEFINED_DESC "notDefined"
#define MODBUS_RETURN_DATA_TYPE_UNSIGNED_INT_DESC "unsignedInt"
#define MODBUS_RETURN_DATA_TYPE_SIGNED_INT_DESC "signedInt"
#define MODBUS_RETURN_DATA_TYPE_UNSIGNED_SHORT_DESC "unsignedShort"
#define MODBUS_RETURN_DATA_TYPE_SIGNED_SHORT_DESC "signedShort"
#define MODBUS_RETURN_DATA_TYPE_CHARACTER_DESC "character"




// Ensure we stick to fixed values by forcing from a selection of values for a Modbus request & response
enum modbusRequestAndResponseStatusValues
{
	preProcessing,
	notHandledRegister,
	invalidFrame,
	responseTooShort,
	noResponse,
	noMQTTPayload,
	invalidMQTTPayload,
	writeSingleRegisterSuccess,
	writeDataRegisterSuccess,
	readDataRegisterSuccess,
	slaveError,
	setDischargeSuccess,
	setChargeSuccess,
	setNormalSuccess,
	payloadExceededCapacity,
	addedToPayload,
//	notValidIncomingTopic,
	readDataInvalidValue
};
#define MODBUS_REQUEST_AND_RESPONSE_PREPROCESSING_MQTT_DESC "preProcessing"
#define MODBUS_REQUEST_AND_RESPONSE_NOT_HANDLED_REGISTER_MQTT_DESC "notHandledRegister"
#define MODBUS_REQUEST_AND_RESPONSE_INVALID_FRAME_MQTT_DESC "invalidFrame"
#define MODBUS_REQUEST_AND_RESPONSE_RESPONSE_TOO_SHORT_MQTT_DESC "responseTooShort"
#define MODBUS_REQUEST_AND_RESPONSE_NO_RESPONSE_MQTT_DESC "noResponse"
#define MODBUS_REQUEST_AND_RESPONSE_NO_MQTT_PAYLOAD_MQTT_DESC "noMQTTPayload"
#define MODBUS_REQUEST_AND_RESPONSE_INVALID_MQTT_PAYLOAD_MQTT_DESC "invalidMQTTPayload"
#define MODBUS_REQUEST_AND_RESPONSE_WRITE_SINGLE_REGISTER_SUCCESS_MQTT_DESC "writeSingleRegisterSuccess"
#define MODBUS_REQUEST_AND_RESPONSE_WRITE_DATA_REGISTER_SUCCESS_MQTT_DESC "writeDataRegisterSuccess"
#define MODBUS_REQUEST_AND_RESPONSE_READ_DATA_REGISTER_SUCCESS_MQTT_DESC "readDataRegisterSuccess"
#define MODBUS_REQUEST_AND_RESPONSE_ERROR_MQTT_DESC "slaveError"
#define MODBUS_REQUEST_AND_RESPONSE_SET_DISCHARGE_SUCCESS_MQTT_DESC "setDishargeSuccess"
#define MODBUS_REQUEST_AND_RESPONSE_SET_CHARGE_SUCCESS_MQTT_DESC "setChargeSuccess"
#define MODBUS_REQUEST_AND_RESPONSE_SET_NORMAL_SUCCESS_MQTT_DESC "setNormalSuccess"
#define MODBUS_REQUEST_AND_RESPONSE_PAYLOAD_EXCEEDED_CAPACITY_MQTT_DESC "payloadExceededCapacity"
#define MODBUS_REQUEST_AND_RESPONSE_ADDED_TO_PAYLOAD_MQTT_DESC "addedToPayload"
//#define MODBUS_REQUEST_AND_RESPONSE_NOT_VALID_INCOMING_TOPIC_MQTT_DESC "notValidIncomingTopic"
#define MODBUS_REQUEST_AND_RESPONSE_READ_DATA_INVALID_VALUE_MQTT_DESC "readDataInvalidValue"


#define MODBUS_REQUEST_AND_RESPONSE_PREPROCESSING_DISPLAY_DESC "PRE-PROC"
#define MODBUS_REQUEST_AND_RESPONSE_NOT_HANDLED_REGISTER_DISPLAY_DESC "NOT-HANDL"
#define MODBUS_REQUEST_AND_RESPONSE_INVALID_FRAME_DISPLAY_DESC "RSP-BADCRC"
#define MODBUS_REQUEST_AND_RESPONSE_RESPONSE_TOO_SHORT_DISPLAY_DESC "RSP-SHORT"
#define MODBUS_REQUEST_AND_RESPONSE_NO_RESPONSE_DISPLAY_DESC "NO-RSP"
#define MODBUS_REQUEST_AND_RESPONSE_NO_DISPLAY_PAYLOAD_DISPLAY_DESC "NO-PAYLOAD"
#define MODBUS_REQUEST_AND_RESPONSE_INVALID_DISPLAY_PAYLOAD_DISPLAY_DESC "INV-PAYLOA"
#define MODBUS_REQUEST_AND_RESPONSE_WRITE_SINGLE_REGISTER_SUCCESS_DISPLAY_DESC "W-SR-SUC"
#define MODBUS_REQUEST_AND_RESPONSE_WRITE_DATA_REGISTER_SUCCESS_DISPLAY_DESC "W-DR-SUC"
#define MODBUS_REQUEST_AND_RESPONSE_READ_DATA_REGISTER_SUCCESS_DISPLAY_DESC "R-DR-SUC"
#define MODBUS_REQUEST_AND_RESPONSE_ERROR_DISPLAY_DESC "SLA-ERROR"
#define MODBUS_REQUEST_AND_RESPONSE_SET_DISCHARGE_SUCCESS_DISPLAY_DESC "SET-DI-SUC"
#define MODBUS_REQUEST_AND_RESPONSE_SET_CHARGE_SUCCESS_DISPLAY_DESC "SET-CH-SUC"
#define MODBUS_REQUEST_AND_RESPONSE_SET_NORMAL_SUCCESS_DISPLAY_DESC "SET-NO-SUC"
#define MODBUS_REQUEST_AND_RESPONSE_PAYLOAD_EXCEEDED_CAPACITY_DISPLAY_DESC "PAYLOAD-ER"
#define MODBUS_REQUEST_AND_RESPONSE_ADDED_TO_PAYLOAD_DISPLAY_DESC "ADDED-PAYL"
//#define MODBUS_REQUEST_AND_RESPONSE_NOT_VALID_INCOMING_TOPIC_DISPLAY_DESC "INV-IN-TOP"
#define MODBUS_REQUEST_AND_RESPONSE_READ_DATA_INVALID_VALUE_DISPLAY_DESC "INV-VAL"

#define MAX_CHARACTER_VALUE_LENGTH 21
#define MAX_MQTT_NAME_LENGTH 81
#define MAX_MQTT_STATUS_LENGTH 51
#define MAX_FORMATTED_DATA_VALUE_LENGTH 513
#define MAX_DATA_TYPE_DESC_LENGTH 20
#define MAX_FORMATTED_DATE_LENGTH 21
#define OLED_CHARACTER_WIDTH_LARGE 21
#define OLED_CHARACTER_WIDTH_SMALL 11
#ifdef LARGE_DISPLAY
#define OLED_CHARACTER_WIDTH OLED_CHARACTER_WIDTH_LARGE
#else // LARGE_DISPLAY
#define OLED_CHARACTER_WIDTH OLED_CHARACTER_WIDTH_SMALL
#endif // LARGE_DISPLAY

// This is the request and return object for the sendModbus() function.
// Some vars are populated prior to the request, which can be used in the response to handle the data
// appropriately, for example typecasting and cleansing.
struct modbusRequestAndResponse
{
	//uint8_t errorLevel;
	uint8_t data[MAX_FRAME_SIZE_ZERO_INDEXED] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	uint8_t dataSize = 0;
	uint8_t functionCode = 0;

	char statusMqttMessage[MAX_MQTT_STATUS_LENGTH] = MODBUS_REQUEST_AND_RESPONSE_PREPROCESSING_MQTT_DESC;
	char displayMessage[OLED_CHARACTER_WIDTH_SMALL] = MODBUS_REQUEST_AND_RESPONSE_PREPROCESSING_DISPLAY_DESC;

	// These variables will be set by the sending process
	uint8_t registerCount = 0;
	modbusReturnDataType returnDataType = modbusReturnDataType::notDefined;
	char returnDataTypeDesc[MAX_DATA_TYPE_DESC_LENGTH] = MODBUS_RETURN_DATA_TYPE_NOT_DEFINED_DESC;
//	bool hasLookup = false;

	// And one of these will be set by the receiving process
	uint32_t unsignedIntValue = 0;
	int32_t signedIntValue = 0;
	uint16_t unsignedShortValue = 0;
	int16_t signedShortValue = 0;
	char characterValue[MAX_CHARACTER_VALUE_LENGTH] = "";
	char dataValueFormatted[MAX_FORMATTED_DATA_VALUE_LENGTH] = "";
};

// MQTT HA Subscription - Lets us know if HA restarts.
#define MQTT_SUB_HOMEASSISTANT "homeassistant/status"

enum mqttEntityId {
#ifdef DEBUG_FREEMEM
	entityFreemem,
#endif // DEBUG_FREEMEM
#ifdef DEBUG_CALLBACKS
	entityCallbacks,
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_WIFI
	entityRSSI,
	entityBSSID,
	entityTxPower,
	entityWifiRecon,
#endif // DEBUG_WIFI
#ifdef DEBUG_RS485
	entityRs485Errors,
#endif // DEBUG_RS485
	entityRs485Avail,
	entityA2MUptime,
	entityA2MVersion,
	entityInverterSn,
	entityInverterVersion,
	entityEmsSn,
	entityEmsVersion,
	entityBatSoc,
	entityBatPwr,
	entityBatEnergyCharge,
	entityBatEnergyDischarge,
	entityGridAvail,
	entityGridPwr,
	entityGridEnergyTo,
	entityGridEnergyFrom,
	entityPvPwr,
	entityPvEnergy,
	entityFrequency,
	entityOpMode,
	entitySocTarget,
	entityChargePwr,
	entityDischargePwr,
	entityPushPwr,
	entityBatCap,
	entityBatTemp,
	entityInverterTemp,
	entityBatFaults,
	entityBatWarnings,
	entityInverterFaults,
	entityInverterWarnings,
	entitySystemFaults,
	entityInverterMode,
	entityGridReg,
	entityRegNum,
	entityRegValue
};

enum mqttUpdateFreq {
	freqTenSec,
	freqOneMin,
	freqFiveMin,
	freqOneHour,
	freqOneDay,
	freqNever		// included in something else (i.e. statusTopic items)
};

enum homeAssistantClass {
	haClassEnergy,
	haClassPower,
	haClassBinaryProblem,
	haClassBattery,
	haClassVoltage,
	haClassCurrent,
	haClassFrequency,
	haClassTemp,
	haClassDuration,
	haClassInfo,
	haClassSelect,
	haClassBox,
	haClassNumber
};

enum opMode {
	opModePvCharge,
	opModeTarget,
	opModePush,
	opModeLoadFollow,
	opModeMaxCharge,
	opModeNoCharge
};

#define OP_MODE_DESC_PV_CHARGE		"PV Charge"
#define OP_MODE_DESC_TARGET		"Target SOC"
#define OP_MODE_DESC_PUSH		"Push To Grid"
#define OP_MODE_DESC_LOAD_FOLLOW	"Load Follow"
#define OP_MODE_DESC_MAX_CHARGE		"Max Charge"
#define OP_MODE_DESC_NO_CHARGE		"No Charge"

enum gridStatus {
	gridOnline,
	gridOffline,
	gridUnknown
};

struct mqttState
{
	mqttEntityId entityId;
	char mqttName[MAX_MQTT_NAME_LENGTH];
	mqttUpdateFreq updateFreq;
	bool subscribe;
	bool retain;
	homeAssistantClass haClass;
};

#endif // ! _Definitions_h
