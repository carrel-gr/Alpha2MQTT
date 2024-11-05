/*
Name:		Alpha2MQTT.ino
Created:	24/Aug/2022
Author:		Daniel Young

This file is part of Alpha2MQTT (A2M) which is released under GNU GENERAL PUBLIC LICENSE.
See file LICENSE or go to https://choosealicense.com/licenses/gpl-3.0/ for full license details.

Notes

First, go and customise options at the top of Definitions.h!
*/

#include <bit>
#include <bitset>
#include <cstdint>
#include <iostream>
// Supporting files
#include "RegisterHandler.h"
#include "RS485Handler.h"
#include "Definitions.h"
#include <Arduino.h>
#if defined MP_ESP8266
#include <ESP8266WiFi.h>
#elif defined MP_ESP32
#include <WiFi.h>
#define LED_BUILTIN 2
#endif
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Device parameters
char _version[6] = "v2.52";
char deviceSerialNumber[17]; // 8 registers = max 16 chars (usually 15)
char deviceBatteryType[32];
char haUniqueId[32];
char statusTopic[128];

// WiFi parameters
WiFiClient _wifi;
#if defined MP_ESP8266
#define WIFI_POWER_MAX 20.5
#define WIFI_POWER_MIN 12
#define WIFI_POWER_DECREMENT .25
float wifiPower = WIFI_MAX_POWER + WIFI_POWER_DECREMENT;  // Will decrement once before setting
#else // MP_ESP8266
wifi_power_t wifiPower = WIFI_POWER_11dBm; // Will bump to max before setting
#endif // MP_ESP8266

// MQTT parameters
PubSubClient _mqtt(_wifi);

// Buffer Size (and therefore payload size calc)
int _maxPayloadSize;

// I want to declare this once at a modular level, keep the heap somewhere in check.
//char _mqttPayload[MAX_MQTT_PAYLOAD_SIZE] = "";
char* _mqttPayload;

bool resendHaData = false;
bool resendAllData = false;

// OLED variables
char _oledOperatingIndicator = '*';
char _oledLine2[OLED_CHARACTER_WIDTH] = "";
char _oledLine3[OLED_CHARACTER_WIDTH] = "";
char _oledLine4[OLED_CHARACTER_WIDTH] = "";

// RS485 and AlphaESS functionality are packed up into classes
// to keep separate from the main program logic.
RS485Handler* _modBus;
RegisterHandler* _registerHandler;

// Fixed char array for messages to the serial port
char _debugOutput[128];

int32_t regNumberToRead = -1;
#ifdef DEBUG_WIFI
uint32_t wifiReconnects = 0;
#endif // DEBUG_WIFI
#ifdef DEBUG_CALLBACKS
uint32_t receivedCallbacks = 0;
uint32_t unknownCallbacks = 0;
uint32_t badCallbacks = 0;
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_RS485
uint32_t rs485Errors = 0;
uint32_t rs485InvalidValues = 0;
#endif // DEBUG_RS485
#ifdef DEBUG_OPS
uint32_t opCounter = 0;
#endif // DEBUG_OPS

//#define OP_DATA_AVG_CNT 4
#define PUSH_FUDGE_FACTOR 200 // Watts
struct {
	opMode   a2mOpMode = opMode::opModeLoadFollow;
	bool     a2mReadyToUseOpMode = false;
	uint16_t a2mSocTarget = SOC_TARGET_MAX;   // Stored as percent (0-100)
	bool     a2mReadyToUseSocTarget = false;
	int32_t  a2mPwrCharge = INVERTER_POWER_MAX;
	bool     a2mReadyToUsePwrCharge = false;
	int32_t  a2mPwrDischarge = INVERTER_POWER_MAX;
	bool     a2mReadyToUsePwrDischarge = false;
	int32_t  a2mPwrPush = 0;
	bool     a2mReadyToUsePwrPush = false;

	uint16_t essDispatchStart = DISPATCH_START_STOP;
	uint16_t essDispatchMode = 0;
	int32_t  essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET;
	uint16_t essDispatchSoc = 0;      // Stored as ESS register value. (percent / 0.4)
	uint16_t essBatterySoc = 0;       // Stored as ESS register value. (percent / 0.1)
	int16_t  essBatteryPower = 0;	// positive->discharge : negative->charge
//	int16_t  essBatteryPowerAvg = 0;
	int32_t  essGridPower = 0;	// positive->fromGrid : negative->toGrid
//	int32_t  essGridPowerAvg = 0;
	int32_t  essPvPower = 0;	// Positive
} opData;

/*
 * Home Assistant auto-discovered values
 */
static struct mqttState _mqttAllEntities[] =
{
#ifdef DEBUG_FREEMEM
	{ mqttEntityId::entityFreemem,            "A2M_freemem",          mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassInfo },
#endif
#ifdef DEBUG_CALLBACKS
	{ mqttEntityId::entityCallbacks,          "A2M_Callbacks",        mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassInfo },
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_RS485
	{ mqttEntityId::entityRs485Errors,        "A2M_RS485_Errors",     mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityRs485InvalidVals,   "A2M_RS485_Inv_Vals",   mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassInfo },
#endif // DEBUG_RS485
#ifdef DEBUG_WIFI
	{ mqttEntityId::entityRSSI,               "A2M_RSSI",             mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityBSSID,              "A2M_BSSID",            mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityTxPower,            "A2M_TX_Power",         mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityWifiRecon,          "A2M_reconnects",       mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassInfo },
#endif // DEBUIG_WIFI
	{ mqttEntityId::entityA2MUptime,          "A2M_uptime",           mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassDuration },
	{ mqttEntityId::entityA2MVersion,         "A2M_version",          mqttUpdateFreq::updateFreqOneDay,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityInverterVersion,    "Inverter_version",     mqttUpdateFreq::updateFreqOneDay,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityInverterSn,         "Inverter_SN",          mqttUpdateFreq::updateFreqOneDay,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityEmsVersion,         "EMS_version",          mqttUpdateFreq::updateFreqOneDay,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityEmsSn,              "EMS_SN",               mqttUpdateFreq::updateFreqOneDay,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityBatSoc,             "State_of_Charge",      mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassBattery },
	{ mqttEntityId::entityBatPwr,             "ESS_Power",            mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassPower },
	{ mqttEntityId::entityBatEnergyCharge,    "ESS_Energy_Charge",    mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassEnergy },
	{ mqttEntityId::entityBatEnergyDischarge, "ESS_Energy_Discharge", mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassEnergy },
	{ mqttEntityId::entityGridAvail,          "Grid_Connected",       mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassBinaryProblem },
	{ mqttEntityId::entityGridPwr,            "Grid_Power",           mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassPower },
	{ mqttEntityId::entityGridEnergyTo,       "Grid_Energy_To",       mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassEnergy },
	{ mqttEntityId::entityGridEnergyFrom,     "Grid_Energy_From",     mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassEnergy },
	{ mqttEntityId::entityPvPwr,              "Solar_Power",          mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassPower },
	{ mqttEntityId::entityPvEnergy,           "Solar_Energy",         mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassEnergy },
	{ mqttEntityId::entityOpMode,             "Op_Mode",              mqttUpdateFreq::updateFreqOneMin,  true,  homeAssistantClass::homeAssistantClassSelect },
	{ mqttEntityId::entitySocTarget,          "SOC_Target",           mqttUpdateFreq::updateFreqOneMin,  true,  homeAssistantClass::homeAssistantClassBox },
	{ mqttEntityId::entityChargePwr,          "Charge_Power",         mqttUpdateFreq::updateFreqOneMin,  true,  homeAssistantClass::homeAssistantClassBox },
	{ mqttEntityId::entityDischargePwr,       "Discharge_Power",      mqttUpdateFreq::updateFreqOneMin,  true,  homeAssistantClass::homeAssistantClassBox },
	{ mqttEntityId::entityPushPwr,            "Push_Power",           mqttUpdateFreq::updateFreqOneMin,  true,  homeAssistantClass::homeAssistantClassBox },
	{ mqttEntityId::entityBatCap,             "Battery_Capacity",     mqttUpdateFreq::updateFreqOneDay,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityBatTemp,            "Battery_Temp",         mqttUpdateFreq::updateFreqFiveMin, false, homeAssistantClass::homeAssistantClassTemp },
	{ mqttEntityId::entityInverterTemp,       "Inverter_Temp",        mqttUpdateFreq::updateFreqFiveMin, false, homeAssistantClass::homeAssistantClassTemp },
	{ mqttEntityId::entityBatFaults,          "Battery_Faults",       mqttUpdateFreq::updateFreqFiveMin, false, homeAssistantClass::homeAssistantClassNumber },
	{ mqttEntityId::entityBatWarnings,        "Battery_Warnings",     mqttUpdateFreq::updateFreqFiveMin, false, homeAssistantClass::homeAssistantClassNumber },
	{ mqttEntityId::entityInverterFaults,     "Inverter_Faults",      mqttUpdateFreq::updateFreqFiveMin, false, homeAssistantClass::homeAssistantClassNumber },
	{ mqttEntityId::entityInverterWarnings,   "Inverter_Warnings",    mqttUpdateFreq::updateFreqFiveMin, false, homeAssistantClass::homeAssistantClassNumber },
	{ mqttEntityId::entitySystemFaults,       "System_Faults",        mqttUpdateFreq::updateFreqFiveMin, false, homeAssistantClass::homeAssistantClassNumber },
	{ mqttEntityId::entityGridReg,            "Grid_Regulation",      mqttUpdateFreq::updateFreqOneDay,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityRegNum,             "Register_Number",      mqttUpdateFreq::updateFreqOneMin,  true,  homeAssistantClass::homeAssistantClassBox },
	{ mqttEntityId::entityRegValue,           "Register_Value",       mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassInfo }
};




// These timers are used in the main loop.
#define RUNSTATE_INTERVAL 5000
#define STATUS_INTERVAL_TEN_SECONDS 10000
#define STATUS_INTERVAL_ONE_MINUTE 60000
#define STATUS_INTERVAL_FIVE_MINUTE 300000
#define STATUS_INTERVAL_ONE_HOUR 3600000
#define STATUS_INTERVAL_ONE_DAY 86400000
#define UPDATE_STATUS_BAR_INTERVAL 500

#ifdef LARGE_DISPLAY
Adafruit_SSD1306 _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 0);
#else // LARGE_DISPLAY
// Wemos OLED Shield set up. 64x48
// Pins D1 D2 if ESP8266
// Pins GPIO22 and GPIO21 (SCL/SDA) with optional reset on GPIO13 if ESP32
#ifdef OLED_HAS_RST_PIN
Adafruit_SSD1306 _display(0);
#else
Adafruit_SSD1306 _display(-1); // No RESET Pin
#endif
#endif // LARGE_DISPLAY

/*
 * setup
 *
 * The setup function runs once when you press reset or power the board
 */
void setup()
{
	// All for testing different baud rates to 'wake up' the inverter
	unsigned long knownBaudRates[7] = { 9600, 115200, 19200, 57600, 38400, 14400, 4800 };
	bool gotResponse = false;
	modbusRequestAndResponseStatusValues result = modbusRequestAndResponseStatusValues::preProcessing;
	modbusRequestAndResponse response;
	char baudRateString[10] = "";
	int baudRateIterator = -1;

	// Set up serial for debugging using an appropriate baud rate
	// This is for communication with the development environment, NOT the Alpha system
	// See Definitions.h for this.
	Serial.begin(9600);

	// Configure LED for output
	pinMode(LED_BUILTIN, OUTPUT);
	
	// Wire.setClock(10000);

	// Display time
	_display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize OLED with the I2C addr 0x3C (for the 64x48)
	_display.clearDisplay();
	_display.display();
	updateOLED(false, "", "", _version);

	// Bit of a delay to give things time to kick in
	delay(500);

#ifdef DEBUG
	sprintf(_debugOutput, "Starting.");
	Serial.println(_debugOutput);
#endif

	// Configure WIFI
	setupWifi(true);

	// Configure MQTT to the address and port specified above
	_mqtt.setServer(MQTT_SERVER, MQTT_PORT);
#ifdef DEBUG
	sprintf(_debugOutput, "About to request buffer");
	Serial.println(_debugOutput);
#endif
	for (int _bufferSize = (MAX_MQTT_PAYLOAD_SIZE + MQTT_HEADER_SIZE); _bufferSize >= MIN_MQTT_PAYLOAD_SIZE + MQTT_HEADER_SIZE; _bufferSize = _bufferSize - 1024) {
#ifdef DEBUG
		sprintf(_debugOutput, "Requesting a buffer of : %d bytes", _bufferSize);
		Serial.println(_debugOutput);
#endif

		if (_mqtt.setBufferSize(_bufferSize)) {
			
			_maxPayloadSize = _bufferSize - MQTT_HEADER_SIZE;
#ifdef DEBUG
			sprintf(_debugOutput, "_bufferSize: %d,\r\n\r\n_maxPayload (Including null terminator): %d", _bufferSize, _maxPayloadSize);
			Serial.println(_debugOutput);
#endif
			
			// Example, 2048, if declared as 2048 is positions 0 to 2047, and position 2047 needs to be zero.  2047 usable chars in payload.
			_mqttPayload = new char[_maxPayloadSize];
			if (_mqttPayload != NULL) {
				emptyPayload();
				break;
			} else {
#ifdef DEBUG
				sprintf(_debugOutput, "Coudln't allocate payload of %d bytes", _maxPayloadSize);
				Serial.println(_debugOutput);
#endif
			}
		} else {
#ifdef DEBUG
			sprintf(_debugOutput, "Coudln't allocate buffer of %d bytes", _bufferSize);
			Serial.println(_debugOutput);
#endif
		}
	}

	// And any messages we are subscribed to will be pushed to the mqttCallback function for processing
	_mqtt.setCallback(mqttCallback);

	// Set up the serial for communicating with the MAX
	_modBus = new RS485Handler;
	_modBus->setDebugOutput(_debugOutput);

	// Set up the helper class for reading with reading registers
	_registerHandler = new RegisterHandler(_modBus);

	// Iterate known baud rates until we find a success
	while (!gotResponse) {
		// Starts at -1, so increment to 0 for example
		baudRateIterator++;

		// Go back to zero if beyond the bounds
		if (baudRateIterator > (sizeof(knownBaudRates) / sizeof(knownBaudRates[0])) - 1) {
			baudRateIterator = 0;
		}

		// Update the display
		sprintf(baudRateString, "%lu", knownBaudRates[baudRateIterator]);

		updateOLED(false, "Test Baud", baudRateString, "");
#ifdef DEBUG
		sprintf(_debugOutput, "About To Try: %lu", knownBaudRates[baudRateIterator]);
		Serial.println(_debugOutput);
#endif
		// Set the rate
		_modBus->setBaudRate(knownBaudRates[baudRateIterator]);

		// Ask for a reading
#ifdef DEBUG_NO_RS485
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_SAFETY_TEST_RW_GRID_REGULATION, &response);
#endif // DEBUG_NO_RS485
		if (result != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
#ifdef DEBUG
			sprintf(_debugOutput, "Baud Rate Checker Problem: %s", response.statusMqttMessage);
			Serial.println(_debugOutput);
#endif
#ifdef DEBUG_RS485
			rs485Errors++;
#endif // DEBUG_RS485
			updateOLED(false, "Test Baud", baudRateString, response.displayMessage);

			// Delay a while before trying the next
			delay(1000);
		} else {
			// Excellent, baud rate is set in the class, we got a response.. get out of here
			gotResponse = true;
		}
	}

	// Get the serial number (especially prefix for error codes)
	getSerialNumber();

	// Connect to MQTT
	mqttReconnect();
	sendHaData();
	resendHaData = true;  // Tell loop() to do it again

	getA2mOpDataFromEss();
#ifndef HA_IS_OP_MODE_AUTHORITY
	opData.a2mReadyToUseOpMode = true;
	opData.a2mReadyToUseSocTarget = true;
	opData.a2mReadyToUsePwrCharge = true;
	opData.a2mReadyToUsePwrDischarge = true;
	// Don't set opData.a2mReadyToUsePwrPush here as HA is only source.
#endif // ! HA_IS_OP_MODE_AUTHORITY

	gotResponse = readEssOpData();
	// loop until we get one clean read
	while (!gotResponse) {
		gotResponse = readEssOpData();
	}
	sendData();
	resendAllData = true; // Tell sendData() to send everything again

	updateOLED(false, "", "", _version);
}




/*
 * loop
 *
 * The loop function runs over and over again until power down or reset
 */
void
loop()
{
#ifdef FORCE_RESTART
	static unsigned long autoReboot = 0;
#endif

	// Refresh LED Screen, will cause the status asterisk to flicker
	updateOLED(true, "", "", "");

	// Make sure WiFi is good
	if (WiFi.status() != WL_CONNECTED) {
		setupWifi(false);
		mqttReconnect();
		resendHaData = true;
	}

	// make sure mqtt is still connected
	if ((!_mqtt.connected()) || !_mqtt.loop()) {
		mqttReconnect();
		resendHaData = true;
	}

	// Check and display the runstate on the display
	updateRunstate();

	// Send HA auto-discovery info
	if (resendHaData == true) {
		sendHaData();
	}

	if (readEssOpData()) {
		static bool longEnough = false;
		if (!longEnough && getUptimeSeconds() > 60) {  // After a minute, set these even if we didn't get a callback
			longEnough = true;
			if (!opData.a2mReadyToUseOpMode) {
				opData.a2mReadyToUseOpMode = true;
			}
			if (!opData.a2mReadyToUseSocTarget) {
				opData.a2mReadyToUseSocTarget = true;
			}
			if (!opData.a2mReadyToUsePwrCharge) {
				opData.a2mReadyToUsePwrCharge = true;
			}
			if (!opData.a2mReadyToUsePwrDischarge) {
				opData.a2mReadyToUsePwrDischarge = true;
			}
			if (!opData.a2mReadyToUsePwrPush) {
				opData.a2mReadyToUsePwrPush = true;
			}
		}
		// Read and transmit all entity data to MQTT
		sendData();
	
		// Check the Operational mode.
		if (!checkEssOpMode()) {
			setEssOpMode();
		}
	}

	// Force Restart?
#ifdef FORCE_RESTART
	if (checkTimer(&autoReboot, FORCE_RESTART_HOURS * 60 * 60 * 1000)) {
		ESP.restart();
	}
#endif
}


uint32_t
getUptimeSeconds(void)
{
	static uint32_t uptimeSeconds = 0, uptimeSecondsSaved = 0;
	uint32_t nowSeconds = millis() / 1000;

	if (nowSeconds < uptimeSeconds) {
		// We wrapped
		uptimeSecondsSaved += (UINT32_MAX / 1000);;
	}
	uptimeSeconds = nowSeconds;
	return uptimeSecondsSaved + uptimeSeconds;
}


/*
  setupWifi

  Connect to WiFi
*/
void
setupWifi(bool initialConnect)
{
	char line3[OLED_CHARACTER_WIDTH];
	char line4[OLED_CHARACTER_WIDTH];

	// We start by connecting to a WiFi network
#ifdef DEBUG
	if (initialConnect) {
		sprintf(_debugOutput, "Connecting to %s", WIFI_SSID);
	} else {
		sprintf(_debugOutput, "Reconnect to %s", WIFI_SSID);
	}
	Serial.println(_debugOutput);
#endif
	if (initialConnect) {
		WiFi.disconnect(); // If it auto-started, restart it our way.
		delay(100);
#ifdef DEBUG_WIFI
	} else {
		wifiReconnects++;
#endif // DEBUG_WIFI
	}

	// And continually try to connect to WiFi.
	// If it doesn't, the device will just wait here before continuing
	for (int tries = 0; WiFi.status() != WL_CONNECTED; tries++) {
		snprintf(line3, sizeof(line3), "WiFi %d ...", tries);

		if (tries == 5000) {
			ESP.restart();
		}

		if (tries % 50 == 0) {
			WiFi.disconnect();

			// Set up in Station Mode - Will be connecting to an access point
			WiFi.mode(WIFI_STA);
			// Helps when multiple APs for our SSID
			WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
			WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);

			// Set the hostname for this Arduino
			WiFi.hostname(DEVICE_NAME);

			// And connect to the details defined at the top
			WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

#if defined MP_ESP8266
			wifiPower -= WIFI_POWER_DECREMENT;
			if (wifiPower < WIFI_POWER_MIN) {
				wifiPower = WIFI_POWER_MAX;
			}
			WiFi.setOutputPower(wifiPower);
			snprintf(line4, sizeof(line4), "TX: %0.2f", wifiPower);
#else
			switch (wifiPower) {
			case WIFI_POWER_19_5dBm:
				wifiPower = WIFI_POWER_19dBm;
				break;
			case WIFI_POWER_19dBm:
				wifiPower = WIFI_POWER_18_5dBm;
				break;
			case WIFI_POWER_18_5dBm:
				wifiPower = WIFI_POWER_17dBm;
				break;
			case WIFI_POWER_17dBm:
				wifiPower = WIFI_POWER_15dBm;
				break;
			case WIFI_POWER_15dBm:
				wifiPower = WIFI_POWER_13dBm;
				break;
			case WIFI_POWER_13dBm:
				wifiPower = WIFI_POWER_11dBm;
				break;
			case WIFI_POWER_11dBm:
			default:
				wifiPower = WIFI_POWER_19_5dBm;
				break;
			}
			WiFi.setTxPower(wifiPower);
			snprintf(line4, sizeof(line4), "TX: %0.01fdBm", (int)wifiPower / 4.0f);
#endif
		}

		if (initialConnect) {
			updateOLED(false, "Connecting", line3, line4);
		} else {
			updateOLED(false, "Reconnect", line3, line4);
		}
		delay(500);
	}

	// Output some debug information
#ifdef DEBUG
	Serial.print("WiFi connected, IP is ");
	Serial.println(WiFi.localIP());
	byte *bssid = WiFi.BSSID();
	sprintf(_debugOutput, "WiFi BSSID is %02X:%02X:%02X:%02X:%02X:%02X", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
	Serial.println(_debugOutput);
	Serial.print("WiFi RSSI: ");
	Serial.println(WiFi.RSSI());
#endif

	// Connected, so ditch out with blank screen
	snprintf(line3, sizeof(line3), "%s", WiFi.localIP().toString().c_str());
	updateOLED(false, line3, "", _version);
}



/*
  checkTimer

  Check to see if the elapsed interval has passed since the passed in millis() value. If it has, return true and update the lastRun.
  Note that millis() overflows after 50 days, so we need to deal with that too... in our case we just zero the last run, which means the timer
  could be shorter but it's not critical... not worth the extra effort of doing it properly for once in 50 days.
*/
bool
checkTimer(unsigned long *lastRun, unsigned long interval)
{
	unsigned long now = millis();

	if (*lastRun > now)
		*lastRun = 0;

	if (*lastRun == 0 || now >= *lastRun + interval) {
		*lastRun = now;
		return true;
	}

	return false;
}

#define CURSOR_LINE_1 0
#define CURSOR_LINE_2 ((SCREEN_HEIGHT / 4) * 1)
#define CURSOR_LINE_3 ((SCREEN_HEIGHT / 4) * 2)
#define CURSOR_LINE_4 ((SCREEN_HEIGHT / 4) * 3)

/*
  updateOLED

  Update the OLED. Use "NULL" for no change to a line or "" for an empty line.
  Three parameters representing each of the three lines available for status indication - Top line functionality fixed
*/
void
updateOLED(bool justStatus, const char* line2, const char* line3, const char* line4)
{
	static unsigned long updateStatusBar = 0;

	_display.clearDisplay();
	_display.setTextSize(1);
	_display.setTextColor(WHITE);
	_display.setCursor(0, CURSOR_LINE_1);

	char line1Contents[OLED_CHARACTER_WIDTH];
	char line2Contents[OLED_CHARACTER_WIDTH];
	char line3Contents[OLED_CHARACTER_WIDTH];
	char line4Contents[OLED_CHARACTER_WIDTH];

	strlcpy(line2Contents, line2, sizeof(line2Contents));
	strlcpy(line3Contents, line3, sizeof(line3Contents));
	strlcpy(line4Contents, line4, sizeof(line4Contents));

	// Only update the operating indicator once per half second.
	if (checkTimer(&updateStatusBar, UPDATE_STATUS_BAR_INTERVAL)) {
		// Simply swap between space and asterisk every time we come here to give some indication of activity
		_oledOperatingIndicator = (_oledOperatingIndicator == '*') ? ' ' : '*';
	}

#ifdef LARGE_DISPLAY
	{
		int8_t rssi = WiFi.RSSI();
		// There's 20 characters we can play with, width wise.
		snprintf(line1Contents, sizeof(line1Contents), "A2M  %c%c%c         %3hhd",
			 _oledOperatingIndicator, (WiFi.status() == WL_CONNECTED ? 'W' : ' '), (_mqtt.connected() && _mqtt.loop() ? 'M' : ' '), rssi );
		_display.println(line1Contents);
		printWifiBars(rssi);
	}
#else // LARGE_DISPLAY
	// There's ten characters we can play with, width wise.
	snprintf(line1Contents, sizeof(line1Contents), "%s%c%c%c", "A2M    ",
		 _oledOperatingIndicator, (WiFi.status() == WL_CONNECTED ? 'W' : ' '), (_mqtt.connected() && _mqtt.loop() ? 'M' : ' ') );
	_display.println(line1Contents);
#endif // LARGE_DISPLAY

	// Next line
	_display.setCursor(0, CURSOR_LINE_2);
	if (!justStatus) {
		_display.println(line2Contents);
		strcpy(_oledLine2, line2Contents);
	} else {
		_display.println(_oledLine2);
	}

	_display.setCursor(0, CURSOR_LINE_3);
	if (!justStatus) {
		_display.println(line3Contents);
		strcpy(_oledLine3, line3Contents);
	} else {
		_display.println(_oledLine3);
	}

	_display.setCursor(0, CURSOR_LINE_4);
	if (!justStatus) {
		_display.println(line4Contents);
		strcpy(_oledLine4, line4Contents);
	} else {
		_display.println(_oledLine4);
	}
	// Refresh the display
	_display.display();
}

#define WIFI_X_POS 75 //102
void
printWifiBars(int rssi)
{
	if (rssi >= -55) { 
		_display.fillRect((WIFI_X_POS + 0),7,4,1, WHITE);
		_display.fillRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.fillRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.fillRect((WIFI_X_POS + 15),2,4,6, WHITE);
		_display.fillRect((WIFI_X_POS + 20),0,4,8, WHITE);
	} else if (rssi < -55 && rssi > -65) {
		_display.fillRect((WIFI_X_POS + 0),7,4,1, WHITE);
		_display.fillRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.fillRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.fillRect((WIFI_X_POS + 15),2,4,6, WHITE);
		_display.drawRect((WIFI_X_POS + 20),0,4,8, WHITE);
	} else if (rssi < -65 && rssi > -75) {
		_display.fillRect((WIFI_X_POS + 0),8,4,1, WHITE);
		_display.fillRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.fillRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.drawRect((WIFI_X_POS + 15),2,2,6, WHITE);
		_display.drawRect((WIFI_X_POS + 20),0,4,8, WHITE);
	} else if (rssi < -75 && rssi > -85) {
		_display.fillRect((WIFI_X_POS + 0),8,4,1, WHITE);
		_display.fillRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.drawRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.drawRect((WIFI_X_POS + 15),2,4,6, WHITE);
		_display.drawRect((WIFI_X_POS + 20),0,4,8, WHITE);
	} else if (rssi < -85 && rssi > -96) {
		_display.fillRect((WIFI_X_POS + 0),8,4,1, WHITE);
		_display.drawRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.drawRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.drawRect((WIFI_X_POS + 15),2,4,6, WHITE);
		_display.drawRect((WIFI_X_POS + 20),0,4,8, WHITE);
	} else {
		_display.drawRect((WIFI_X_POS + 0),8,4,1, WHITE);
		_display.drawRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.drawRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.drawRect((WIFI_X_POS + 15),2,4,6, WHITE);
		_display.drawRect((WIFI_X_POS + 20),0,4,8, WHITE);
	}
}



/*
 *getSerialNumber
 *
 * Display on load to demonstrate connectivty and send the prefix into RegisterHandler as
 * some system fault descriptions depend on knowing whether an AL based or AE based inverter.
 */
modbusRequestAndResponseStatusValues
getSerialNumber()
{
	modbusRequestAndResponseStatusValues result = modbusRequestAndResponseStatusValues::preProcessing;
	modbusRequestAndResponse response;
#ifndef DEBUG_NO_RS485
	uint32_t tries = 0;
#endif
	char oledLine3[OLED_CHARACTER_WIDTH];
	char oledLine4[OLED_CHARACTER_WIDTH];

#ifdef DEBUG_NO_RS485
	result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
	strcpy(response.dataValueFormatted, "AL9876543210987");
#else // DEBUG_NO_RS485
	// Get the serial number
	result = _registerHandler->readHandledRegister(REG_SYSTEM_INFO_R_EMS_SN_BYTE_1_2, &response);

	// Loop forever until we get this!
	while ((result != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) ||
	       (strlen(response.dataValueFormatted) < 15)) {
		tries++;
#ifdef DEBUG_RS485
		rs485Errors++;
#endif // DEBUG_RS485
		snprintf(oledLine4, sizeof(oledLine4), "%ld", tries);
		updateOLED(false, "Alpha sys", "not known", oledLine4);
		delay(1000);
		result = _registerHandler->readHandledRegister(REG_SYSTEM_INFO_R_EMS_SN_BYTE_1_2, &response);
	}
#endif // DEBUG_NO_RS485
	strlcpy(deviceSerialNumber, response.dataValueFormatted, 16);

#ifdef DEBUG_NO_RS485
	result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
	strcpy(response.dataValueFormatted, "FAKE-BAT");
#else // DEBUG_NO_RS485
	// Get the Battery Type
	result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_TYPE, &response);
	// Loop forever until we get this!
	while (result != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		tries++;
#ifdef DEBUG_RS485
		rs485Errors++;
#endif // DEBUG_RS485
		snprintf(oledLine4, sizeof(oledLine4), "%ld", tries);
		updateOLED(false, "Bat type", "not known", oledLine4);
		delay(1000);
		result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_TYPE, &response);
	}
#endif // DEBUG_NO_RS485
	strlcpy(deviceBatteryType, response.dataValueFormatted, sizeof(deviceBatteryType));

#ifdef LARGE_DISPLAY
	strlcpy(oledLine3, deviceSerialNumber, sizeof(oledLine3));
	strlcpy(oledLine4, deviceBatteryType, sizeof(oledLine4));
#else //LARGE_DISPLAY
	strlcpy(oledLine3, &response.dataValueFormatted[0], 11);
	strlcpy(oledLine4, &response.dataValueFormatted[10], 6);
#endif //LARGE_DISPLAY
	updateOLED(false, "Hello", oledLine3, oledLine4);

#ifdef DEBUG
	sprintf(_debugOutput, "Alpha Serial Number: %s", deviceSerialNumber);
	Serial.println(_debugOutput);
#endif

	_registerHandler->setSerialNumberPrefix(deviceSerialNumber[0], deviceSerialNumber[1]);
	snprintf(haUniqueId, sizeof(haUniqueId), "A2M-%s", deviceSerialNumber);
	snprintf(statusTopic, sizeof(statusTopic), DEVICE_NAME "/%s/status", haUniqueId);

	delay(4000);

	//Flash the LED
	digitalWrite(LED_BUILTIN, LOW);
	delay(4);
	digitalWrite(LED_BUILTIN, HIGH);

	return result;
}


/*
 * updateRunstate
 *
 * Determines a few things about the sytem and updates the display
 * Things updated - Dispatch state discharge/charge, battery power, battery percent
 */
#ifdef LARGE_DISPLAY
void
updateRunstate()
{
	static unsigned long lastRun = 0;
#ifndef DEBUG_NO_RS485
	const char *dMode = NULL, *dAction = NULL;
#endif
	char line2[OLED_CHARACTER_WIDTH] = "";
	char line3[OLED_CHARACTER_WIDTH] = "";
	char line4[OLED_CHARACTER_WIDTH] = "";


	if (checkTimer(&lastRun, RUNSTATE_INTERVAL)) {
		//Flash the LED
		digitalWrite(LED_BUILTIN, LOW);
		delay(4);
		digitalWrite(LED_BUILTIN, HIGH);

#ifdef DEBUG_NO_RS485
		strcpy(line2, "NO RS485");
#else // DEBUG_NO_RS485
		// Line 2: Get Dispatch Start - Is Alpha2MQTT controlling the inverter?
		if (opData.essDispatchStart != DISPATCH_START_START) {
			strcpy(line2, "Stopped");
		} else {
			switch (opData.essDispatchMode) {
			case DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV:
				dMode = "PV Only";
				break;
			case DISPATCH_MODE_STATE_OF_CHARGE_CONTROL:
				dMode = "SOC Ctl";
				break;
			case DISPATCH_MODE_LOAD_FOLLOWING:
				dMode = "LoadFollow";
				break;
			case DISPATCH_MODE_MAXIMISE_OUTPUT:
				dMode = "MaxOut";
				break;
			case DISPATCH_MODE_NORMAL_MODE:
				dMode = "Normal";
				break;
			case DISPATCH_MODE_OPTIMISE_CONSUMPTION:
				dMode = "OptConsmpt";
				break;
			case DISPATCH_MODE_MAXIMISE_CONSUMPTION:
				dMode = "MaxConsmpt";
				break;
			case DISPATCH_MODE_ECO_MODE:
				dMode = "ECO";
				break;
			case DISPATCH_MODE_FCAS_MODE:
				dMode = "FCAS";
				break;
			case DISPATCH_MODE_PV_POWER_SETTING:
				dMode = "PV Pwr";
				break;
			case DISPATCH_MODE_NO_BATTERY_CHARGE:
				dMode = "No Bat Chg";
				break;
			case DISPATCH_MODE_BURNIN_MODE:
				dMode = "Burnin";
				break;
			default:
				dMode = "BadMode";
				break;
			}

			// Determine if charging or discharging by looking at power
			if (opData.essDispatchActivePower < DISPATCH_ACTIVE_POWER_OFFSET) {
				dAction = "Charge";
			} else if (opData.essDispatchActivePower > DISPATCH_ACTIVE_POWER_OFFSET) {
				dAction = "Dischrg";
			} else {
				dAction = "Hold";
			}
			snprintf(line2, sizeof(line2), "%s : %s", dMode, dAction);
		}
#endif // DEBUG_NO_RS485

		// Get battery info for line 3
		snprintf(line3, sizeof(line3), "Bat: %4dW  %0.02f%%", opData.essBatteryPower, opData.essBatterySoc * BATTERY_SOC_MULTIPLIER);

		{   // Line 4 - Rotating diags
			static int debugIdx = 0;

			if (debugIdx < 1) {
				snprintf(line4, sizeof(line4), "Uptime: %lu", getUptimeSeconds());
				debugIdx = 1;
#ifdef DEBUG_FREEMEM
			} else if (debugIdx < 2) {
				snprintf(line4, sizeof(line4), "Mem: %u", freeMemory());
				debugIdx = 2;
#endif // DEBUG_FREEMEM
#ifdef DEBUG_WIFI
			} else if (debugIdx < 3) {
				snprintf(line4, sizeof(line4), "WiFi recon: %lu", wifiReconnects);
				debugIdx = 3;
			} else if (debugIdx < 4) {
				snprintf(line4, sizeof(line4), "WiFi TX: %0.01fdBm", (int)WiFi.getTxPower() / 4.0f);
				debugIdx = 4;
			} else if (debugIdx < 5) {
				snprintf(line4, sizeof(line4), "WiFi RSSI: %d", WiFi.RSSI());
				debugIdx = 5;
#endif // DEBUG_WIFI
#ifdef DEBUG_CALLBACKS
			} else if (debugIdx < 6) {
				snprintf(line4, sizeof(line4), "Callbacks: %lu", receivedCallbacks);
				debugIdx = 6;
			} else if (debugIdx < 7) {
				snprintf(line4, sizeof(line4), "Unk CBs: %lu", unknownCallbacks);
				debugIdx = 7;
			} else if (debugIdx < 8) {
				snprintf(line4, sizeof(line4), "Bad CBs: %lu", badCallbacks);
				debugIdx = 8;
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_RS485
			} else if (debugIdx < 9) {
				snprintf(line4, sizeof(line4), "RS485 Err: %lu", rs485Errors);
				debugIdx = 9;
			} else if (debugIdx < 10) {
				snprintf(line4, sizeof(line4), "RS485 Inv: %lu", rs485InvalidValues);
				debugIdx = 10;
#endif // DEBUG_RS485
			} else if (debugIdx < 11) {
				char tmpOpMode[12];
				getOpModeDesc(tmpOpMode, sizeof(tmpOpMode), opData.a2mOpMode);
				snprintf(line4, sizeof(line4), "OpMode: %s", tmpOpMode);
				debugIdx = 11;
#ifndef DEBUG_NO_RS485
			} else if (debugIdx < 12) {
				snprintf(line4, sizeof(line4), "Pwr: %ldW", DISPATCH_ACTIVE_POWER_OFFSET - opData.essDispatchActivePower);
				debugIdx = 12;
			} else if (debugIdx < 13) {
				snprintf(line4, sizeof(line4), "SOC TGT: %hu%% %0.02f%%", opData.a2mSocTarget, opData.essDispatchSoc * DISPATCH_SOC_MULTIPLIER);
				debugIdx = 13;
#endif // ! DEBUG_NO_RS485
#ifdef DEBUG_OPS
			} else if (debugIdx < 15) {
				snprintf(line4, sizeof(line4), "opCnt: %lu", opCounter);
				debugIdx = 15;
#endif // DEBUG_OPS
			} else { // Must be last
				snprintf(line4, sizeof(line4), "Version: %s", _version);
				debugIdx = 0;
			}
		}

		updateOLED(false, line2, line3, line4);
	}
}
#else // LARGE_DISPLAY
void updateRunstate()
{
	
	static unsigned long lastRun = 0;
	static int lastLine2 = 0;

	char line2[OLED_CHARACTER_WIDTH] = "";
	char line3[OLED_CHARACTER_WIDTH] = "";
	char line4[OLED_CHARACTER_WIDTH] = "";


	if (checkTimer(&lastRun, RUNSTATE_INTERVAL)) {
		//Flash the LED
		digitalWrite(LED_BUILTIN, LOW);
		delay(4);
		digitalWrite(LED_BUILTIN, HIGH);

		if (opData.essDispatchStart != DISPATCH_START_START) {
			strcpy(line2, "Stopped");
		} else {
			if (lastLine2 == 0) {
				lastLine2 = 1;
				// Get the mode.
				switch (opData.essDispatchMode) {
				case DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV:
					strcpy(line2, "PV Only");
					break;
				case DISPATCH_MODE_STATE_OF_CHARGE_CONTROL:
					strcpy(line2, "SOC Ctl");
					break;
				case DISPATCH_MODE_LOAD_FOLLOWING:
					strcpy(line2, "LoadFollow");
					break;
				case DISPATCH_MODE_MAXIMISE_OUTPUT:
					strcpy(line2, "MaxOut");
					break;
				case DISPATCH_MODE_NORMAL_MODE:
					strcpy(line2, "Normal");
					break;
				case DISPATCH_MODE_OPTIMISE_CONSUMPTION:
					strcpy(line2, "OptConsmpt");
					break;
				case DISPATCH_MODE_MAXIMISE_CONSUMPTION:
					strcpy(line2, "MaxConsmpt");
					break;
				case DISPATCH_MODE_ECO_MODE:
					strcpy(line2, "ECO");
					break;
				case DISPATCH_MODE_FCAS_MODE:
					strcpy(line2, "FCAS");
					break;
				case DISPATCH_MODE_PV_POWER_SETTING:
					strcpy(line2, "PV Pwr");
					break;
				case DISPATCH_MODE_NO_BATTERY_CHARGE:
					dMode = "No Bat Chg";
					break;
				case DISPATCH_MODE_BURNIN_MODE:
					dMode = "Burnin";
					break;
				default:
					strcpy(line2, "BadMode");
					break;
				}
			} else {
				lastLine2 = 0;
				// Determine if charging or discharging by looking at power
				if (opData.essDispatchActivePower < DISPATCH_ACTIVE_POWER_OFFSET) {
					strcpy(line2, "Charge");
				} else if (opData.essDispatchActivePower > DISPATCH_ACTIVE_POWER_OFFSET) {
					strcpy(line2, "Discharge");
				} else {
					strcpy(line2, "Hold");
				}
			}
		}

		if (lastLine2 == 1) {
			// Get battery power for line 3
			snprintf(line3, sizeof(line3), "Bat:%dW", opData.essBatteryPower);

			// And percent for line 4
			snprintf(line4, sizeof(line4), "%0.02f%%", opData.essBatterySoc * BATTERY_SOC_MULTIPLIER);
		} else {
			snprintf(line3, sizeof(line3), "Mem: %u", freeMemory());
#if defined MP_ESP8266
			snprintf(line4, sizeof(line4), "TX: %0.2f", wifiPower);
#else
			strcpy(line4, "");
#endif
		}

		updateOLED(false, line2, line3, line4);
	}
}
#endif // LARGE_DISPLAY



/*
 * mqttReconnect
 *
 *This function reconnects the ESP8266 to the MQTT broker
 */
void
mqttReconnect(void)
{
	bool subscribed = false;
	char subscriptionDef[100];
	char line3[OLED_CHARACTER_WIDTH];
	char line4[OLED_CHARACTER_WIDTH];
	int tries = 0;

	// Loop until we're reconnected
	while (true) {
		tries++;

		_mqtt.disconnect();		// Just in case.
		delay(200);

		if (WiFi.status() != WL_CONNECTED) {
			setupWifi(false);
		}

#ifdef DEBUG
		Serial.print("Attempting MQTT connection...");
#endif

		snprintf(line3, sizeof(line3), "MQTT %d ...", tries);
		snprintf(line4, sizeof(line4), "RSSI %d", WiFi.RSSI());
		updateOLED(false, "Connecting", line3, _version);
		delay(100);

		// Attempt to connect
		if (_mqtt.connect(haUniqueId, MQTT_USERNAME, MQTT_PASSWORD, statusTopic, 0, true, "offline")) {
			int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);
#ifdef DEBUG
			Serial.println("Connected MQTT");
#endif

			// Special case for Home Assistant
			sprintf(subscriptionDef, "%s", MQTT_SUB_HOMEASSISTANT);
			subscribed = _mqtt.subscribe(subscriptionDef, MQTT_SUBSCRIBE_QOS);
#ifdef DEBUG
			snprintf(_debugOutput, sizeof(_debugOutput), "Subscribed to \"%s\" : %d", subscriptionDef, subscribed);
			Serial.println(_debugOutput);
#endif

			for (int i = 0; i < numberOfEntities; i++) {
				if (_mqttAllEntities[i].subscribe) {
					sprintf(subscriptionDef, DEVICE_NAME "/%s/%s/command", haUniqueId, _mqttAllEntities[i].mqttName);
					subscribed = subscribed && _mqtt.subscribe(subscriptionDef, MQTT_SUBSCRIBE_QOS);
#ifdef DEBUG
					snprintf(_debugOutput, sizeof(_debugOutput), "Subscribed to \"%s\" : %d", subscriptionDef, subscribed);
					Serial.println(_debugOutput);
#endif
				}
			}

			// Subscribe or resubscribe to topics.
			if (subscribed) {
				break;
			}
		}

#ifdef DEBUG
		sprintf(_debugOutput, "MQTT Failed: RC is %d\r\nTrying again in five seconds...", _mqtt.state());
		Serial.println(_debugOutput);
#endif

		// Wait 5 seconds before retrying
		delay(5000);
	}
	// Connected, so ditch out with runstate on the screen, update some diags
	updateRunstate();
}

mqttState *
lookupSubscription(char *entityName)
{
	int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);
	for (int i = 0; i < numberOfEntities; i++) {
		if (_mqttAllEntities[i].subscribe &&
		    !strcmp(entityName, _mqttAllEntities[i].mqttName)) {
			return &_mqttAllEntities[i];
		}
	}
	return NULL;
}

mqttState *
lookupEntity(mqttEntityId entityId)
{
	int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);
	for (int i = 0; i < numberOfEntities; i++) {
		if (_mqttAllEntities[i].entityId == entityId) {
			return &_mqttAllEntities[i];
		}
	}
	return NULL;
}

modbusRequestAndResponseStatusValues
readEntity(mqttState *singleEntity, modbusRequestAndResponse* rs)
{
	modbusRequestAndResponseStatusValues result = modbusRequestAndResponseStatusValues::preProcessing;

	rs->dataValueFormatted[0] = 0;

	switch (singleEntity->entityId) {
	case mqttEntityId::entityRegValue:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = regNumberToRead;  // Just return the register #
		sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		if (regNumberToRead < 0) {
			rs->returnDataType = modbusReturnDataType::character;
			strcpy(rs->characterValue, "Nothing read");
			sprintf(rs->dataValueFormatted, "%s", rs->characterValue);
			result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		} else {
			result = _registerHandler->readHandledRegister(regNumberToRead, rs);
			switch (result) {
			case modbusRequestAndResponseStatusValues::readDataRegisterSuccess:
				if (!strcmp(rs->dataValueFormatted, "Unknown")) {
					switch (rs->returnDataType) {
					case modbusReturnDataType::character:
						sprintf(rs->dataValueFormatted, "Unknown (%s)", rs->characterValue);
						break;
					case modbusReturnDataType::signedInt:
						sprintf(rs->dataValueFormatted, "Unknown (%ld)", rs->signedIntValue);
						break;
					case modbusReturnDataType::signedShort:
						sprintf(rs->dataValueFormatted, "Unknown (%d)", rs->signedShortValue);
						break;
					case modbusReturnDataType::unsignedInt:
						sprintf(rs->dataValueFormatted, "Unknown (%lu)", rs->unsignedIntValue);
						break;
					case modbusReturnDataType::unsignedShort:
						sprintf(rs->dataValueFormatted, "Unknown (%u)", rs->unsignedShortValue);
						break;
					case modbusReturnDataType::notDefined:
						sprintf(rs->dataValueFormatted, "Unknown (XX)");
						break;
					}
				}
				break;
			case modbusRequestAndResponseStatusValues::notHandledRegister:
				strcpy(rs->dataValueFormatted, "Invalid register");
				break;
			case modbusRequestAndResponseStatusValues::noResponse:
				strcpy(rs->dataValueFormatted, "No response");
				break;
			case modbusRequestAndResponseStatusValues::responseTooShort:
				strcpy(rs->dataValueFormatted, "Resp too short");
				break;
			case modbusRequestAndResponseStatusValues::slaveError:
				strcpy(rs->dataValueFormatted, "Slave Error");
				break;
			case modbusRequestAndResponseStatusValues::invalidFrame:
				strcpy(rs->dataValueFormatted, "Invalid Frame");
				break;
			default:
				sprintf(rs->dataValueFormatted, "Unexpected result: %d", result);
				break;
			}
			result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		}
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityRegNum:
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = regNumberToRead;
		sprintf(rs->dataValueFormatted, "%ld", regNumberToRead);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityGridReg:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->unsignedShortValue = GRID_REGULATION_AL_17;
		sprintf(rs->dataValueFormatted, "%s", GRID_REGULATION_AL_17_DESC);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_SAFETY_TEST_RW_GRID_REGULATION, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityBatCap:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->unsignedShortValue = 2345;
		sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * INVERTER_TEMP_MULTIPLIER);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_CAPACITY, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityInverterTemp:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->unsignedShortValue = 2750;
		sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * INVERTER_TEMP_MULTIPLIER);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_INVERTER_HOME_R_INVERTER_TEMP, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityBatTemp:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->signedShortValue = 2750;
		sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * INVERTER_TEMP_MULTIPLIER);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_MAX_CELL_TEMPERATURE, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityBatFaults:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 0;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		{
			unsigned int count = 0;
			result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_FAULT_1, rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_FAULT_1_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_FAULT_2_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_FAULT_3_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_FAULT_4_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_FAULT_5_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_FAULT_6_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
			}
			sprintf(rs->dataValueFormatted, "%u", count);
		}
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityBatWarnings:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 1;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		{
			unsigned int count = 0;
			result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_WARNING_1, rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_WARNING_1_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_WARNING_2_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_WARNING_3_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_WARNING_4_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_WARNING_5_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_WARNING_6_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
			}
			sprintf(rs->dataValueFormatted, "%u", count);
		}
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityInverterFaults:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 0;
		sprintf(rs->dataValueFormatted, "%u", std::popcount(rs->unsignedIntValue));
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		{
			unsigned int count = 0;
			result = _registerHandler->readHandledRegister(REG_INVERTER_HOME_R_INVERTER_FAULT_1_1, rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_INVERTER_HOME_R_INVERTER_FAULT_2_1, rs);
			}
#ifdef EMS_35_36
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_1_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_2_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_3_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_4_1, rs);
			}
#endif // EMS_35_36
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
			}
			sprintf(rs->dataValueFormatted, "%u", count);
		}
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityInverterWarnings:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 22;
		sprintf(rs->dataValueFormatted, "%u", std::popcount(rs->unsignedIntValue));
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		{
			unsigned int count = 0;
			result = _registerHandler->readHandledRegister(REG_INVERTER_HOME_R_INVERTER_WARNING_1_1, rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_INVERTER_HOME_R_INVERTER_WARNING_2_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
			}
			sprintf(rs->dataValueFormatted, "%u", count);
		}
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entitySystemFaults:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 3;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		{
			unsigned int count = 0;
			result = _registerHandler->readHandledRegister(REG_SYSTEM_INFO_R_SYSTEM_FAULT, rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
				result = _registerHandler->readHandledRegister(REG_SYSTEM_OP_R_SYSTEM_FAULT_1, rs);
			}
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				count += std::popcount(rs->unsignedIntValue);
			}
			sprintf(rs->dataValueFormatted, "%u", count);
		}
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityPushPwr:
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = opData.a2mPwrPush;
		sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityDischargePwr:
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = opData.a2mPwrDischarge;
		sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityChargePwr:
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = opData.a2mPwrCharge;
		sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entitySocTarget:
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->unsignedShortValue = opData.a2mSocTarget / DISPATCH_SOC_MULTIPLIER;
		sprintf(rs->dataValueFormatted, "%u", opData.a2mSocTarget);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityOpMode:
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->unsignedShortValue = (uint16_t)opData.a2mOpMode;
		getOpModeDesc(rs->dataValueFormatted, sizeof(rs->dataValueFormatted), opData.a2mOpMode);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityPvEnergy:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 3399;
		sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * TOTAL_ENERGY_MULTIPLIER);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		{
			static unsigned int saved = 0;
			result = _registerHandler->readHandledRegister(REG_SYSTEM_OP_R_SYSTEM_TOTAL_PV_ENERGY_1, rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				if (rs->unsignedIntValue > (saved + 5)) {
					saved = rs->unsignedIntValue;
				} else {
					// Smooth out mini (glitch) values.
					rs->unsignedIntValue = saved;
					sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * TOTAL_ENERGY_MULTIPLIER);
				}
			}
		}
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityPvPwr:
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = opData.essPvPower;
		sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityGridEnergyTo:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 4499;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_GRID_METER_R_TOTAL_ENERGY_FEED_TO_GRID_1, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityGridEnergyFrom:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 4488;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_GRID_METER_R_TOTAL_ENERGY_CONSUMED_FROM_GRID_1, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityGridPwr:
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = opData.essGridPower;
		sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityGridAvail:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		{
			static unsigned short sval = 240;
			if (sval == 0) sval = 240;
			else if (sval == 240) sval = 100;
			else sval = 0;
			rs->unsignedShortValue = sval;
		}
		sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_GRID_METER_R_VOLTAGE_OF_A_PHASE, rs);
		if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			if (rs->unsignedShortValue <= 215 || rs->unsignedShortValue >= 265) {
				// De-bounce.  If "off" then try reading again.
				result = _registerHandler->readHandledRegister(REG_GRID_METER_R_VOLTAGE_OF_A_PHASE, rs);
			}
		}
#endif // DEBUG_NO_RS485
		if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			if (rs->unsignedShortValue > 215 && rs->unsignedShortValue < 265) {
				strcpy(rs->dataValueFormatted, "ok");
			} else {
				strcpy(rs->dataValueFormatted, "problem");
			}
		}
		break;
	case mqttEntityId::entityBatEnergyCharge:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 5599;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_CHARGE_ENERGY_1, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityBatEnergyDischarge:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 5588;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_DISCHARGE_ENERGY_1, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityBatPwr:
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->signedShortValue = opData.essBatteryPower;
		sprintf(rs->dataValueFormatted, "%d", rs->signedShortValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityBatSoc:
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->unsignedShortValue = opData.essBatterySoc;
		sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * BATTERY_SOC_MULTIPLIER);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
#ifdef DEBUG_CALLBACKS
	case mqttEntityId::entityCallbacks:
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = receivedCallbacks;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_RS485
	case mqttEntityId::entityRs485Errors:
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = rs485Errors;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityRs485InvalidVals:
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = rs485InvalidValues;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
#endif // DEBUG_RS485
#ifdef DEBUG_FREEMEM
	case mqttEntityId::entityFreemem:
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = freeMemory();
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
#endif // DEBUG_FREEMEM
	case mqttEntityId::entityA2MUptime:
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = getUptimeSeconds();
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityA2MVersion:
		rs->returnDataType = modbusReturnDataType::character;
		strcpy(rs->characterValue, _version);
		sprintf(rs->dataValueFormatted, "%s", rs->characterValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityInverterSn:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::character;
		strcpy(rs->characterValue, "fake-inv-sn");
		sprintf(rs->dataValueFormatted, "%s", rs->characterValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_INVERTER_INFO_R_SERIAL_NUMBER_1, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityInverterVersion:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::character;
		strcpy(rs->characterValue, "fake.inv.ver");
		sprintf(rs->dataValueFormatted, "%s", rs->characterValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_INVERTER_INFO_R_MASTER_SOFTWARE_VERSION_1, rs);
		if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			char master[64], slave[64];
			strlcpy(master, rs->dataValueFormatted, sizeof(master));
			result = _registerHandler->readHandledRegister(REG_INVERTER_INFO_R_SLAVE_SOFTWARE_VERSION_1, rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				strlcpy(slave, rs->dataValueFormatted, sizeof(slave));
				snprintf(rs->dataValueFormatted, sizeof(rs->dataValueFormatted), "%s-%s", master, slave);
			}
		}
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityEmsSn:
		rs->returnDataType = modbusReturnDataType::character;
		strcpy(rs->characterValue, deviceSerialNumber);
		sprintf(rs->dataValueFormatted, "%s", deviceSerialNumber);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityEmsVersion:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::character;
		strcpy(rs->characterValue, "fake.ems.ver");
		sprintf(rs->dataValueFormatted, "%s", rs->characterValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_SYSTEM_INFO_R_EMS_VERSION_HIGH, rs);
		if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			char high[8], middle[8], low[8];
			strlcpy(high, rs->dataValueFormatted, sizeof(high));
			result = _registerHandler->readHandledRegister(REG_SYSTEM_INFO_R_EMS_VERSION_MIDDLE, rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				strlcpy(middle, rs->dataValueFormatted, sizeof(middle));
				result = _registerHandler->readHandledRegister(REG_SYSTEM_INFO_R_EMS_VERSION_LOW, rs);
				if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
					strlcpy(low, rs->dataValueFormatted, sizeof(low));
					snprintf(rs->dataValueFormatted, sizeof(rs->dataValueFormatted), "%s.%s.%s", high, middle, low);
				}
			}
		}
#endif // DEBUG_NO_RS485
		break;
#ifdef DEBUG_WIFI
	case mqttEntityId::entityRSSI:
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = WiFi.RSSI();
		sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityBSSID:
		rs->returnDataType = modbusReturnDataType::character;
		{
			byte *bssid = WiFi.BSSID();
			sprintf(rs->characterValue, "%02X:%02X:%02X:%02X:%02X:%02X", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
		}
		strcpy(rs->dataValueFormatted, rs->characterValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityTxPower:
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = WiFi.getTxPower();
		sprintf(rs->dataValueFormatted, "%0.1f", rs->signedIntValue / 4.0f);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityWifiRecon:
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = wifiReconnects;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
#endif // DEBUG_WIFI
	}

	if (result == modbusRequestAndResponseStatusValues::readDataInvalidValue) {
#ifdef DEBUG_RS485
		rs485InvalidValues++;
#endif // DEBUG_RS485
#ifdef DEBUG
		snprintf(_debugOutput, sizeof(_debugOutput), "readEntity: invalid val for: %s: ", singleEntity->mqttName);
		Serial.print(_debugOutput);
		Serial.println(rs->dataValueFormatted);
#endif
	} else if (result != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
#ifdef DEBUG_RS485
		rs485Errors++;
#endif // DEBUG_RS485
#ifdef DEBUG
		snprintf(_debugOutput, sizeof(_debugOutput), "Failed to read register: %s, Result = %d", singleEntity->mqttName, result);
		Serial.println(_debugOutput);
#endif
	}

	return result;
}

/*
 * addState
 *
 * Query the handled entity in the usual way, and add the cleansed output to the buffer
 */
modbusRequestAndResponseStatusValues
addState(mqttState *singleEntity, modbusRequestAndResponseStatusValues *resultAddedToPayload)
{
	modbusRequestAndResponse response;
	modbusRequestAndResponseStatusValues result;

	// Read the register
	result = readEntity(singleEntity, &response);

	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		// Let the onward process also know if the buffer failed.
		*resultAddedToPayload = addToPayload(response.dataValueFormatted);
	} else {
		*resultAddedToPayload = modbusRequestAndResponseStatusValues::preProcessing;
	}
	return result;
}

void
sendStatus(void)
{
	char stateAddition[64] = "";
	modbusRequestAndResponseStatusValues resultAddedToPayload;

	emptyPayload();

	snprintf(stateAddition, sizeof(stateAddition), "online");
	resultAddedToPayload = addToPayload(stateAddition);
	if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
		return;
	}

	sendMqtt(statusTopic, MQTT_RETAIN);
}

modbusRequestAndResponseStatusValues
addConfig(mqttState *singleEntity, modbusRequestAndResponseStatusValues& resultAddedToPayload)
{
	char stateAddition[1024] = "";
	char prettyName[64];

	sprintf(stateAddition, "{");
	resultAddedToPayload = addToPayload(stateAddition);
	if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
		return resultAddedToPayload;
	}

	switch (singleEntity->haClass) {
	case homeAssistantClass::homeAssistantClassBox:
	case homeAssistantClass::homeAssistantClassNumber:
		sprintf(stateAddition, "\"component\": \"number\"");
		break;
	case homeAssistantClass::homeAssistantClassSelect:
		sprintf(stateAddition, "\"component\": \"select\"");
		break;
	case homeAssistantClass::homeAssistantClassBinaryProblem:
		sprintf(stateAddition, "\"component\": \"binary_sensor\"");
		break;
	default:
		sprintf(stateAddition, "\"component\": \"sensor\"");
		break;
	}
	resultAddedToPayload = addToPayload(stateAddition);
	if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
		return resultAddedToPayload;
	}

	snprintf(stateAddition, sizeof(stateAddition),
		 ", \"device\": {"
		 " \"name\": \"%s\", \"model\": \"%s\", \"manufacturer\": \"AlphaESS\","
		 " \"identifiers\": [\"%s\"]}",
		 haUniqueId, deviceBatteryType,
		 haUniqueId);
	resultAddedToPayload = addToPayload(stateAddition);
	if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
		return resultAddedToPayload;
	}

	strlcpy(prettyName, singleEntity->mqttName, sizeof(prettyName));
	while(char *ch = strchr(prettyName, '_')) {
		*ch = ' ';
	}
	snprintf(stateAddition, sizeof(stateAddition), ", \"name\": \"%s\"", prettyName);
	resultAddedToPayload = addToPayload(stateAddition);
	if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
		return resultAddedToPayload;
	}

	snprintf(stateAddition, sizeof(stateAddition), ", \"unique_id\": \"%s_%s\"", haUniqueId, singleEntity->mqttName);
	resultAddedToPayload = addToPayload(stateAddition);
	if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
		return resultAddedToPayload;
	}

	switch (singleEntity->haClass) {
	case homeAssistantClass::homeAssistantClassEnergy:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"energy\""
			 ", \"state_class\": \"total_increasing\""
			 ", \"unit_of_measurement\": \"kWh\""
			 ", \"force_update\": \"true\"");
		break;
	case homeAssistantClass::homeAssistantClassPower:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"power\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"W\""
			 ", \"force_update\": \"true\"");
		break;
	case homeAssistantClass::homeAssistantClassBinaryProblem:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"problem\""
			 ", \"payload_on\": \"problem\""
			 ", \"payload_off\": \"ok\"");
		break;
	case homeAssistantClass::homeAssistantClassBattery:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"battery\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"%%\""
			 ", \"force_update\": \"true\"");
		break;
	case homeAssistantClass::homeAssistantClassVoltage:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"voltage\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"V\""
			 ", \"force_update\": \"true\"");
		break;
	case homeAssistantClass::homeAssistantClassCurrent:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"current\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"A\""
			 ", \"force_update\": \"true\"");
		break;
	case homeAssistantClass::homeAssistantClassTemp:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"temperature\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"°C\""
//		 ", \"force_update\": \"true\""
			 ", \"entity_category\": \"diagnostic\"");
		break;
	case homeAssistantClass::homeAssistantClassDuration:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"duration\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"s\""
			 ", \"entity_category\": \"diagnostic\"");
		break;
	case homeAssistantClass::homeAssistantClassBox:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"mode\": \"box\"");
		break;
	case homeAssistantClass::homeAssistantClassInfo:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"entity_category\": \"diagnostic\"");
		break;
	case homeAssistantClass::homeAssistantClassSelect:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"enum\""
//		 ", \"force_update\": \"true\""
			);
		break;
	case homeAssistantClass::homeAssistantClassNumber:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"entity_category\": \"diagnostic\""
			 ", \"entity_type\": \"number\"");
		break;
	default:
		strcpy(stateAddition, "");
		break;
	}
	if (strlen(stateAddition) != 0) {
		resultAddedToPayload = addToPayload(stateAddition);
		if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
			return resultAddedToPayload;
		}
	}

	switch (singleEntity->entityId) {
	case mqttEntityId::entityRegNum:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"entity_category\": \"diagnostic\""
			 ", \"icon\": \"mdi:pound\""
			 ", \"min\": -1, \"max\": 41000");
		break;
	case mqttEntityId::entityRegValue:
		sprintf(stateAddition, ", \"icon\": \"mdi:folder-pound-outline\"");
		break;
	case mqttEntityId::entityGridReg:
		sprintf(stateAddition, ", \"icon\": \"mdi:security\"");
		break;
	case mqttEntityId::entityPvPwr:
		sprintf(stateAddition, ", \"icon\": \"mdi:solar-power\"");
		break;
	case mqttEntityId::entityPvEnergy:
		sprintf(stateAddition, ", \"icon\": \"mdi:solar-power-variant-outline\"");
		break;
	case mqttEntityId::entityGridPwr:
		sprintf(stateAddition, ", \"icon\": \"mdi:transmission-tower\"");
		break;
	case mqttEntityId::entityGridEnergyTo:
		sprintf(stateAddition, ", \"icon\": \"mdi:transmission-tower-export\"");
		break;
	case mqttEntityId::entityGridEnergyFrom:
		sprintf(stateAddition, ", \"icon\": \"mdi:transmission-tower-import\"");
		break;
	case mqttEntityId::entityBatPwr:
		sprintf(stateAddition, ", \"icon\": \"mdi:battery-charging-100\"");
		break;
	case mqttEntityId::entityBatEnergyCharge:
		sprintf(stateAddition, ", \"icon\": \"mdi:battery-plus\"");
		break;
	case mqttEntityId::entityBatEnergyDischarge:
		sprintf(stateAddition, ", \"icon\": \"mdi:battery-minus\"");
		break;
	case mqttEntityId::entityBatCap:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"energy\""
			 ", \"state_class\": \"total_increasing\""
			 ", \"unit_of_measurement\": \"kWh\""
			 ", \"icon\": \"mdi:home-battery\"");
		break;
	case mqttEntityId::entityOpMode:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"options\": [ \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\" ]"
#ifdef HA_IS_OP_MODE_AUTHORITY
			 ", \"retain\": \"true\""
#endif // HA_IS_OP_MODE_AUTHORITY
			 , OP_MODE_DESC_LOAD_FOLLOW, OP_MODE_DESC_TARGET, OP_MODE_DESC_PUSH,
			 OP_MODE_DESC_PV_CHARGE, OP_MODE_DESC_MAX_CHARGE, OP_MODE_DESC_NO_CHARGE);
		break;
	case mqttEntityId::entitySocTarget:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"battery\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"%%\""
			 ", \"icon\": \"mdi:battery\""
			 ", \"min\": %d, \"max\": %d"
#ifdef HA_IS_OP_MODE_AUTHORITY
			 ", \"retain\": \"true\""
#endif // HA_IS_OP_MODE_AUTHORITY
			 ,
			 SOC_TARGET_MIN, SOC_TARGET_MAX);
		break;
	case mqttEntityId::entityChargePwr:
	case mqttEntityId::entityDischargePwr:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"power\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"W\""
			 ", \"icon\": \"mdi:lightning-bolt-circle\""
			 ", \"min\": %d, \"max\": %d"
#ifdef HA_IS_OP_MODE_AUTHORITY
			 ", \"retain\": \"true\""
#endif // HA_IS_OP_MODE_AUTHORITY
			 ,
			 0, INVERTER_POWER_MAX);
		break;
	case mqttEntityId::entityPushPwr:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"power\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"W\""
			 ", \"icon\": \"mdi:lightning-bolt-circle\""
			 ", \"min\": %d, \"max\": %d"
			 ", \"retain\": \"true\"",
			 0, INVERTER_POWER_MAX);
		break;
#ifdef DEBUG_WIFI
	case mqttEntityId::entityRSSI:
	case mqttEntityId::entityBSSID:
	case mqttEntityId::entityTxPower:
	case mqttEntityId::entityWifiRecon:
		sprintf(stateAddition, ", \"icon\": \"mdi:wifi\"");
		break;
#endif // DEBUG_WIFI
	case mqttEntityId::entityA2MVersion:
	case mqttEntityId::entityInverterVersion:
	case mqttEntityId::entityEmsVersion:
		sprintf(stateAddition, ", \"icon\": \"mdi:numeric\"");
		break;
	case mqttEntityId::entityInverterSn:
	case mqttEntityId::entityEmsSn:
		sprintf(stateAddition, ", \"icon\": \"mdi:identifier\"");
		break;
#ifdef DEBUG_RS485
	case mqttEntityId::entityRs485Errors:
	case mqttEntityId::entityRs485InvalidVals:
#endif // DEBUG_RS485
	case mqttEntityId::entityBatFaults:
	case mqttEntityId::entityBatWarnings:
	case mqttEntityId::entityInverterFaults:
	case mqttEntityId::entityInverterWarnings:
	case mqttEntityId::entitySystemFaults:
		sprintf(stateAddition, ", \"icon\": \"mdi:alert-decagram-outline\"");
		break;
#ifdef DEBUG_FREEMEM
	case mqttEntityId::entityFreemem:
		sprintf(stateAddition, ", \"icon\": \"mdi:memory\"");
		break;
#endif // DEBUG_FREEMEM
#ifdef DEBUG_CALLBACKS
	case mqttEntityId::entityCallbacks:
#endif // DEBUG_CALLBACKS
	case mqttEntityId::entityA2MUptime:
	case mqttEntityId::entityBatSoc:
	case mqttEntityId::entityBatTemp:
	case mqttEntityId::entityInverterTemp:
	case mqttEntityId::entityGridAvail:
		strcpy(stateAddition, "");
		break;
	}
	if (strlen(stateAddition) != 0) {
		resultAddedToPayload = addToPayload(stateAddition);
		if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
			return resultAddedToPayload;
		}
	}

	if (singleEntity->subscribe) {
		sprintf(stateAddition, ", \"qos\": %d", MQTT_SUBSCRIBE_QOS);
		resultAddedToPayload = addToPayload(stateAddition);
		if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
			return resultAddedToPayload;
		}
	}

	sprintf(stateAddition, ", \"state_topic\": \"" DEVICE_NAME "/%s/%s/state\"",
		haUniqueId, singleEntity->mqttName);
	resultAddedToPayload = addToPayload(stateAddition);
	if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
		return resultAddedToPayload;
	}

	if (singleEntity->subscribe) {
		sprintf(stateAddition, ", \"command_topic\": \"" DEVICE_NAME "/%s/%s/command\"",
			haUniqueId, singleEntity->mqttName);
		resultAddedToPayload = addToPayload(stateAddition);
		if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
			return resultAddedToPayload;
		}
	}

	sprintf(stateAddition, ", \"availability_topic\": \"" DEVICE_NAME "/%s/status\"", haUniqueId);
	resultAddedToPayload = addToPayload(stateAddition);
	if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
		return resultAddedToPayload;
	}

// DAVE - make json_attributes be optional
#if 0
	sprintf(stateAddition, ", \"json_attributes_topic\": \"" DEVICE_NAME "/%s/%s/attributes\"",
		haUniqueId, singleEntity->mqttName);
	resultAddedToPayload = addToPayload(stateAddition);
	if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
		return resultAddedToPayload;
	}
#endif

	strcpy(stateAddition, "}");
	resultAddedToPayload = addToPayload(stateAddition);
	if (resultAddedToPayload == modbusRequestAndResponseStatusValues::payloadExceededCapacity) {
		return resultAddedToPayload;
	}

	return modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
}


modbusRequestAndResponseStatusValues
addToPayload(const char* addition)
{
	int targetRequestedSize = strlen(_mqttPayload) + strlen(addition);

	// If max payload size is 2048 it is stored as (0-2047), however character 2048  (position 2047) is null terminator so 2047 chars usable usable
	if (targetRequestedSize > _maxPayloadSize - 1) {
		// Safely print using snprintf
		snprintf(_mqttPayload, _maxPayloadSize, "{\r\n    \"mqttError\": \"Length of payload exceeds %d bytes.  Length would be %d bytes.\"\r\n}",
			 _maxPayloadSize - 1, targetRequestedSize);
		return modbusRequestAndResponseStatusValues::payloadExceededCapacity;
	} else {
		strlcat(_mqttPayload, addition, _maxPayloadSize);
		return modbusRequestAndResponseStatusValues::addedToPayload;
	}
}


void
sendHaData()
{
	int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);

	for (int i = 0; i < numberOfEntities; i++) {
		sendDataFromMqttState(&_mqttAllEntities[i], true);
	}
	resendHaData = false;
}

/*
 * readEssOpData
 *
 * Read data from ESS and A2M
 */
bool
readEssOpData()
{
#ifdef DEBUG_NO_RS485
	opData.essDispatchStart = DISPATCH_START_START;
	opData.essDispatchMode = DISPATCH_MODE_NORMAL_MODE;
	opData.essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET;
	opData.essDispatchSoc = 50 / DISPATCH_SOC_MULTIPLIER;
	opData.essBatterySoc = 65 / BATTERY_SOC_MULTIPLIER;
	opData.essBatteryPower = -1357;
//	opData.essBatteryPowerAvg = -1357;
	opData.essGridPower = -1368;
//	opData.essGridPowerAvg = -1368;
	opData.essPvPower = -1379;
	return true;
#else // DEBUG_NO_RS485
	static unsigned long lastRun = 0;
	modbusRequestAndResponseStatusValues result = modbusRequestAndResponseStatusValues::preProcessing;
	modbusRequestAndResponse response;

	if (!checkTimer(&lastRun, STATUS_INTERVAL_TEN_SECONDS)) {
		// If less than interval, then return false so nothing gets read or written.
		return false;
	}

	result = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_START, &response);
	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		opData.essDispatchStart = response.unsignedShortValue;
		result = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_MODE, &response);
	}
	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		opData.essDispatchMode = response.unsignedShortValue;
		result = _registerHandler->readHandledRegister(REG_DISPATCH_RW_ACTIVE_POWER_1, &response);
	}
	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		opData.essDispatchActivePower = response.signedIntValue;
		result = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_SOC, &response);
	}
	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		opData.essDispatchSoc = response.unsignedShortValue;
		result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_SOC, &response);
	}
	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		opData.essBatterySoc = response.unsignedShortValue;
		result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_POWER, &response);
	}
	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		opData.essBatteryPower = response.signedShortValue;
		result = _registerHandler->readHandledRegister(REG_GRID_METER_R_TOTAL_ACTIVE_POWER_1, &response);
	}
	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		opData.essGridPower = response.signedIntValue;
		result = _registerHandler->readHandledRegister(REG_CUSTOM_TOTAL_SOLAR_POWER, &response);
	}
	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		opData.essPvPower = response.signedIntValue;
	}

	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
//		static int16_t  essBatteryPowerAvgArray[OP_DATA_AVG_CNT] = {0};
//		static int32_t  essGridPowerAvgArray[OP_DATA_AVG_CNT] = {0};
//		static uint32_t avgIndex = 0;
//		uint32_t cnt;

//		essBatteryPowerAvgArray[avgIndex % OP_DATA_AVG_CNT] = opData.essBatteryPower;
//		essGridPowerAvgArray[avgIndex % OP_DATA_AVG_CNT] = opData.essGridPower;
//		avgIndex++;
//		if (avgIndex > OP_DATA_AVG_CNT) {
//			cnt = OP_DATA_AVG_CNT;
//		} else {
//			cnt = avgIndex;
//		}
//		opData.essBatteryPowerAvg = 0;
//		opData.essGridPowerAvg = 0;
//		for (int i = 0; i < cnt; i++) {
//			opData.essBatteryPowerAvg += essBatteryPowerAvgArray[i];
//			opData.essGridPowerAvg += essGridPowerAvgArray[i];
//		}
//		opData.essBatteryPowerAvg /= cnt;
//		opData.essGridPowerAvg /= cnt;
	} else {
#ifdef DEBUG_RS485
		rs485Errors++;
#endif // DEBUG_RS485
		return false;
	}
	return true;
#endif // DEBUG_NO_RS485
}

/*
  sendData

  Runs once every loop, checks to see if time periods have elapsed to allow the schedules to run.
  Each time, the appropriate arrays are iterated, processed and added to the payload.
*/
void
sendData()
{
	static unsigned long lastRunTenSeconds = 0;
	static unsigned long lastRunOneMinute = 0;
	static unsigned long lastRunFiveMinutes = 0;
	static unsigned long lastRunOneHour = 0;
	static unsigned long lastRunOneDay = 0;
	int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);

	if (resendAllData) {
		resendAllData = false;
		lastRunTenSeconds = lastRunOneMinute = lastRunFiveMinutes = lastRunOneHour = lastRunOneDay = 0;
	}

	// Update all parameters and send to MQTT.
	if (checkTimer(&lastRunTenSeconds, STATUS_INTERVAL_TEN_SECONDS)) {
		sendStatus();
		for (int i = 0; i < numberOfEntities; i++) {
			if (_mqttAllEntities[i].updateFreq == updateFreqTenSec) {
				sendDataFromMqttState(&_mqttAllEntities[i], false);
			}
		}
	}

	if (checkTimer(&lastRunOneMinute, STATUS_INTERVAL_ONE_MINUTE)) {
		for (int i = 0; i < numberOfEntities; i++) {
			if (_mqttAllEntities[i].updateFreq == updateFreqOneMin) {
				sendDataFromMqttState(&_mqttAllEntities[i], false);
			}
		}
		sendHaData();
	}

	if (checkTimer(&lastRunFiveMinutes, STATUS_INTERVAL_FIVE_MINUTE)) {
		for (int i = 0; i < numberOfEntities; i++) {
			if (_mqttAllEntities[i].updateFreq == updateFreqFiveMin) {
				sendDataFromMqttState(&_mqttAllEntities[i], false);
			}
		}
	}

	if (checkTimer(&lastRunOneHour, STATUS_INTERVAL_ONE_HOUR)) {
		for (int i = 0; i < numberOfEntities; i++) {
			if (_mqttAllEntities[i].updateFreq == updateFreqOneHour) {
				sendDataFromMqttState(&_mqttAllEntities[i], false);
			}
		}
	}

	if (checkTimer(&lastRunOneDay, STATUS_INTERVAL_ONE_DAY)) {
		for (int i = 0; i < numberOfEntities; i++) {
			if (_mqttAllEntities[i].updateFreq == updateFreqOneDay) {
				sendDataFromMqttState(&_mqttAllEntities[i], false);
			}
		}
	}
}

void
sendDataFromMqttState(mqttState *singleEntity, bool doHomeAssistant)
{
	char topic[256];
	modbusRequestAndResponseStatusValues result;
	modbusRequestAndResponseStatusValues resultAddedToPayload;

	if (singleEntity == NULL)
		return;

	emptyPayload();

	if (doHomeAssistant) {
		const char *entityType;
		switch (singleEntity->haClass) {
		case homeAssistantClass::homeAssistantClassBox:
			entityType = "number";
			break;
		case homeAssistantClass::homeAssistantClassSelect:
			entityType = "select";
			break;
		case homeAssistantClass::homeAssistantClassBinaryProblem:
			entityType = "binary_sensor";
			break;
		default:
			entityType = "sensor";
			break;
		}

		snprintf(topic, sizeof(topic), "homeassistant/%s/%s/%s/config", entityType, haUniqueId, singleEntity->mqttName);
		result = addConfig(singleEntity, resultAddedToPayload);
	} else {
		bool skip = false;
		if (!opData.a2mReadyToUseOpMode && (singleEntity->entityId == mqttEntityId::entityOpMode)) {
			skip = true;
		}
		if (!opData.a2mReadyToUseSocTarget && (singleEntity->entityId == mqttEntityId::entitySocTarget)) {
			skip = true;
		}
		if (!opData.a2mReadyToUsePwrCharge && (singleEntity->entityId == mqttEntityId::entityChargePwr)) {
			skip = true;
		}
		if (!opData.a2mReadyToUsePwrDischarge && (singleEntity->entityId == mqttEntityId::entityDischargePwr)) {
			skip = true;
		}
		if (!opData.a2mReadyToUsePwrPush && (singleEntity->entityId == mqttEntityId::entityPushPwr)) {
			skip = true;
		}
		if (!skip) {
			snprintf(topic, sizeof(topic), DEVICE_NAME "/%s/%s/state", haUniqueId, singleEntity->mqttName);
			result = addState(singleEntity, &resultAddedToPayload);
		} else {
			result = modbusRequestAndResponseStatusValues::preProcessing;
		}
	}

	if ((resultAddedToPayload != modbusRequestAndResponseStatusValues::payloadExceededCapacity) &&
	    (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)) {
		// And send
		bool retain = getRetain(singleEntity->entityId);
		sendMqtt(topic, retain);
	}
}


/*
  mqttCallback()

// This function is executed when an MQTT message arrives on a topic that we are subscribed to.
*/
void mqttCallback(char* topic, byte* message, unsigned int length)
{
	char mqttIncomingPayload[64] = ""; // Should be enough to cover command requests
	mqttState *mqttEntity = NULL;

#ifdef DEBUG
	sprintf(_debugOutput, "Topic: %s", topic);
	Serial.println(_debugOutput);
#endif

#ifdef DEBUG_CALLBACKS
	receivedCallbacks++;
#endif // DEBUG_CALLBACKS

	if ((length == 0) || (length >= sizeof(mqttIncomingPayload))) {
#ifdef DEBUG
		sprintf(_debugOutput, "mqttCallback: bad length: %d", length);
		Serial.println(_debugOutput);
#endif
#ifdef DEBUG_CALLBACKS
		badCallbacks++;
#endif // DEBUG_CALLBACKS
		return; // We won't be doing anything
	} else {
		// Get the payload (ensure NULL termination)
		strlcpy(mqttIncomingPayload, (char *)message, length + 1);
	}
#ifdef DEBUG
	sprintf(_debugOutput, "Payload: %d", length);
	Serial.println(_debugOutput);
	Serial.println(mqttIncomingPayload);
#endif

	// Special case for Home Assistant itself
	if (strcmp(topic, MQTT_SUB_HOMEASSISTANT) == 0) {
		if (strcmp(mqttIncomingPayload, "online") == 0) {
			resendHaData = true;
			resendAllData = true;
		} else {
#ifdef DEBUG
			Serial.println("Unknown homeassistant/status: ");
			Serial.println(mqttIncomingPayload);
#endif
		}
		return; // No further processing needed.
	} else {
		// match to DEVICE_NAME "/SERIAL#/MQTT_NAME/command"
		char matchPrefix[64];

		snprintf(matchPrefix, sizeof(matchPrefix), DEVICE_NAME "/%s/", haUniqueId);
		if (!strncmp(topic, matchPrefix, strlen(matchPrefix)) &&
		    !strcmp(&topic[strlen(topic) - strlen("/command")], "/command")) {
			char topicEntityName[64];
			int topicEntityLen = strlen(topic) - strlen(matchPrefix) - strlen("/command");
			if (topicEntityLen < sizeof(topicEntityName)) {
				strlcpy(topicEntityName, &topic[strlen(matchPrefix)], topicEntityLen + 1);
				mqttEntity = lookupSubscription(topicEntityName);
			}
		}
		if (mqttEntity == NULL) {
#ifdef DEBUG_CALLBACKS
			unknownCallbacks++;
#endif // DEBUG_CALLBACKS
			return; // No further processing possible.
		}
	}

	// Update system!!!
	{
		int32_t singleInt32 = -1;
		char *singleString;
		char *endPtr = NULL;
		mqttState *relatedMqttEntity;
		bool valueProcessingError = false;

		// First, process value.
		switch (mqttEntity->entityId) {
		case mqttEntityId::entitySocTarget:
		case mqttEntityId::entityChargePwr:
		case mqttEntityId::entityDischargePwr:
		case mqttEntityId::entityPushPwr:
		case mqttEntityId::entityRegNum:
			singleInt32 = strtol(mqttIncomingPayload, &endPtr, 10);
			if ((endPtr == mqttIncomingPayload) || ((singleInt32 == 0) && (errno != 0))) {
				valueProcessingError = true;
			}
			break;
		case mqttEntityId::entityOpMode:
			singleString = mqttIncomingPayload;
			break;
		default:
#ifdef DEBUG
			sprintf(_debugOutput, "Trying to update an unhandled entity! %d", mqttEntity->entityId);
			Serial.println(_debugOutput);
#endif
#ifdef DEBUG_CALLBACKS
			unknownCallbacks++;
#endif // DEBUG_CALLBACKS
			return; // No further processing possible.
		}

		if (valueProcessingError) {
#ifdef DEBUG
			snprintf(_debugOutput, sizeof(_debugOutput), "Callback for %s with bad value: ", mqttEntity->mqttName);
			Serial.print(_debugOutput);
			Serial.println(mqttIncomingPayload);
#endif
#ifdef DEBUG_CALLBACKS
			badCallbacks++;
#endif // DEBUG_CALLBACKS
		} else {
			// Now set the value and take appropriate action(s)
			switch (mqttEntity->entityId) {
			case mqttEntityId::entitySocTarget:
				if ((singleInt32 < SOC_TARGET_MIN) || (singleInt32 > SOC_TARGET_MAX)) {
#ifdef DEBUG
					sprintf(_debugOutput, "HA sent invalid SocTarget! %ld", singleInt32);
					Serial.println(_debugOutput);
#endif
#ifdef DEBUG_CALLBACKS
					badCallbacks++;
#endif // DEBUG_CALLBACKS
				} else {
					opData.a2mSocTarget = singleInt32;
					opData.a2mReadyToUseSocTarget = true;
				}
				break;
			case mqttEntityId::entityChargePwr:
				if ((singleInt32 < 0) || (singleInt32 > INVERTER_POWER_MAX)) {
#ifdef DEBUG
					sprintf(_debugOutput, "HA sent invalid Charge Power! %ld", singleInt32);
					Serial.println(_debugOutput);
#endif
#ifdef DEBUG_CALLBACKS
					badCallbacks++;
#endif // DEBUG_CALLBACKS
				} else {
					opData.a2mPwrCharge = singleInt32;
					opData.a2mReadyToUsePwrCharge = true;
				}
				break;
			case mqttEntityId::entityDischargePwr:
				if ((singleInt32 < 0) || (singleInt32 > INVERTER_POWER_MAX)) {
#ifdef DEBUG
					sprintf(_debugOutput, "HA sent invalid Discharge Power! %ld", singleInt32);
					Serial.println(_debugOutput);
#endif
#ifdef DEBUG_CALLBACKS
					badCallbacks++;
#endif // DEBUG_CALLBACKS
				} else {
					opData.a2mPwrDischarge = singleInt32;
					opData.a2mReadyToUsePwrDischarge = true;
				}
				break;
			case mqttEntityId::entityPushPwr:
				if ((singleInt32 < 0) || (singleInt32 > INVERTER_POWER_MAX)) {
#ifdef DEBUG
					sprintf(_debugOutput, "HA sent invalid Push Power! %ld", singleInt32);
					Serial.println(_debugOutput);
#endif
#ifdef DEBUG_CALLBACKS
					badCallbacks++;
#endif // DEBUG_CALLBACKS
				} else {
					opData.a2mPwrPush = singleInt32;
					opData.a2mReadyToUsePwrPush = true;
				}
				break;
			case mqttEntityId::entityRegNum:
				regNumberToRead = singleInt32;    // Set local variable
				relatedMqttEntity = lookupEntity(mqttEntityId::entityRegValue);
				sendDataFromMqttState(relatedMqttEntity, false);    // Send update for related entity
				break;
			case mqttEntityId::entityOpMode:
				{
					enum opMode tempOpMode = lookupOpMode(singleString);
					if (tempOpMode != (enum opMode)-1) {
						opData.a2mOpMode = tempOpMode;
						opData.a2mReadyToUseOpMode = true;
					} else {
#ifdef DEBUG
						snprintf(_debugOutput, sizeof(_debugOutput), "Callback: Bad opMode: %s", singleString);
						Serial.println(_debugOutput);
#endif
#ifdef DEBUG_CALLBACKS
						badCallbacks++;
#endif // DEBUG_CALLBACKS
					}
				}
				break;
			default:
#ifdef DEBUG
				sprintf(_debugOutput, "Trying to write an unhandled entity! %d", mqttEntity->entityId);
				Serial.println(_debugOutput);
#endif
				break;
			}
		}
	}

	// Send (hopefully) updated state.  If we failed to update, sender should notice value not changing.
	sendDataFromMqttState(mqttEntity, false);

	return;
}


/*
 * sendMqtt
 *
 * Sends whatever is in the modular level payload to the specified topic.
 */
void sendMqtt(const char *topic, bool retain)
{
	// Attempt a send
	if (!_mqtt.publish(topic, _mqttPayload, retain)) {
#ifdef DEBUG
		snprintf(_debugOutput, sizeof(_debugOutput), "MQTT publish failed to %s", topic);
		Serial.println(_debugOutput);
		Serial.println(_mqttPayload);
#endif
	} else {
#ifdef DEBUG
		//sprintf(_debugOutput, "MQTT publish success");
		//Serial.println(_debugOutput);
#endif
	}

	// Empty payload for next use.
	emptyPayload();
	return;
}

/*
 * emptyPayload
 *
 * Clears every char so end of string can be easily found
 */
void emptyPayload()
{
	// DAVE - can we just set first byte to NULL
	memset(_mqttPayload, 0, _maxPayloadSize);
}

void
getOpModeDesc(char *dest, size_t size, enum opMode mode)
{
	snprintf(dest, size, "Unknown %u", mode);
	switch (mode) {
	case opMode::opModePvCharge:
		strlcpy(dest, OP_MODE_DESC_PV_CHARGE, size);
		break;
	case opMode::opModeTarget:
		strlcpy(dest, OP_MODE_DESC_TARGET, size);
		break;
	case opMode::opModePush:
		strlcpy(dest, OP_MODE_DESC_PUSH, size);
		break;
	case opMode::opModeLoadFollow:
		strlcpy(dest, OP_MODE_DESC_LOAD_FOLLOW, size);
		break;
	case opMode::opModeMaxCharge:
		strlcpy(dest, OP_MODE_DESC_MAX_CHARGE, size);
		break;
	case opMode::opModeNoCharge:
		strlcpy(dest, OP_MODE_DESC_NO_CHARGE, size);
		break;
	}
}

enum opMode
lookupOpMode(char *opModeDesc)
{
	if (!strcmp(opModeDesc, OP_MODE_DESC_PV_CHARGE))
		return opMode::opModePvCharge;
	if (!strcmp(opModeDesc, OP_MODE_DESC_TARGET))
		return opMode::opModeTarget;
	if (!strcmp(opModeDesc, OP_MODE_DESC_PUSH))
		return opMode::opModePush;
	if (!strcmp(opModeDesc, OP_MODE_DESC_LOAD_FOLLOW))
		return opMode::opModeLoadFollow;
	if (!strcmp(opModeDesc, OP_MODE_DESC_MAX_CHARGE))
		return opMode::opModeMaxCharge;
	if (!strcmp(opModeDesc, OP_MODE_DESC_NO_CHARGE))
		return opMode::opModeNoCharge;
	return (enum opMode)-1;  // Shouldn't happen
}

void
setEssOpMode(void)
{
#ifndef DEBUG_NO_RS485
	modbusRequestAndResponseStatusValues result = modbusRequestAndResponseStatusValues::preProcessing;
	modbusRequestAndResponse response;
	uint16_t essDispatchMode, essBatterySocPct, essDispatchSoc;
	int32_t essDispatchActivePower;

#ifdef DEBUG_OPS
	opCounter++;
#endif

	essBatterySocPct = opData.essBatterySoc * BATTERY_SOC_MULTIPLIER;
	if (opData.a2mSocTarget == 100) {
		essDispatchSoc = 252;  // (100/DISPATCH_SOC_MULTIPLIER) = 250 but we want it a smidge higher
		// and leave power charging.  Let Alpha stop it when ready.
		essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET - opData.a2mPwrCharge;
	} else {
		essDispatchSoc = opData.a2mSocTarget / DISPATCH_SOC_MULTIPLIER;
		if (essBatterySocPct == opData.a2mSocTarget) {
			essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET;
		} else if (essBatterySocPct > opData.a2mSocTarget) {
			essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET + opData.a2mPwrDischarge;
		} else {
			essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET - opData.a2mPwrCharge;
		}
	}

	switch (opData.a2mOpMode) {
	case opMode::opModePvCharge:		// Honors Power and SOC
		essDispatchMode = DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV;	// Honors Power but not SOC
		// use essDispatchActivePower from above
		break;
	case opMode::opModeTarget:		// Honors Power and SOC
		essDispatchMode = DISPATCH_MODE_STATE_OF_CHARGE_CONTROL;	// Honors Power and SOC
		// use essDispatchActivePower from above
		break;
	case opMode::opModePush:		// Honors PushPwr and SOC
		if (essBatterySocPct > opData.a2mSocTarget) {
			int32_t newBatteryPower = opData.essBatteryPower + opData.essGridPower + opData.a2mPwrPush;
			if (newBatteryPower < opData.a2mPwrPush) {
				newBatteryPower = opData.a2mPwrPush;
			}
			if (newBatteryPower > INVERTER_POWER_MAX) {
				newBatteryPower = INVERTER_POWER_MAX; // Should never happen, but just to be safe...
			}
			essDispatchMode = DISPATCH_MODE_STATE_OF_CHARGE_CONTROL;	// Honors Power and SOC
			essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET + newBatteryPower;
		} else {
			essDispatchMode = DISPATCH_MODE_NO_BATTERY_CHARGE;		// Doesn't honor Power or SOC
			essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET;
		}
		break;
	case opMode::opModeLoadFollow:		// Honors Power and SOC
		essDispatchMode = DISPATCH_MODE_LOAD_FOLLOWING;			// Honors Power but not SOC
		// use essDispatchActivePower from above
		break;
	case opMode::opModeMaxCharge:		// Doesn't honors Power or SOC
		essDispatchMode = DISPATCH_MODE_OPTIMISE_CONSUMPTION;		// Doesn't honor Power or SOC
		essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET - opData.a2mPwrCharge;
		break;
	case opMode::opModeNoCharge:		// Doesn't honors Power or SOC
		essDispatchMode = DISPATCH_MODE_NO_BATTERY_CHARGE;		// Doesn't honor Power or SOC
		essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET;
		break;
	default:
		return; // Shouldn't happen!  opMode is corrupt.
	}

	result = _registerHandler->writeDispatchRegisters(essDispatchActivePower, essDispatchMode, essDispatchSoc, &response);
	if (result != modbusRequestAndResponseStatusValues::writeDataRegisterSuccess) {
#ifdef DEBUG_RS485
		rs485Errors++;
#endif // DEBUG_RS485
	}
#endif // ! DEBUG_NO_RS485
}

bool
checkEssOpMode(void)
{
#ifndef DEBUG_NO_RS485
	uint16_t essDispatchMode, essBatterySocPct, essDispatchSoc;
	int32_t essDispatchActivePower;

	if (!opData.a2mReadyToUseOpMode || !opData.a2mReadyToUseSocTarget || !opData.a2mReadyToUsePwrCharge || !opData.a2mReadyToUsePwrDischarge || !opData.a2mReadyToUsePwrPush) {
		return true;  // Don't set anything if opData isn't ready.
	}

	if (opData.essDispatchStart != DISPATCH_START_START) {
		return false;
	}

	essBatterySocPct = opData.essBatterySoc * BATTERY_SOC_MULTIPLIER;
	if (opData.a2mSocTarget == 100) {
		essDispatchSoc = 252;  // (100/DISPATCH_SOC_MULTIPLIER) = 250 but we want it a smidge higher
		// and leave power charging.  Let Alpha stop it when ready.
		essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET - opData.a2mPwrCharge;
	} else {
		essDispatchSoc = opData.a2mSocTarget / DISPATCH_SOC_MULTIPLIER;
		if (essBatterySocPct == opData.a2mSocTarget) {
			essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET;
		} else if (essBatterySocPct > opData.a2mSocTarget) {
			essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET + opData.a2mPwrDischarge;
		} else {
			essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET - opData.a2mPwrCharge;
		}
	}

	switch (opData.a2mOpMode) {
	case opMode::opModePvCharge:
		essDispatchMode = DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV;
		// use essDispatchActivePower from above
		break;
	case opMode::opModeTarget:
		essDispatchMode = DISPATCH_MODE_STATE_OF_CHARGE_CONTROL;
		// use essDispatchActivePower from above
		break;
	case opMode::opModePush:
		if (essBatterySocPct > opData.a2mSocTarget) {
			int32_t newBatteryPower = opData.essBatteryPower + opData.essGridPower + opData.a2mPwrPush;
			if (newBatteryPower < opData.a2mPwrPush) {
				newBatteryPower = opData.a2mPwrPush;
			}
			if (newBatteryPower > INVERTER_POWER_MAX) {
				newBatteryPower = INVERTER_POWER_MAX; // Should never happen, but just to be safe...
			}
			essDispatchMode = DISPATCH_MODE_STATE_OF_CHARGE_CONTROL;	// Honors Power and SOC
			essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET + newBatteryPower;
			// Smoothing - If power doesn't change much, pretend it isn't changing.
			if ((essDispatchActivePower < (opData.essDispatchActivePower + PUSH_FUDGE_FACTOR)) &&
			    (essDispatchActivePower > (opData.essDispatchActivePower - PUSH_FUDGE_FACTOR))) {
				essDispatchActivePower = opData.essDispatchActivePower;
			}
		} else {
			essDispatchMode = DISPATCH_MODE_NO_BATTERY_CHARGE;		// Doesn't honor Power or SOC
			essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET;
		}
		break;
	case opMode::opModeLoadFollow:
		essDispatchMode = DISPATCH_MODE_LOAD_FOLLOWING;
		// use essDispatchActivePower from above
		break;
	case opMode::opModeMaxCharge:
		essDispatchMode = DISPATCH_MODE_OPTIMISE_CONSUMPTION;
		essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET - opData.a2mPwrCharge;
		break;
	case opMode::opModeNoCharge:
		essDispatchMode = DISPATCH_MODE_NO_BATTERY_CHARGE;
		essDispatchActivePower = DISPATCH_ACTIVE_POWER_OFFSET;
		break;
	default:
		return false;  // Shouldn't happen!  opMode is corrupt.
	}

	if (opData.essDispatchMode != essDispatchMode) {
		return false;
	}

	if (opData.essDispatchActivePower != essDispatchActivePower) {
		return false;
	}

	if (opData.essDispatchSoc != essDispatchSoc) {
		return false;
	}

	// No need to check REG_DISPATCH_RW_DISPATCH_TIME_1

#endif // ! DEBUG_NO_RS485
	return true;
}

void
getA2mOpDataFromEss(void)
{
#ifdef DEBUG_NO_RS485
	opData.a2mOpMode = opMode::opModeNoCharge;
	opData.a2mSocTarget = SOC_TARGET_MAX;
	opData.a2mPwrCharge = INVERTER_POWER_MAX;
	opData.a2mPwrDischarge = INVERTER_POWER_MAX;
#else // DEBUG_NO_RS485
	modbusRequestAndResponseStatusValues result;
	modbusRequestAndResponse response;
	bool found;

	found = false;
	while (!found) {
		result = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_MODE, &response);
		if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			switch (response.unsignedShortValue) {
			case DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV:
				opData.a2mOpMode = opMode::opModePvCharge;
				break;
			case DISPATCH_MODE_STATE_OF_CHARGE_CONTROL:
				opData.a2mOpMode = opMode::opModeTarget;
				break;
			case DISPATCH_MODE_LOAD_FOLLOWING:
			case DISPATCH_MODE_NORMAL_MODE:
				opData.a2mOpMode = opMode::opModeLoadFollow;
				break;
			case DISPATCH_MODE_OPTIMISE_CONSUMPTION:
			case DISPATCH_MODE_MAXIMISE_OUTPUT:
			case DISPATCH_MODE_MAXIMISE_CONSUMPTION:
				opData.a2mOpMode = opMode::opModeMaxCharge;
				break;
			case DISPATCH_MODE_NO_BATTERY_CHARGE:
				opData.a2mOpMode = opMode::opModeNoCharge;
				break;
			default:
#ifdef DEBUG
				snprintf(_debugOutput, sizeof(_debugOutput), "getA2mOpDataFromEss: Unhandled Dispatch Mode: %u/", response.unsignedShortValue);
				Serial.print(_debugOutput);
				Serial.println(response.dataValueFormatted);
#endif
				opData.a2mOpMode = opMode::opModeLoadFollow; // Just set to a "default" value.
				break;
			}
			found = true;
		} else {
#ifdef DEBUG_RS485
			rs485Errors++;
#endif // DEBUG_RS485
#ifdef DEBUG
			snprintf(_debugOutput, sizeof(_debugOutput), "getA2mOpDataFromEss: read failed");
			Serial.println(_debugOutput);
		}
#endif
	}

	found = false;
	while (!found) {
		result = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_SOC, &response);
		if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			opData.a2mSocTarget = response.unsignedShortValue * DISPATCH_SOC_MULTIPLIER;
			found = true;
		} else {
#ifdef DEBUG
			snprintf(_debugOutput, sizeof(_debugOutput), "getA2mOpDataFromEss: read failed");
			Serial.println(_debugOutput);
#endif // DEBUG
#ifdef DEBUG_RS485
			rs485Errors++;
#endif // DEBUG_RS485
		}
	}

	found = false;
	while (!found) {
		result = _registerHandler->readHandledRegister(REG_DISPATCH_RW_ACTIVE_POWER_1, &response);
		if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			if (response.signedIntValue > DISPATCH_ACTIVE_POWER_OFFSET) {
				opData.a2mPwrCharge = INVERTER_POWER_MAX;
				opData.a2mPwrDischarge = response.signedIntValue - DISPATCH_ACTIVE_POWER_OFFSET;
			} else if (response.signedIntValue < DISPATCH_ACTIVE_POWER_OFFSET) {
				opData.a2mPwrCharge = DISPATCH_ACTIVE_POWER_OFFSET - response.signedIntValue;
				opData.a2mPwrDischarge = INVERTER_POWER_MAX;
			} else {
				opData.a2mPwrCharge = INVERTER_POWER_MAX;
				opData.a2mPwrDischarge = INVERTER_POWER_MAX;
			}
			found = true;
		} else {
#ifdef DEBUG
			snprintf(_debugOutput, sizeof(_debugOutput), "getA2mOpDataFromEss: read failed");
			Serial.println(_debugOutput);
#endif // DEBUG
#ifdef DEBUG_RS485
			rs485Errors++;
#endif // DEBUG_RS485
		}
	}
#endif // DEBUG_NO_RS485
}

bool
getRetain(enum mqttEntityId entityId)
{
	switch(entityId) {
#ifdef DEBUG_FREEMEM
	case entityFreemem:
#endif // DEBUG_FREEMEM
#ifdef DEBUG_CALLBACKS
	case entityCallbacks:
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_WIFI
	case entityRSSI:
	case entityBSSID:
	case entityTxPower:
	case entityWifiRecon:
#endif // DEBUG_WIFI
#ifdef DEBUG_RS485
	case entityRs485Errors:
	case entityRs485InvalidVals:
#endif // DEBUG_RS485
	case entityA2MUptime:
		return false;
	case entityA2MVersion:
	case entityInverterVersion:
	case entityInverterSn:
	case entityEmsVersion:
	case entityEmsSn:
	case entityBatSoc:
	case entityBatPwr:
	case entityBatEnergyCharge:
	case entityBatEnergyDischarge:
	case entityGridAvail:
	case entityGridPwr:
	case entityGridEnergyTo:
	case entityGridEnergyFrom:
	case entityPvPwr:
	case entityPvEnergy:
	case entityOpMode:
	case entitySocTarget:
	case entityChargePwr:
	case entityDischargePwr:
	case entityPushPwr:
	case entityBatCap:
	case entityBatTemp:
	case entityInverterTemp:
	case entityBatFaults:
	case entityBatWarnings:
	case entityInverterFaults:
	case entityInverterWarnings:
	case entitySystemFaults:
	case entityGridReg:
	case entityRegNum:
	case entityRegValue:
		break;
	}
	return MQTT_RETAIN;
}

#ifdef DEBUG_FREEMEM
uint32_t freeMemory()
{
	return ESP.getFreeHeap();
}
#endif // DEBUG_FREEMEM
