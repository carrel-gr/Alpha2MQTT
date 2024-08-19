/*
Name:		Alpha2MQTT.ino
Created:	24/Aug/2022
Author:		Daniel Young

This file is part of Alpha2MQTT (A2M) which is released under GNU GENERAL PUBLIC LICENSE.
See file LICENSE or go to https://choosealicense.com/licenses/gpl-3.0/ for full license details.

Notes

First, go and customise options at the top of Definitions.h!
*/

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
char _version[6] = "v2.30";
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
#ifdef DEBUG_NO_RS485
int16_t dispatchMode = DISPATCH_MODE_LOAD_FOLLOWING;
char dispatchModeDesc[32] = DISPATCH_MODE_LOAD_FOLLOWING_DESC;
uint16_t socTarget = 252;
#endif // DEBUG_NO_RS485

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
	{ mqttEntityId::entityUptime,             "A2M_uptime",           mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassDuration },
	{ mqttEntityId::entityVersion,            "A2M_version",          mqttUpdateFreq::updateFreqOneDay, false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityBatSoc,             "State_of_Charge",      mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassBattery },
	{ mqttEntityId::entityBatPwr,             "ESS_Power",            mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassPower },
	{ mqttEntityId::entityBatEnergyCharge,    "ESS_Energy_Charge",    mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassEnergy },
	{ mqttEntityId::entityBatEnergyDischarge, "ESS_Energy_Discharge", mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassEnergy },
	{ mqttEntityId::entityGridAvail,          "Grid_Connected",       mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassBinaryPower },
	{ mqttEntityId::entityGridPwr,            "Grid_Power",           mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassPower },
	{ mqttEntityId::entityGridEnergyTo,       "Grid_Energy_To",       mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassEnergy },
	{ mqttEntityId::entityGridEnergyFrom,     "Grid_Energy_From",     mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassEnergy },
	{ mqttEntityId::entityPvPwr,              "Solar_Power",          mqttUpdateFreq::updateFreqTenSec,  false, homeAssistantClass::homeAssistantClassPower },
	{ mqttEntityId::entityPvEnergy,           "Solar_Energy",         mqttUpdateFreq::updateFreqOneMin,  false, homeAssistantClass::homeAssistantClassEnergy },
	{ mqttEntityId::entityDispatchMode,       "Dispatch_Mode",        mqttUpdateFreq::updateFreqTenSec,  true,  homeAssistantClass::homeAssistantClassSelect },
	{ mqttEntityId::entitySocTarget,          "SOC_Target"   ,        mqttUpdateFreq::updateFreqTenSec,  true,  homeAssistantClass::homeAssistantClassBox },
	{ mqttEntityId::entityMaxCellTemp,        "Max_Cell_Temp",        mqttUpdateFreq::updateFreqFiveMin, false, homeAssistantClass::homeAssistantClassTemp },
	{ mqttEntityId::entityBatCap,             "Battery_Capacity",     mqttUpdateFreq::updateFreqOneDay,  false, homeAssistantClass::homeAssistantClassInfo },
	{ mqttEntityId::entityInverterTemp,       "Inverter_Temp",        mqttUpdateFreq::updateFreqFiveMin, false, homeAssistantClass::homeAssistantClassTemp },
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
	sendData();
	resendAllData = true; // Tell sendData() to send everything again

	updateOLED(false, "", "", _version);
}




/*
 * loop
 *
 * The loop function runs overand over again until power down or reset
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

	// Read and transmit all entity data to MQTT
	sendData();
	
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
	// There's 20 characters we can play with, width wise.
	snprintf(line1Contents, sizeof(line1Contents), "A2M  %c%c%c  RSSI: %d",
		 _oledOperatingIndicator, (WiFi.status() == WL_CONNECTED ? 'W' : ' '), (_mqtt.connected() && _mqtt.loop() ? 'M' : ' '), WiFi.RSSI() );
#else // LARGE_DISPLAY
	// There's ten characters we can play with, width wise.
	snprintf(line1Contents, sizeof(line1Contents), "%s%c%c%c", "A2M    ",
		 _oledOperatingIndicator, (WiFi.status() == WL_CONNECTED ? 'W' : ' '), (_mqtt.connected() && _mqtt.loop() ? 'M' : ' ') );
#endif // LARGE_DISPLAY
	_display.println(line1Contents);

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
	modbusRequestAndResponse response;
#ifndef DEBUG_NO_RS485
	modbusRequestAndResponseStatusValues request;
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
		strcpy(line3, "");
#else // DEBUG_NO_RS485
		// Line 2: Get Dispatch Start - Is Alpha2MQTT controlling the inverter?
		request = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_START, &response);
		if (request != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			snprintf(line2, sizeof(line2), "DS Err: %s", response.displayMessage);
#ifdef DEBUG_RS485
			rs485Errors++;
#endif // DEBUG_RS485
		} else {
			if (response.unsignedShortValue != DISPATCH_START_START) {
				strcpy(line2, "Stopped");
			} else {
				// Get the mode.
				request = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_MODE, &response);
				if (request != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
					snprintf(line2, sizeof(line2), "Mode Err: %s", response.displayMessage);
#ifdef DEBUG_RS485
					rs485Errors++;
#endif // DEBUG_RS485
				} else {
					switch (response.unsignedShortValue) {
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
					default:
						dMode = "BadMode";
						break;
					}

					// Determine if charging or discharging by looking at power
					request = _registerHandler->readHandledRegister(REG_DISPATCH_RW_ACTIVE_POWER_1, &response);
					if (request != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
						snprintf(line2, sizeof(line2), "AP Err: %s", response.displayMessage);
#ifdef DEBUG_RS485
						rs485Errors++;
#endif // DEBUG_RS485
					} else {
						if (response.signedIntValue < 32000) {
							dAction = "Charge";
						} else if (response.signedIntValue > 32000) {
							dAction = "Discharge";
						} else {
							dAction = "Hold";
						}
						snprintf(line2, sizeof(line2), "%s : %s", dMode, dAction);
					}
				}
			}
		}
#ifdef DEBUG
		if (request != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			Serial.println(response.statusMqttMessage);
		}
#endif // DEBUG

		// Get battery info for line 3
		request = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_POWER, &response);
		if (request != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			snprintf(line3, sizeof(line3), "Bat Err: %s", response.displayMessage);
#ifdef DEBUG_RS485
			rs485Errors++;
#endif // DEBUG_RS485
		} else {
			int batPower = response.signedShortValue;

			request = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_SOC, &response);
			if (request != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				snprintf(line3, sizeof(line3), "SOC Err: %s", response.displayMessage);
#ifdef DEBUG_RS485
				rs485Errors++;
#endif // DEBUG_RS485
			} else {
				snprintf(line3, sizeof(line3), "Bat: %4dW  %0.02f%%", batPower, response.unsignedShortValue * 0.1);
			}
		}
#ifdef DEBUG
		if (request != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			Serial.println(response.statusMqttMessage);
		}
#endif // DEBUG
#endif // DEBUG_NO_RS485

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
	modbusRequestAndResponse response;
	modbusRequestAndResponseStatusValues request;

	char line2[OLED_CHARACTER_WIDTH] = "";
	char line3[OLED_CHARACTER_WIDTH] = "";
	char line4[OLED_CHARACTER_WIDTH] = "";


	if (checkTimer(&lastRun, RUNSTATE_INTERVAL)) {
		//Flash the LED
		digitalWrite(LED_BUILTIN, LOW);
		delay(4);
		digitalWrite(LED_BUILTIN, HIGH);

		// Get Dispatch Start - Is Alpha2MQTT controlling the inverter?
		request = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_START, &response);
		if (request != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			// Use line3 (line 3) for errors
			strcpy(line3, "DS Err");
#ifdef DEBUG_RS485
			rs485Errors++;
#endif // DEBUG_RS485
		} else {
			if (response.unsignedShortValue != DISPATCH_START_START) {
				strcpy(line2, "Stopped");
			} else {
				if (lastLine2 == 0) {
					lastLine2 = 1;
					// Get the mode.
					request = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_MODE, &response);
					if (request != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
						// Use line3 (line 3) for errors
						strcpy(line3, "Mode Err");
#ifdef DEBUG_RS485
						rs485Errors++;
#endif // DEBUG_RS485
					} else {
						switch (response.unsignedShortValue) {
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
						default:
							strcpy(line2, "BadMode");
							break;
						}
					}
				} else {
					lastLine2 = 0;
					// Determine if charging or discharging by looking at power
					request = _registerHandler->readHandledRegister(REG_DISPATCH_RW_ACTIVE_POWER_1, &response);
					if (request != modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
						// Use line3 (line 3) for errors
						strcpy(line3, "AP Err");
#ifdef DEBUG_RS485
						rs485Errors++;
#endif // DEBUG_RS485
					} else {
						if (response.signedIntValue < 32000) {
							strcpy(line2, "Charge");
						} else if (response.signedIntValue > 32000) {
							strcpy(line2, "Discharge");
						} else {
							strcpy(line2, "Hold");
						}
					}
				}
			}
		}

		if (lastLine2 == 1) {
			// Get battery power for line 3
			if (request == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				request = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_POWER, &response);
				if (request == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
					snprintf(line3, sizeof(line3), "Bat:%dW", response.signedShortValue);
				} else {
					// Use line3 (line 3) for errors
					strcpy(line3, "Bat Err");
#ifdef DEBUG_RS485
					rs485Errors++;
#endif // DEBUG_RS485
				}
			}

			// And percent for line 4
			if (request == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				request = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_SOC, &response);
				if (request == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
					snprintf(line4, sizeof(line4), "%0.02f%%", response.unsignedShortValue * 0.1);
				} else {
					// Use line3 (line 3) for errors
					strcpy(line3, "SOC Err");
#ifdef DEBUG_RS485
					rs485Errors++;
#endif // DEBUG_RS485
				}
			}
		} else {
			snprintf(line3, sizeof(line3), "Mem: %u", freeMemory());
#if defined MP_ESP8266
			snprintf(line4, sizeof(line4), "TX: %0.2f", wifiPower);
#else
			strcpy(line4, "");
#endif
		}

		if (request == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			updateOLED(false, line2, line3, line4);
		} else {
#ifdef DEBUG
			Serial.println(response.statusMqttMessage);
#endif
			// Use line3 (line 3) for errors
			updateOLED(false, "", line3, response.displayMessage);
		}
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
	char mqttClientName[64];
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

		strlcpy(mqttClientName, DEVICE_NAME, sizeof(mqttClientName));
#ifdef DEBUG_NO_RS485
		strlcat(mqttClientName, "-test", sizeof(mqttClientName));
#endif // DEBUG_NO_RS485

		// Attempt to connect
		if (_mqtt.connect(mqttClientName, MQTT_USERNAME, MQTT_PASSWORD, statusTopic, 0, true, "offline")) {
			int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);
#ifdef DEBUG
			Serial.println("Connected MQTT");
#endif

			// Special case for Home Assistant
			sprintf(subscriptionDef, "%s", MQTT_SUB_HOMEASSISTANT);
			subscribed = _mqtt.subscribe(subscriptionDef);
#ifdef DEBUG
			snprintf(_debugOutput, sizeof(_debugOutput), "Subscribed to \"%s\" : %d", subscriptionDef, subscribed);
			Serial.println(_debugOutput);
#endif

			for (int i = 0; i < numberOfEntities; i++) {
				if (_mqttAllEntities[i].subscribe) {
					sprintf(subscriptionDef, DEVICE_NAME "/%s/%s/command", haUniqueId, _mqttAllEntities[i].mqttName);
					subscribed = subscribed && _mqtt.subscribe(subscriptionDef);
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
			if (result == modbusRequestAndResponseStatusValues::notHandledRegister) {
				rs->returnDataType = modbusReturnDataType::character;
				strcpy(rs->characterValue, "Invalid register");
				sprintf(rs->dataValueFormatted, "%s", rs->characterValue);
				result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
			} else {
				if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
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
#ifdef DEBUG_RS485
				} else {
					rs485Errors++;
#endif // DEBUG_RS485
				}
			}
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
	case mqttEntityId::entityMaxCellTemp:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->signedShortValue = 2750;
		sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * INVERTER_TEMP_MULTIPLIER);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_MAX_CELL_TEMPERATURE, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entitySocTarget:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->unsignedShortValue = socTarget;
		sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.4);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_SOC, rs);
#endif // DEBUG_NO_RS485
		{
			int socPercent = (int)(rs->unsignedShortValue * 0.4);
			// HA needs an int.  Re-format
			sprintf(rs->dataValueFormatted, "%d", socPercent);
			// De-bounce
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
				if ((socPercent < SOC_TARGET_MIN) || (socPercent > SOC_TARGET_MAX)) {
					result = modbusRequestAndResponseStatusValues::readDataInvalidValue;
				}
			}
		}
		break;
	case mqttEntityId::entityDispatchMode:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->unsignedShortValue = dispatchMode;
		strcpy(rs->dataValueFormatted, dispatchModeDesc);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_START, rs);
		if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			if (rs->unsignedShortValue == DISPATCH_START_START) {
				result = _registerHandler->readHandledRegister(REG_DISPATCH_RW_DISPATCH_MODE, rs);
			}
		}
#endif // DEBUG_NO_RS485
		// De-bounce
		if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
			if (!strncmp(rs->dataValueFormatted, "Unknown", strlen("Unknown"))) {
				result = modbusRequestAndResponseStatusValues::readDataInvalidValue;
			}
		}
		break;
	case mqttEntityId::entityPvEnergy:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = 3399;
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_SYSTEM_OP_R_SYSTEM_TOTAL_PV_ENERGY_1, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityPvPwr:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = 3388;
		sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_CUSTOM_TOTAL_SOLAR_POWER, rs);
#endif // DEBUG_NO_RS485
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
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->signedIntValue = 1221;
		sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_GRID_METER_R_TOTAL_ACTIVE_POWER_1, rs);
#endif // DEBUG_NO_RS485
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
#endif // DEBUG_NO_RS485
		if (rs->unsignedShortValue > 215 && rs->unsignedShortValue < 265) {
			strcpy(rs->dataValueFormatted, "on");
		} else {
			strcpy(rs->dataValueFormatted, "off");
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
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->signedShortValue = -2255;
		sprintf(rs->dataValueFormatted, "%d", rs->signedShortValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_BATTERY_POWER, rs);
#endif // DEBUG_NO_RS485
		break;
	case mqttEntityId::entityBatSoc:
#ifdef DEBUG_NO_RS485
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->unsignedShortValue = 1000; // x 10
		sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
#else // DEBUG_NO_RS485
		result = _registerHandler->readHandledRegister(REG_BATTERY_HOME_R_SOC, rs);
#endif // DEBUG_NO_RS485
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
	case mqttEntityId::entityUptime:
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->unsignedIntValue = getUptimeSeconds();
		sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
		break;
	case mqttEntityId::entityVersion:
		rs->returnDataType = modbusReturnDataType::character;
		strcpy(rs->characterValue, _version);
		sprintf(rs->dataValueFormatted, "%s", rs->characterValue);
		result = modbusRequestAndResponseStatusValues::readDataRegisterSuccess;
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
		snprintf(_debugOutput, sizeof(_debugOutput), "readEntity: invalid val for: %s (%s)", singleEntity->mqttName, rs->dataValueFormatted);
		Serial.println(_debugOutput);
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

	sendMqtt(statusTopic);
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
		sprintf(stateAddition, "\"component\": \"number\"");
		break;
	case homeAssistantClass::homeAssistantClassSelect:
		sprintf(stateAddition, "\"component\": \"select\"");
		break;
	case homeAssistantClass::homeAssistantClassBinaryPower:
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
	case homeAssistantClass::homeAssistantClassBinaryPower:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"connectivity\""
			 ", \"payload_on\": \"on\""
			 ", \"payload_off\": \"off\"");
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
	case mqttEntityId::entityDispatchMode:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"options\": [ \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\" ]",
			 DISPATCH_START_STOP_DESC, DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV_DESC, DISPATCH_MODE_ECO_MODE_DESC,
			 DISPATCH_MODE_FCAS_MODE_DESC, DISPATCH_MODE_LOAD_FOLLOWING_DESC, DISPATCH_MODE_MAXIMISE_CONSUMPTION_DESC, DISPATCH_MODE_NORMAL_MODE_DESC,
			 DISPATCH_MODE_OPTIMISE_CONSUMPTION_DESC, DISPATCH_MODE_PV_POWER_SETTING_DESC, DISPATCH_MODE_STATE_OF_CHARGE_CONTROL_DESC);
		break;
	case mqttEntityId::entitySocTarget:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"battery\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"%%\""
			 ", \"icon\": \"mdi:battery\""
			 ", \"min\": %d, \"max\": %d",
			 SOC_TARGET_MIN, SOC_TARGET_MAX);
		break;
#ifdef DEBUG_WIFI
	case mqttEntityId::entityRSSI:
	case mqttEntityId::entityBSSID:
	case mqttEntityId::entityTxPower:
	case mqttEntityId::entityWifiRecon:
		sprintf(stateAddition, ", \"icon\": \"mdi:wifi\"");
		break;
#endif // DEBUG_WIFI
	case mqttEntityId::entityVersion:
		sprintf(stateAddition, ", \"icon\": \"mdi:numeric\"");
		break;
#ifdef DEBUG_RS485
	case mqttEntityId::entityRs485Errors:
	case mqttEntityId::entityRs485InvalidVals:
		sprintf(stateAddition, ", \"icon\": \"mdi:alert-decagram-outline\"");
		break;
#endif // DEBUG_RS485
#ifdef DEBUG_FREEMEM
	case mqttEntityId::entityFreemem:
		sprintf(stateAddition, ", \"icon\": \"mdi:memory\"");
		break;
#endif // DEBUG_FREEMEM
#ifdef DEBUG_CALLBACKS
	case mqttEntityId::entityCallbacks:
#endif // DEBUG_CALLBACKS
	case mqttEntityId::entityUptime:
	case mqttEntityId::entityBatSoc:
	case mqttEntityId::entityMaxCellTemp:
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
		case homeAssistantClass::homeAssistantClassBinaryPower:
			entityType = "binary_sensor";
			break;
		default:
			entityType = "sensor";
			break;
		}

		snprintf(topic, sizeof(topic), "homeassistant/%s/%s/%s/config", entityType, haUniqueId, singleEntity->mqttName);
		result = addConfig(singleEntity, resultAddedToPayload);
	} else {
		snprintf(topic, sizeof(topic), DEVICE_NAME "/%s/%s/state", haUniqueId, singleEntity->mqttName);
		result = addState(singleEntity, &resultAddedToPayload);
	}

	if ((resultAddedToPayload != modbusRequestAndResponseStatusValues::payloadExceededCapacity) &&
	    (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)) {
		// And send
		sendMqtt(topic);
	}
}


/*
  mqttCallback()

// This function is executed when an MQTT message arrives on a topic that we are subscribed to.
*/
void mqttCallback(char* topic, byte* message, unsigned int length)
{
#ifndef DEBUG_NO_RS485
	modbusRequestAndResponseStatusValues result = modbusRequestAndResponseStatusValues::preProcessing;
#endif
	modbusRequestAndResponse response;
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
		uint16_t singleRegisterValueConverted;
		char *endPtr = NULL;
		mqttState *relatedMqttEntity;
		bool valueProcessingError = false;

		// First, process value.
		switch (mqttEntity->entityId) {
		case mqttEntityId::entitySocTarget:
		case mqttEntityId::entityRegNum:
			singleInt32 = strtol(mqttIncomingPayload, &endPtr, 10);
			if ((endPtr == mqttIncomingPayload) || ((singleInt32 == 0) && (errno != 0))) {
				valueProcessingError = true;
#ifdef DEBUG
				snprintf(_debugOutput, sizeof(_debugOutput), "Trying to write %s with a bad value! %d", mqttEntity->mqttName, errno);
				Serial.println(_debugOutput);
#endif
			}
			break;
		case mqttEntityId::entityDispatchMode:
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

		if (!valueProcessingError) {
			// Now set the value and take appropriate action(s)
			switch (mqttEntity->entityId) {
			case mqttEntityId::entitySocTarget:
				if ((singleInt32 < SOC_TARGET_MIN) || (singleInt32 > SOC_TARGET_MAX)) {
#ifdef DEBUG
					sprintf(_debugOutput, "HA sent invalid SocTarget! %ld", singleInt32);
					Serial.println(_debugOutput);
#endif
				} else {
					singleRegisterValueConverted = (uint16_t)(singleInt32 / .4);
#ifdef DEBUG_NO_RS485
					socTarget = singleRegisterValueConverted;
#else // DEBUG_NO_RS485
					result = _registerHandler->writeRawSingleRegister(REG_DISPATCH_RW_DISPATCH_SOC, singleRegisterValueConverted, &response);
					if (result != modbusRequestAndResponseStatusValues::writeSingleRegisterSuccess) {
#ifdef DEBUG_RS485
						rs485Errors++;
#endif // DEBUG_RS485
					}
#endif // DEBUG_NO_RS485
				}
				break;
			case mqttEntityId::entityRegNum:
				regNumberToRead = singleInt32;    // Set local variable
				relatedMqttEntity = lookupEntity(mqttEntityId::entityRegValue);
				sendDataFromMqttState(relatedMqttEntity, false);    // Send update for related entity
				break;
			case mqttEntityId::entityDispatchMode:
				// DAVE - sanity check value.
				// DAVE - need to actually write a register here.  -- And be sure to handle DISPATCH_START_STOP_DESC
#ifdef DEBUG_NO_RS485
				strlcpy(dispatchModeDesc, singleString, sizeof(dispatchModeDesc));
#else // DEBUG_NO_RS485
				if (!strcmp(singleString, DISPATCH_START_STOP_DESC)) {
					result = _registerHandler->writeRawSingleRegister(REG_DISPATCH_RW_DISPATCH_START, DISPATCH_START_STOP, &response);
					if (result != modbusRequestAndResponseStatusValues::writeSingleRegisterSuccess) {
#ifdef DEBUG_RS485
						rs485Errors++;
#endif // DEBUG_RS485
					}
				} else {
					singleRegisterValueConverted = lookupDispatchMode(singleString);
					if (singleRegisterValueConverted != (uint16_t)-1) {
						result = _registerHandler->writeRawSingleRegister(REG_DISPATCH_RW_DISPATCH_MODE, singleRegisterValueConverted, &response);
						if (result == modbusRequestAndResponseStatusValues::writeSingleRegisterSuccess) {
							result = _registerHandler->writeRawSingleRegister(REG_DISPATCH_RW_DISPATCH_START, DISPATCH_START_START, &response);
							if (result != modbusRequestAndResponseStatusValues::writeSingleRegisterSuccess) {
#ifdef DEBUG_RS485
								rs485Errors++;
#endif // DEBUG_RS485
							}
						} else {
#ifdef DEBUG_RS485
							rs485Errors++;
#endif // DEBUG_RS485
						}
					}
				}
#endif // DEBUG_NO_RS485
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
void sendMqtt(const char *topic)
{
	// Attempt a send
	if (!_mqtt.publish(topic, _mqttPayload, MQTT_RETAIN)) {
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
getDispatchModeDesc(char *dest, uint16_t mode)
{
	// Type: Unsigned Short
	// <<Note7 - DISPATCH MODE LOOKUP>>
	switch (mode) {
	case DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV:
		strcpy(dest, DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV_DESC);
		break;
	case DISPATCH_MODE_ECO_MODE:
		strcpy(dest, DISPATCH_MODE_ECO_MODE_DESC);
		break;
	case DISPATCH_MODE_FCAS_MODE:
		strcpy(dest, DISPATCH_MODE_FCAS_MODE_DESC);
		break;
	case DISPATCH_MODE_LOAD_FOLLOWING:
		strcpy(dest, DISPATCH_MODE_LOAD_FOLLOWING_DESC);
		break;
	case DISPATCH_MODE_MAXIMISE_CONSUMPTION:
		strcpy(dest, DISPATCH_MODE_MAXIMISE_CONSUMPTION_DESC);
		break;
	case DISPATCH_MODE_NORMAL_MODE:
		strcpy(dest, DISPATCH_MODE_NORMAL_MODE_DESC);
		break;
	case DISPATCH_MODE_OPTIMISE_CONSUMPTION:
		strcpy(dest, DISPATCH_MODE_OPTIMISE_CONSUMPTION_DESC);
		break;
	case DISPATCH_MODE_PV_POWER_SETTING:
		strcpy(dest, DISPATCH_MODE_PV_POWER_SETTING_DESC);
		break;
	case DISPATCH_MODE_STATE_OF_CHARGE_CONTROL:
		strcpy(dest, DISPATCH_MODE_STATE_OF_CHARGE_CONTROL_DESC);
		break;
	default:
		sprintf(dest, "Unknown (%u)", mode);
		break;
	}
}

uint16_t
lookupDispatchMode(char *dispatchModeDesc)
{
	// Type: Unsigned Short
	// <<Note7 - DISPATCH MODE LOOKUP>>
	if (!strcmp(dispatchModeDesc, DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV_DESC)) 
		return DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV;
	if (!strcmp(dispatchModeDesc, DISPATCH_MODE_ECO_MODE_DESC))
		return DISPATCH_MODE_ECO_MODE;
	if (!strcmp(dispatchModeDesc, DISPATCH_MODE_FCAS_MODE_DESC))
		return DISPATCH_MODE_FCAS_MODE;
	if (!strcmp(dispatchModeDesc, DISPATCH_MODE_LOAD_FOLLOWING_DESC))
		return DISPATCH_MODE_LOAD_FOLLOWING;
	if (!strcmp(dispatchModeDesc, DISPATCH_MODE_MAXIMISE_CONSUMPTION_DESC))
		return DISPATCH_MODE_MAXIMISE_CONSUMPTION;
	if (!strcmp(dispatchModeDesc, DISPATCH_MODE_NORMAL_MODE_DESC))
		return DISPATCH_MODE_NORMAL_MODE;
	if (!strcmp(dispatchModeDesc, DISPATCH_MODE_OPTIMISE_CONSUMPTION_DESC))
		return DISPATCH_MODE_OPTIMISE_CONSUMPTION;
	if (!strcmp(dispatchModeDesc, DISPATCH_MODE_PV_POWER_SETTING_DESC))
		return DISPATCH_MODE_PV_POWER_SETTING;
	if (!strcmp(dispatchModeDesc, DISPATCH_MODE_STATE_OF_CHARGE_CONTROL_DESC))
		return DISPATCH_MODE_STATE_OF_CHARGE_CONTROL;
	return (uint16_t)-1;  // Shouldn't happen
}

#ifdef DEBUG_FREEMEM
uint32_t freeMemory()
{
	return ESP.getFreeHeap();
}
#endif // DEBUG_FREEMEM
