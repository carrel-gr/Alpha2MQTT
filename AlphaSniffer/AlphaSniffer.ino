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
char _version[6] = "v2.21";

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
#ifdef DEBUG_RS485
uint32_t rs485Errors = 0;
uint32_t rs485InvalidValues = 0;
#endif // DEBUG_RS485
uint32_t numRs485Tx = 0;
uint32_t numRs485Rx = 0;
uint32_t numRs485Unk = 0;
uint32_t numRs485NoResp = 0;

// These timers are used in the main loop.
#define RUNSTATE_INTERVAL 2500
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

#ifdef DEBUG
	sprintf(_debugOutput, "About to request buffer");
	Serial.println(_debugOutput);
#endif
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

	digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX);

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
	modbusRequestAndResponseStatusValues result = modbusRequestAndResponseStatusValues::preProcessing;
	modbusRequestAndResponse resp;
	char line[2048];

	// Refresh LED Screen, will cause the status asterisk to flicker
	updateOLED(true, "", "", "");

	// Make sure WiFi is good
	if ((WiFi.status() != WL_CONNECTED) || !_wifi.connected()) {
		setupWifi(false);
	}

	// Check and display the runstate on the display
	updateRunstate();

	// Read one modbus packet (RS485 data)
        digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX);
	result = _modBus->listenResponse(&resp);

	if (result == modbusRequestAndResponseStatusValues::noResponse) {
		numRs485NoResp++;
		return;
	}

	switch (resp.direction) {
	case dataDirection::unknown:
		numRs485Unk++;
		strcpy(line, "UK ");
		break;
	case dataDirection::rx:
		numRs485Rx++;
		strcpy(line, "RX ");
		break;
	case dataDirection::tx:
		numRs485Tx++;
		strcpy(line, "TX ");
		break;
	}
	if (result == modbusRequestAndResponseStatusValues::writeDataRegisterSuccess) {
		strcat(line, "WD");
	} else if (result == modbusRequestAndResponseStatusValues::writeSingleRegisterSuccess) {
		strcat(line, "WS");
	} else if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess) {
		strcat(line, "RD");
	} else {
		snprintf(line, sizeof(line), "ER %s", resp.statusMqttMessage);
	}

	if ((result == modbusRequestAndResponseStatusValues::writeDataRegisterSuccess) ||
	    (result == modbusRequestAndResponseStatusValues::writeSingleRegisterSuccess) ||
	    (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)) {
		for (int i = 0; i < resp.dataSize; i++) {
			char data[4];
			snprintf(data, sizeof(data), " %02X", resp.data[i]);
			strlcat(line, data, sizeof(line));
		}
	}

	_wifi.println(line);
// DAVE - NOTE addrH is never 2 or 3
		// For MODBUS_FN_READDATAREGISTER (0x03)    - 8 bytes - 0x55, fn, addrH, addrL, 0, regCount, crc, crc
		//       response              - 5 + (2 * regC) bytes - 0x55, fn, 2*regCount, No1.dataH, No1.dataL, ... , crc, crc  (... is 2 bytes for each reg in regCount)
		// For MODBUS_FN_WRITESINGLEREGISTER (0x06) - 8 bytes - 0x55, fn, addrH, addrL, valH, valL, crc, crc
		//       response                           - 8 bytes - 0x55, fn, addrH, addrL, valH, valL, crc, crc
		// For MODBUS_FN_WRITEDATAREGISTER (0x10)  - 11 bytes - 0x55, fn, addrH, addrL, 0, 1, 2, valH, valL, crc, crc
		//     MODBUS_FN_WRITEDATAREGISTER (0x10)  - 13 bytes - 0x55, fn, addrH, addrL, 0, 2, 4, valHH, valHL, valLH, valLL, crc, crc
		//       response                           - 8 bytes - 0x55, fn, addrH, addrL, 0, regCount, crc, crc
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
			updateOLED(false, "WiFi Connecting", line3, line4);
		} else {
			updateOLED(false, "WiFi Reconnect", line3, line4);
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

	while (!_wifi.connected()) {
		_wifi.flush();
		_wifi.stop();
		if (_wifi.connect(SNIFF_SERVER, SNIFF_PORT)) {
			snprintf(line3, sizeof(line3), "Success");
			_wifi.print("AlphaSNIFF connected.  IP is ");
			_wifi.println(WiFi.localIP());
		} else {
#ifdef DEBUG
			sprintf(_debugOutput, "Failed Connection to %s:%d", SNIFF_SERVER, SNIFF_PORT);
			Serial.println(_debugOutput);
#endif
			snprintf(line3, sizeof(line3), "Failed");
		}
		updateOLED(false, "Client Connecting", line3, "");
		delay(500);
	}
#ifdef DEBUG
	sprintf(_debugOutput, "Connected to %s:%d", SNIFF_SERVER, SNIFF_PORT);
	Serial.println(_debugOutput);
#endif

	snprintf(line3, sizeof(line3), "%s", WiFi.localIP().toString().c_str());
	snprintf(line4, sizeof(line4), "%s:%d", SNIFF_SERVER, SNIFF_PORT);

	// Connected, so ditch out with blank screen
	updateOLED(false, line3, line4, _version);
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
	snprintf(line1Contents, sizeof(line1Contents), "SNIF %c%c%c  RSSI: %d",
		 _oledOperatingIndicator, (WiFi.status() == WL_CONNECTED ? 'W' : ' '), (_wifi.connected() ? 'C' : ' '), WiFi.RSSI() );
#else // LARGE_DISPLAY
	// There's ten characters we can play with, width wise.
	snprintf(line1Contents, sizeof(line1Contents), "%s%c%c%c", "SNIF   ",
		 _oledOperatingIndicator, (WiFi.status() == WL_CONNECTED ? 'W' : ' '), (_wifi.connected() ? 'C' : ' ') );
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
	char line2[OLED_CHARACTER_WIDTH] = "";
	char line3[OLED_CHARACTER_WIDTH] = "";
	char line4[OLED_CHARACTER_WIDTH] = "";


	if (checkTimer(&lastRun, RUNSTATE_INTERVAL)) {
		//Flash the LED
		digitalWrite(LED_BUILTIN, LOW);
		delay(4);
		digitalWrite(LED_BUILTIN, HIGH);

		snprintf(line2, sizeof(line2), "Num TX:: %lu", numRs485Tx);
		snprintf(line3, sizeof(line3), "Num RX:: %lu", numRs485Rx);

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
#ifdef DEBUG_RS485
			} else if (debugIdx < 9) {
				snprintf(line4, sizeof(line4), "RS485 Err: %lu", rs485Errors);
				debugIdx = 9;
			} else if (debugIdx < 10) {
				snprintf(line4, sizeof(line4), "RS485 Inv: %lu", rs485InvalidValues);
				debugIdx = 10;
#endif // DEBUG_RS485
			} else if (debugIdx < 11) {
				snprintf(line4, sizeof(line4), "RS485 Tx: %lu", numRs485Tx);
				debugIdx = 11;
			} else if (debugIdx < 12) {
				snprintf(line4, sizeof(line4), "RS485 Rx: %lu", numRs485Rx);
				debugIdx = 12;
			} else if (debugIdx < 13) {
				snprintf(line4, sizeof(line4), "RS485 UK: %lu", numRs485Unk);
				debugIdx = 13;
			} else if (debugIdx < 14) {
				snprintf(line4, sizeof(line4), "RS485 NR: %lu", numRs485NoResp);
				debugIdx = 14;
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
