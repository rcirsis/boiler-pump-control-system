	#include <OneWire.h>
	#include <DallasTemperature.h>
	#include <Arduino.h>
	#include "credentials.h" // holds wifi points names and their passwords, server address where data will be posted
  
	#include <Ticker.h>
	Ticker Timer1;
	unsigned int sendDataToServerBool = 0;
	Ticker Timer2; // Main Program Timer
	unsigned int updateFlag = 1;  // Do Autorun?

	#include <ESP8266WiFi.h>
	#include <ESP8266WiFiMulti.h>
	#include <ESP8266HTTPClient.h>
	#include <WiFiClient.h>
	ESP8266WiFiMulti WiFiMulti;
	char link[1000];

	#include <ModbusRTU.h>
	#include <SoftwareSerial.h>
	ModbusRTU mb;
	int DE_RE = 16; // D0  For MAX485 chip direction control
	int R0 = 12; 	// D6
	int DI = 13; 	// D7
	SoftwareSerial rs485;
	uint16_t Mread0[2];

	#define ON LOW
	#define OFF HIGH

	#define ONE_WIRE_BUS D2 // Data wire is plugged TO GPIO 4 
	#define LED D4
	#define MOTOR D5
	#define BOILER D1

	int numberOfDevices; // Number of temperature devices found
	int errorCode = 0; // Errors
	int motor = 0; // status
	int boiler = 1;  // status (by default - works)
	float heaterTempC = -127;
	float heaterTempMathC = -127;
	float boilerTempC = -127;
	float kWatts = -1;

	unsigned long previousMillis = 0; // for led blinking
	const long interval = 900;  
	unsigned long wifiAttemp = 0;

	OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
	DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature. 

	DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
	DeviceAddress heaterThermometer = { 0x28, 0xFF, 0x64, 0xB8, 0x85, 0x16, 0x03, 0x38 }; // #28 FF 64 B8 85 16 03 38
	DeviceAddress boilerThermometer = { 0x28, 0xFF, 0xAA, 0xF4, 0xC1, 0x16, 0x04, 0x66 }; // #28 FF AA F4 C1 16 04 66

	void setup(){
		// start serial port
		Serial.begin(115200);
		//Serial.setDebugOutput(true);
		while (!Serial) delay(50);

		pinMode(LED, OUTPUT);
		pinMode(MOTOR, OUTPUT);
		pinMode(BOILER, OUTPUT);
		digitalWrite(MOTOR, OFF);
		digitalWrite(BOILER, LOW);  // HIGH = BOILER IS WORKING ; LOW = BOILER IS OFF
		boiler = 0;

		// Start up the library
		sensors.begin();

		// Grab a count of devices on the wire
		numberOfDevices = sensors.getDeviceCount();

		Serial.println();
		Serial.println();
		Serial.println();

		for (uint8_t t = 5; t > 0; t--) {
			Serial.printf("[BOOT] WAIT %d\n", t);
			Serial.flush();
			delay(1000);
		}

		// === ENABLE WIFI  ===
		Serial.println("Connecting to WIFI");
		WiFi.mode(WIFI_STA);
		for(int i = 0; i < sizeof(wifi_data); i++) {
		WiFiMulti.addAP(wifi_data[i][0], wifi_data[i][1]);
		}
		WiFiMulti.run();
		delay(500);

		while(WiFiMulti.run() != WL_CONNECTED) {
			delay(500);
			wifiAttemp++;
			Serial.print("..");
			if (wifiAttemp>=20) break;
		}
		Serial.println("");
   
		if (WiFiMulti.run() == WL_CONNECTED) {
			Serial.print("WiFi connected! IP address: ");
			Serial.println(WiFi.localIP());
		} else {
			Serial.println("[!] WiFi settings problem!");
		}

		delay(1000);

		// locate devices on the bus
		Serial.print("Locating devices...");
		Serial.print("Found ");
		Serial.print(numberOfDevices, DEC);
		Serial.println(" devices.");

		delay(1000);

		// Loop through each device, print out address
		for(int i=0;i<numberOfDevices; i++){
			// Search the wire for address
			if(sensors.getAddress(tempDeviceAddress, i)){
			Serial.print("Found device ");
			Serial.print(i, DEC);
			Serial.print(" with address: #");
			printAddress(tempDeviceAddress);
			Serial.println();
			} else {
			Serial.print("Found ghost device at ");
			Serial.print(i, DEC);
			Serial.print(" but could not detect address. Check power and cabling");
			}
		}

		if (sensors.isConnected(heaterThermometer) && sensors.isConnected(boilerThermometer)) {
			Serial.println("Sensors from memory connected to device");
		} else {
			Serial.println("ERROR: heaterThermometer or boilerThermometer not FOUND! [Error #001]");
			errorCode = 1;
		}

		// === Software Serial + RS485 + MODBUS RTU  ===
		Serial.print("MODBUS RTU Setup: ");
		rs485.begin(9600, SWSERIAL_8E1, R0, DI);
		if (!rs485) {  // If the object did not initialize, then its configuration - is invalid
			Serial.println("Invalid SoftwareSerial pin configuration, check config");
			errorCode = 1;
		} else {
			Serial.println("Software Serial is Active");
		}
		mb.begin(&rs485, DE_RE);// Assing Software serial port to Modbus instance for MAX485 chip having DI,DE,RE,RO Pin at TTL side
		mb.master();			// Assing Modbus function as master

		// === SETUP ACTION TIMERS  ===
		Timer1.attach(90, sendDataToServerTimer); // Timer for action
		Timer2.attach(10, updateTimer); // Timer for main code

		Serial.println("--- Setup is complieted ---");
	}

	void sendDataToServerTimer() {
		if (sendDataToServerBool == 0) sendDataToServerBool = 1;
	}

	void sendDataToServer() {
		sendDataToServerBool = 2;
		// SENDINGS DATA TO SERVER VIA WIFI
		if ((WiFiMulti.run() == WL_CONNECTED)) {
			WiFiClient client;
			HTTPClient http;
			sprintf(link,	"%s?heater=%f&boiler=%f&motor=%d&boilerR=%d&tac=%f", server, heaterTempC, boilerTempC, motor, boiler, kWatts);
			if (http.begin(client, link)) {  // HTTP
				// Serial.println(link);
				int httpCode = http.GET();
				if (httpCode > 0) {
					Serial.printf("[WIFI] Data send to server [%d]\n", httpCode);
					if (httpCode == HTTP_CODE_OK ||
						httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
						// String payload = http.getString();
						// Serial.println(payload);
					}
				} else {
					Serial.printf("[WIFI] Sends failed, error: %s\n",
								http.errorToString(httpCode).c_str());
				}
				http.end();
			} else {
				Serial.printf("[WIFI] Unable to connect\n");
			}
		}
		sendDataToServerBool = 0;
	}

	void updateTimer() {
		if (updateFlag == 0) updateFlag = 1;
	}

	void update() {
		updateFlag = 2;

		// GET DATA FROM SENSORS
		sensors.requestTemperatures();  // Send the command to get temperatures
		heaterTempC = sensors.getTempC(heaterThermometer);
		boilerTempC = sensors.getTempC(boilerThermometer);
		Serial.print("Boiler temperature: ");
		Serial.print(boilerTempC);
		Serial.print("ºC and Heater temperature: ");
		Serial.print(heaterTempC);
		Serial.println("ºC");

		// THE MOTOR LOGIC (RELAY)
		if (boilerTempC <= 0 || heaterTempC <= 0) {
			digitalWrite(MOTOR, OFF);
			motor = 0;
			Serial.println("ERROR! Unreal Sensor data - stopping motor");
			for (uint8_t t = 50; t > 0; t--) {
				digitalWrite(LED, !digitalRead(LED));
				delay(200);
			}
		} else {
			heaterTempMathC = heaterTempC - 1;
			if (boilerTempC >= 65) {
				digitalWrite(MOTOR, OFF);
				motor = 0;
				Serial.println("Boiler temperature over 65ºC | MOTOR: OFF - BOILER: OFF");

				digitalWrite(BOILER, LOW); // HIGH = BOILER IS WORKING; LOW = BOILER IS OFF
				boiler = 0;
				Serial.println("BOILER: OFF");
			} else if ((heaterTempC >= 31) && (heaterTempMathC >= boilerTempC)) {
				digitalWrite(MOTOR, ON);
				motor = 1;
				Serial.println("MOTOR: ON");

				digitalWrite(BOILER, LOW); // HIGH = BOILER IS WORKING; LOW = BOILER IS OFF
				boiler = 0;
				Serial.println("BOILER: OFF");	
			} else {
				digitalWrite(MOTOR, OFF);
				motor = 0;
				Serial.println("MOTOR: OFF");
				if (boilerTempC<=36) {
					digitalWrite(BOILER, HIGH); // HIGH = BOILER IS WORKING; LOW = BOILER IS OFF
					boiler = 1;
					Serial.println("BOILER: ON");	
				} else if (boilerTempC>38) {	
					digitalWrite(BOILER, LOW); // HIGH = BOILER IS WORKING; LOW = BOILER IS OFF
					boiler = 0;
					Serial.println("BOILER: OFF");	
				}
			}
		}

		// GET COUNTER DATA - MODBUS
		if (!mb.slave()) {  
			// (SlaevID, Address, Buffer, Range of data, Modus call) mbReply
			//mb.readIreg(49, 0x0000, Mread0, 2); // Voltage
			mb.readIreg(49, 0x0100, Mread0, 2); // Power
			kWatts = InttoFloat(Mread0[0], Mread0[1]);
            Serial.printf("[MODBUS 49] Total active power: %f KWh\n", kWatts);
		}
		mb.task();
		yield();

		updateFlag = 0;
	}

	void loop()	{
		if (errorCode==0) {
			if (updateFlag == 1) update();
			if (sendDataToServerBool == 1) sendDataToServer();

			unsigned long timeCounter = millis();
			if (timeCounter - previousMillis >= interval) {
				previousMillis = timeCounter;
				digitalWrite(LED, ON);
				delay(50);
				digitalWrite(LED, OFF);
				delay(120);
                digitalWrite(LED, ON);
                delay(50);
                digitalWrite(LED, OFF);
			}
		} else {
			while(1) { 
				digitalWrite(LED, !digitalRead(LED));
				delay(100);
            }
        }
	}

	// function to print a device address
	void printAddress(DeviceAddress deviceAddress) {
		for (uint8_t i = 0; i < 8; i++){
			if (deviceAddress[i] < 16) Serial.print("0");
			Serial.print(deviceAddress[i], HEX);
		}
	}

	float InttoFloat(uint16_t Data0, uint16_t Data1) {
		float x;
		unsigned long* p;
		p = (unsigned long*)&x;
		*p = (unsigned long)Data0 << 16 | Data1;  // Big-endian
		return (x);
	}

	bool mbReply(Modbus::ResultCode event, uint16_t transactionId, void* data) {
		Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
		return true;
	}
