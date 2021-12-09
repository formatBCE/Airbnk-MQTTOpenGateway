#include <stdlib.h>
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include <AsyncTCP.h>
#include <Arduino.h>

#include <NimBLEDevice.h>
#include <NimBLEAdvertisedDevice.h>
#include "NimBLEEddystoneURL.h"
#include "NimBLEEddystoneTLM.h"
#include "NimBLEBeacon.h"

#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include "Settings.h"

static const int scanTime = singleScanTime;
static const int waitTime = scanInterval;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
String localIp;
byte retryAttempts = 0;
unsigned long last = 0;
NimBLEScan* pBLEScan;
NimBLEClient* pClient;
NimBLEAdvertisedDevice lockDevice; 
TaskHandle_t NimBLEScan;
size_t commandLength;
bool isSending;

void connectToWifi() {
  	Serial.println("Connecting to WiFi...");
	WiFi.begin(ssid, password);
	WiFi.setHostname(hostname);
}

void connectToMqtt() {
  	Serial.println("Connecting to MQTT");
	if (WiFi.isConnected()) {
		mqttClient.setCredentials(mqttUser, mqttPassword);
		mqttClient.setClientId(hostname);
	 	mqttClient.connect();
	} else {
		Serial.println("Cannot reconnect MQTT - WiFi error");
		handleWifiDisconnect();
	}
}

void handleMqttDisconnect() {
	if (retryAttempts > 10) {
		Serial.println("Too many retries. Restarting");
		ESP.restart();
	} else {
		retryAttempts++;
	}
	if (WiFi.isConnected()) {
		Serial.println("Starting MQTT reconnect timer");
		if (xTimerReset(mqttReconnectTimer, 0) == pdFAIL) {
			Serial.println("failed to restart");
			xTimerStart(mqttReconnectTimer, 0);
		} else {
			Serial.println("restarted");
		}
  	} else {
		Serial.print("Disconnected from WiFi; starting WiFi reconnect timiler\t");
		handleWifiDisconnect();
	}
}

bool handleWifiDisconnect() {
	if (WiFi.isConnected()) {
		Serial.println("WiFi appears to be connected. Not retrying.");
		return true;
	}
	if (retryAttempts > 10) {
		Serial.println("Too many retries. Restarting");
		ESP.restart();
	} else {
		retryAttempts++;
	}
	if (mqttClient.connected()) {
		mqttClient.disconnect();
	}
	if (xTimerIsTimerActive(mqttReconnectTimer) != pdFALSE) {
		xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
	}

	if (xTimerReset(wifiReconnectTimer, 0) == pdFAIL) {
		Serial.println("failed to restart");
		xTimerStart(wifiReconnectTimer, 0);
		return false;
	} else {
		Serial.println("restarted");
		return true;
	}

}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %x\n\r", event);

	switch(event) {
	case SYSTEM_EVENT_STA_GOT_IP:
		digitalWrite(LED_BUILTIN, !LED_ON);
		Serial.print("IP address: \t");
		Serial.println(WiFi.localIP());
		localIp = WiFi.localIP().toString().c_str();
		Serial.print("Hostname: \t");
		Serial.println(WiFi.getHostname());
		connectToMqtt();
		if (xTimerIsTimerActive(wifiReconnectTimer) != pdFALSE) {
			Serial.println("Stopping wifi reconnect timer");
			xTimerStop(wifiReconnectTimer, 0);
		}
		retryAttempts = 0;
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		digitalWrite(LED_BUILTIN, LED_ON);
		Serial.println("WiFi lost connection, resetting timer\t");
		handleWifiDisconnect();
		break;
	case SYSTEM_EVENT_WIFI_READY:
		Serial.println("Wifi Ready");
		handleWifiDisconnect();
		break;
	case SYSTEM_EVENT_STA_START:
		Serial.println("STA Start");
		tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, hostname);
		if (xTimerIsTimerActive(wifiReconnectTimer) != pdFALSE) {
			TickType_t xRemainingTime = xTimerGetExpiryTime( wifiReconnectTimer ) - xTaskGetTickCount();
			Serial.print("WiFi Time remaining: ");
			Serial.println(xRemainingTime);
		} else {
			Serial.println("WiFi Timer is inactive; resetting\t");
			handleWifiDisconnect();
		}
		break;
	case SYSTEM_EVENT_STA_STOP:
		Serial.println("STA Stop");
		handleWifiDisconnect();
		break;

    }
}

bool sendTelemetry() {
	StaticJsonDocument<256> tele;
	tele["ip"] = localIp;
	tele["hostname"] = WiFi.getHostname();
	tele["scan_dur"] = scanTime;
	tele["wait_dur"] = waitTime;

	char teleMessageBuffer[258];
	serializeJson(tele, teleMessageBuffer);

	if (mqttClient.publish(telemetryTopic, 0, 0, teleMessageBuffer) == true) {
		Serial.println("Telemetry sent");
		return true;
	} else {
		Serial.println("Error sending telemetry");
		return false;
	}
}

bool reportDevice(NimBLEAdvertisedDevice advertisedDevice) {

	String mac_address = advertisedDevice.getAddress().toString().c_str();
	mac_address.toUpperCase();
	if (mac_address != lockMacAddress) {
		return false;
	}

	lockDevice = advertisedDevice;

	StaticJsonDocument<500> doc;

	std::string manData = advertisedDevice.getManufacturerData();
	char *pHex = NimBLEUtils::buildHexData(nullptr, (uint8_t*)manData.data(), manData.length());

	doc["mac"] = mac_address;
	doc["rssi"] = advertisedDevice.getRSSI();
	doc["data"] = pHex;

	char JSONmessageBuffer[512];
	serializeJson(doc, JSONmessageBuffer);

	String publishTopic = String(hostname) + "/adv";

	if (mqttClient.connected()) {
		if (mqttClient.publish((char *)publishTopic.c_str(), 0, 0, JSONmessageBuffer) == true) {
			Serial.println("Lock advertisement sent.");
			return true;
		} else {
			Serial.print("Error sending advertisement message: ");
			Serial.println(publishTopic);
			Serial.print("Message: ");
			Serial.println(JSONmessageBuffer);
			return false;
		}
	} else {
		Serial.println("MQTT disconnected.");
		if (xTimerIsTimerActive(mqttReconnectTimer) != pdFALSE) {
			TickType_t xRemainingTime = xTimerGetExpiryTime( mqttReconnectTimer ) - xTaskGetTickCount();
			Serial.print("Time remaining: ");
			Serial.println(xRemainingTime);
		} else {
			handleMqttDisconnect();
		}
	}
	return false;
}

void sendCommandResult(boolean success, String error, int sign) {
	StaticJsonDocument<128> doc;
	doc["success"] = success;
	doc["error"] = error;
	doc["sign"] = sign;
	doc["mac"] = lockMacAddress;
	char messageBuffer[130];
	serializeJson(doc, messageBuffer);

	if (mqttClient.publish(commandResultTopic, 0, 0, messageBuffer) == true) {
		Serial.println("Command result sent");
	} else {
		Serial.println("Error sending command result");
	}
}

int fromHex(uint8_t *dest, const char *src, int maxlen) {
  int srclen = strlen(src)/2;
  if (srclen > maxlen) {
    return 0;
  }

  memset(dest, 0, maxlen);

  for (int i = 0; i < srclen; i++){
    char t[3];

    t[0] = src[i*2];
    t[1] = src[i*2 + 1];
    t[2] = 0;
    if (t[0] == '/'){
      t[0] = '0';
    }
    if (t[1] == '/'){
      t[1] = '0';
    }

    if (!isalnum(t[0])){
      return 0;
    }
    if (!isalnum(t[1])){
      return 0;
    }


    t[0] |= 0x20;
    t[1] |= 0x20;
    if (isalpha(t[0])){
      if (t[0] < 'a' || t[0] > 'f'){
        return 0;
      }
    }
    if (isalpha(t[1])){
      if (t[1] < 'a' || t[1] > 'f'){
        return 0;
      }
    }

    int byte = strtol(t, NULL, 16);
    *dest++ = byte;
  }
  return srclen;
}


void sendBlePayload(char* mqttMessage, size_t commandLength) {
	isSending = true;
	if (pBLEScan->isScanning()) {
		pBLEScan->stop();
		while (pBLEScan->isScanning()) {
			delay(100);
		}
	}
	bool result = false;
	String error = "";
	char pldArr[commandLength];
	strcpy(pldArr, mqttMessage);
	Serial.println(pldArr);
	DynamicJsonDocument doc (1024);
	deserializeJson(doc, pldArr);
	int sign = doc["sign"];
	const char* cmnd1 = doc["command1"];
	const char* cmnd2 = doc["command2"];
	uint8_t command1[20];
	uint8_t command2[20];
	int len1 = fromHex(command1, cmnd1, 20);
	int len2 = fromHex(command2, cmnd2, 20);
	Serial.print("Sending to ");
	Serial.println(lockDevice.toString().c_str());
	int retry = 1;
	while (retry < 5 && result == false) {
		if (retry > 1) {
			delay(1000);
		}
		Serial.print("Sending try ");
		Serial.println(retry);
		if (pClient->connect(&lockDevice)) {
			Serial.println("Connected to lock.");
			uint16_t shortServiceUuid = strtol(serviceUUID, NULL, 0);
			NimBLERemoteService* pRemoteService = pClient->getService(BLEUUID(shortServiceUuid));
			if (pRemoteService == nullptr) {
				Serial.println("Failed to get service.");
				error = "FAILED TO GET SERVICE";
			} else {
				Serial.println("Got service.");
				uint16_t shortCharactUuid = strtol(characteristicUUID, NULL, 0);
				NimBLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(NimBLEUUID(shortCharactUuid));
				if (pRemoteCharacteristic == nullptr) {
					Serial.println("Failed to get characteristic.");
					error = "FAILED TO GET CHARACTERISTIC";
				} else {
					Serial.println("Got characteristic, writing.");
					if (
						pRemoteCharacteristic->writeValue(command1, len1, true)
						&& pRemoteCharacteristic->writeValue(command2, len2, true)
						) {
							Serial.println("Write successful.");
							result = true;
						} else {
							error = "FAILED TO WRITE";
							Serial.println("Failed to write characteristic.");
						}
					pRemoteCharacteristic = NULL;
				}
				pRemoteService = NULL;
			}
			pClient->disconnect();
		} else {
			Serial.println("Failed to connect to lock.");
			error = "FAILED TO CONNECT";
		}
		retry++;
	}
	isSending = false;
	sendCommandResult(result, error, sign);
}

void scanForDevices(void * parameter) {
	while (1) {
		while (isSending) {
			delay(10);
		}
		if (WiFi.isConnected() && (millis() - last > (waitTime * 1000) || last == 0)) {
			Serial.println("Scanning...\t");
			pBLEScan->start(scanTime);
			Serial.print("Scanning done.");
			pBLEScan->clearResults();
			sendTelemetry();
			last = millis();
		}
	}
}

class AirbnkAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* device) {
		if (mqttClient.connected()) {
			if (reportDevice(*device)) {
				pBLEScan->stop(); // resetting the scan
			}
		} else {
			Serial.println("Cannot report; mqtt disconnected");
			if (xTimerIsTimerActive(mqttReconnectTimer) != pdFALSE) {
				TickType_t xRemainingTime = xTimerGetExpiryTime( mqttReconnectTimer ) - xTaskGetTickCount();
				Serial.print("Time remaining: ");
				Serial.println(xRemainingTime);
			} else {
				handleMqttDisconnect();
			}
		}
    }
};

void onMqttConnect(bool sessionPresent) {
  	Serial.println("Connected to MQTT.");
	retryAttempts = 0;

	if (mqttClient.publish(availabilityTopic, 0, 1, "CONNECTED") == true) {
		Serial.print("Success sending message to topic:\t");
		Serial.println(availabilityTopic);
		mqttClient.subscribe(commandTopic, 2);
	} else {
		Serial.println("Error sending message");
	}

	sendTelemetry();
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
	Serial.println("Got command");
	if (isSending) {
		Serial.println("Already sending something, abort");
		return;
	}
	sendBlePayload(payload, len);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
	Serial.println("Disconnected from MQTT.");
	handleMqttDisconnect();
}

void setup() {

	Serial.begin(115200);

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LED_ON);

	mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
	wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

	WiFi.onEvent(WiFiEvent);

	mqttClient.onConnect(onMqttConnect);
	mqttClient.onDisconnect(onMqttDisconnect);
	mqttClient.onMessage(onMqttMessage);

	mqttClient.setServer(mqttHost, mqttPort);
	mqttClient.setWill(availabilityTopic, 0, 1, "DISCONNECTED");
	mqttClient.setKeepAlive(60);

	connectToWifi();

  	NimBLEDevice::init("");
	NimBLEDevice::setPower(ESP_PWR_LVL_P9);
	pClient = NimBLEDevice::createClient();
  	pBLEScan = NimBLEDevice::getScan(); //create new scan
  	//pBLEScan->setActiveScan(true);
	pBLEScan->setAdvertisedDeviceCallbacks(new AirbnkAdvertisedDeviceCallbacks());
	pBLEScan->setInterval(bleScanInterval);
	pBLEScan->setWindow(bleScanWindow);
	xTaskCreatePinnedToCore(
		scanForDevices,
		"BLE Scan",
		4096,
		pBLEScan,
		1,
		&NimBLEScan,
		1);
}

void loop() {
	TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
	TIMERG0.wdt_feed=1;
	TIMERG0.wdt_wprotect=0;
}
