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
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <Preferences.h>

#include <NimBLEDevice.h>
#include <NimBLEAdvertisedDevice.h>
#include "NimBLEEddystoneURL.h"
#include "NimBLEEddystoneTLM.h"
#include "NimBLEBeacon.h"

#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include "Settings.h"

// Config fields
AsyncWebServer server(80);
size_t content_len;
Preferences preferences;
bool isSetUp = false;

String wifi_ssid = "";
String wifi_pwd = "";
String mqtt_ip = "";
int mqtt_port = 0;
String mqtt_user = "";
String mqtt_pass = "";
String mqtt_topic = "";
String lock_mac = "";

// Main fields
static const int scanTime = singleScanTime;
static const int waitTime = scanInterval;
String availabilityTopic;
String commandTopic;
String telemetryTopic;
String commandResultTopic;
String advertTopic;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
String localIp;
byte mqttRetryAttempts = 0;
byte wifiRetryAttempts = 0;
unsigned long last = 0;
NimBLEScan* pBLEScan;
NimBLEClient* pClient;
NimBLEAdvertisedDevice lockDevice; 
TaskHandle_t NimBLEScan;
size_t commandLength;
bool isSending;

void connectToWifi() {
  	Serial.println("Connecting to WiFi...");
	WiFi.begin(wifi_ssid.c_str(), wifi_pwd.c_str());
	WiFi.setHostname(mqtt_topic.c_str());
}

void connectToMqtt() {
  	Serial.println("Connecting to MQTT");
	if (WiFi.isConnected()) {
		mqttClient.setCredentials(mqtt_user.c_str(), mqtt_pass.c_str());
		mqttClient.setClientId(mqtt_topic.c_str());
	 	mqttClient.connect();
	} else {
		Serial.println("Cannot reconnect MQTT - WiFi error");
		handleWifiDisconnect();
	}
}

void handleMqttDisconnect() {
	if (mqttRetryAttempts > 10) {
		Serial.println("Too many retries. Restarting.");
		ESP.restart();
	} else {
		mqttRetryAttempts++;
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
	digitalWrite(LED_BUILTIN, !LED_ON);
	delay(500);
	digitalWrite(LED_BUILTIN, LED_ON);
	if (wifiRetryAttempts > 10) {
		preferences.begin(wifi_prefs, false);
		uint16_t wiFiRestartCounter = preferences.getShort("wifi_reconnect", 0);
		if (wiFiRestartCounter > 9) {
			preferences.clear();
			preferences.end();
			preferences.begin(main_prefs, false);
			preferences.clear();
			preferences.end();
			Serial.println("Too many retries. Cleaning data and restarting.");
		} else {
			wiFiRestartCounter++;
			preferences.putShort("wifi_reconnect", wiFiRestartCounter);
			preferences.end();
			Serial.println("Too many retries. Restarting.");
		}
		ESP.restart();
	} else {
		wifiRetryAttempts++;
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
		wifiRetryAttempts = 0;
		preferences.begin(wifi_prefs, false);
		preferences.clear();
		preferences.end();
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		Serial.println("WiFi lost connection, resetting timer\t");
		handleWifiDisconnect();
		break;
	case SYSTEM_EVENT_WIFI_READY:
		Serial.println("Wifi Ready");
		handleWifiDisconnect();
		break;
	case SYSTEM_EVENT_STA_START:
		Serial.println("STA Start");
		tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, mqtt_topic.c_str());
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
	preferences.begin(wifi_prefs, false);
	uint16_t wiFiRestartCounter = preferences.getShort("wifi_reconnect", 0);
	preferences.end();
	if (wiFiRestartCounter > 0) {
		tele["wifi_restart_count"] = wiFiRestartCounter;
	}
	char teleMessageBuffer[258];
	serializeJson(tele, teleMessageBuffer);

	if (mqttClient.publish(telemetryTopic.c_str(), 0, 0, teleMessageBuffer) == true) {
		Serial.println("Telemetry sent");
		return true;
	} else {
		Serial.println("Error sending telemetry");
		mqttClient.disconnect();
		return false;
	}
}

bool reportDevice(NimBLEAdvertisedDevice advertisedDevice) {

	String mac_address = advertisedDevice.getAddress().toString().c_str();
	mac_address.toUpperCase();
	if (mac_address != lock_mac.c_str()) {
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

	if (mqttClient.connected()) {
		if (mqttClient.publish(advertTopic.c_str(), 0, 0, JSONmessageBuffer) == true) {
			Serial.println("Lock advertisement sent.");
			return true;
		} else {
			Serial.print("Error sending advertisement message.");
			Serial.println(advertTopic);
			Serial.print("Message: ");
			Serial.println(JSONmessageBuffer);
			mqttClient.disconnect();
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

void sendCommandResult(boolean success, String error, int sign, std::string status) {
	StaticJsonDocument<128> doc;
	doc["success"] = success;
	doc["error"] = error;
	doc["sign"] = sign;
	doc["mac"] = lock_mac;
	doc["lockStatus"] = status.c_str();
	char messageBuffer[130];
	serializeJson(doc, messageBuffer);

	if (mqttClient.publish(commandResultTopic.c_str(), 0, 0, messageBuffer) == true) {
		Serial.println("Command result sent");
	} else {
		Serial.println("Error sending command result");
		mqttClient.disconnect();
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

char const hex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

std::string toHex(std::string status) {
	char* bytes = (char *)status.c_str();
	int size = 20;
	std::string str;
	for (int i = 0; i < size; ++i) {
		const char ch = bytes[i];
		str.append(&hex[(ch  & 0xF0) >> 4], 1);
		str.append(&hex[ch & 0xF], 1);
	}
	return str;
}


void sendBlePayload(char* mqttMessage, size_t commandLength) {
	isSending = true;
	pBLEScan->stop();
	while (pBLEScan->isScanning()) {
		delay(100);
	}
	bool result = false;
	String error = "";
	std::string status = "";
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
							Serial.println("Write successful, reading status.");
							uint16_t statusCharactUuid = strtol(statusCharacteristicUUID, NULL, 0);
							NimBLERemoteCharacteristic* pStatusCharacteristic = pRemoteService->getCharacteristic(NimBLEUUID(statusCharactUuid));
							if (pStatusCharacteristic == nullptr) {
								Serial.println("Failed to get status characteristic.");
								error = "FAILED TO GET STATUS CHARACTERISTIC";
							} else {
								time_t timestamp = (time_t) 0;
								int tries = 10;
								while (tries > 0 && (status.empty() || (0 == status.compare(status.length() - 2, 2, "00")))) {
									if (tries > 0) {
										delay(700);
									}
									std::string readStatus = pStatusCharacteristic->readValue(&timestamp);
									if (readStatus.empty()) {
										tries = 0;
										error = "FAILED TO READ STATUS";
										Serial.println("Failed to read status characteristic.");
									} else {
										Serial.println("Read status successful.");
										error = "";
										result = true;
										status = toHex(readStatus);
										Serial.println(status.c_str());
									}
									tries--;
								}
								pStatusCharacteristic = NULL;
							}
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
	sendCommandResult(result, error, sign, status);
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
			Serial.println("Cannot report: mqtt disconnected");
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
	digitalWrite(LED_BUILTIN, !LED_ON);
	mqttRetryAttempts = 0;
	
	if (mqttClient.publish(availabilityTopic.c_str(), 0, 1, "CONNECTED") == true) {
		Serial.print("Success sending message to topic:\t");
		Serial.println(availabilityTopic);
		mqttClient.subscribe(commandTopic.c_str(), 2);
		sendTelemetry();
	} else {
		Serial.println("Error sending message");
		mqttClient.disconnect();
	}
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
	digitalWrite(LED_BUILTIN, LED_ON);
	handleMqttDisconnect();
}

String processor(const String& var) {
	if (var == "VERSION") {
		return String(version);
	}
	if (var == "WIFI") {
		return wifi_ssid;
	}
	if (var == "MQTT_IP") {
		return mqtt_ip;
	}
	if (var == "MQTT_PORT") {
		return String(mqtt_port);
	}
	if (var == "MQTT_USER") {
		return mqtt_user;
	}
	if (var == "MQTT_TOPIC") {
		return mqtt_topic;
	}
	if (var == "LOCK_MAC") {
		return lock_mac;
	}
	return String();
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void mainSetup() {
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LED_ON);

	mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
	wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

	WiFi.onEvent(WiFiEvent);

	mqttClient.onConnect(onMqttConnect);
	mqttClient.onDisconnect(onMqttDisconnect);
	mqttClient.onMessage(onMqttMessage);

	mqttClient.setServer(mqtt_ip.c_str(), mqtt_port);
	mqttClient.setWill(availabilityTopic.c_str(), 0, 1, "DISCONNECTED");
	mqttClient.setKeepAlive(60);

	connectToWifi();

  	NimBLEDevice::init("");
	NimBLEDevice::setPower(ESP_PWR_LVL_P9);
	pClient = NimBLEDevice::createClient();
  	pBLEScan = NimBLEDevice::getScan();
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
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send_P(200, "text/html", reset_html, processor);
	});
	server.on("/reset", HTTP_GET, [] (AsyncWebServerRequest *request) {
		preferences.begin(main_prefs, false);
		preferences.clear();
		preferences.end();
		preferences.begin(wifi_prefs, false);
		preferences.clear();
		preferences.end();
		request->send_P(200, "text/html", confirm_html, processor);
		digitalWrite(LED_BUILTIN, LED_ON);
		delay(3000);
		ESP.restart();
	});
}

void mainLoop() {
	TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
	TIMERG0.wdt_feed=1;
	TIMERG0.wdt_wprotect=0;
}

void configSetup() {
	Serial.print("Setting AP (Access Point)…");
  	WiFi.softAP(ap_ssid);
	IPAddress IP = WiFi.softAPIP();
  	Serial.print("AP IP address: ");
  	Serial.println(IP);
  	// Send web page with input fields to client
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
		request->send_P(200, "text/html", index_html, processor);
	});

	// Send a GET request to <ESP_IP>/get?input1=<inputMessage>
	server.on("/config", HTTP_GET, [] (AsyncWebServerRequest *request) {
		String wifi_ssid_param = request->getParam(PARAM_INPUT_1)->value();
		String wifi_pwd_param = request->getParam(PARAM_INPUT_2)->value();
		String mqtt_ip_param = request->getParam(PARAM_INPUT_3)->value();
		String mqtt_port_param = request->getParam(PARAM_INPUT_4)->value();
		String mqtt_user_param = request->getParam(PARAM_INPUT_5)->value();
		String mqtt_pass_param = request->getParam(PARAM_INPUT_6)->value();
		String mqtt_topic_param = request->getParam(PARAM_INPUT_7)->value();
		String lock_mac_param = request->getParam(PARAM_INPUT_8)->value();
		preferences.begin(main_prefs, false);
		preferences.clear();
		preferences.putString(wifi_ssid_pref, wifi_ssid_param);
		preferences.putString(wifi_pwd_pref, wifi_pwd_param);
		preferences.putString(mqtt_ip_pref, mqtt_ip_param);
		preferences.putInt(mqtt_port_pref, mqtt_port_param.toInt());
		preferences.putString(mqtt_user_pref, mqtt_user_param);
		preferences.putString(mqtt_pass_pref, mqtt_pass_param);
		preferences.putString(mqtt_topic_pref, mqtt_topic_param);
		preferences.putString(lock_mac_pref, lock_mac_param);
		preferences.end();
		preferences.begin(wifi_prefs, false);
		preferences.clear();
		preferences.end();

		request->send_P(200, "text/html", confirm_html, processor);
		delay(3000);
		ESP.restart();
	});
}

bool readPrefs() {
	preferences.begin(main_prefs, true);
	wifi_ssid = preferences.getString(wifi_ssid_pref);
	wifi_pwd = preferences.getString(wifi_pwd_pref);
	mqtt_ip = preferences.getString(mqtt_ip_pref);
	mqtt_port = preferences.getInt(mqtt_port_pref);
	mqtt_user = preferences.getString(mqtt_user_pref);
	mqtt_pass = preferences.getString(mqtt_pass_pref);
	mqtt_topic = preferences.getString(mqtt_topic_pref);
	lock_mac = preferences.getString(lock_mac_pref);
	preferences.end();

	advertTopic = mqtt_topic + "/adv";
	commandResultTopic = mqtt_topic + "/command_result";
	availabilityTopic = mqtt_topic + "/availability";
	commandTopic = mqtt_topic + "/command";
	telemetryTopic = mqtt_topic + "/tele";

	return wifi_ssid != "" 
		&& mqtt_ip != "" 
		&& mqtt_port != 0 
		&& mqtt_topic != "" 
		&& lock_mac != "";
}

void handleUpdate(AsyncWebServerRequest *request) {
  request->send(200, "text/html", update_html);
}

void handleDoUpdate(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
	if (!index){
		Serial.println("Update");
		content_len = request->contentLength();
		if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
			Update.printError(Serial);
		}
	}

	if (Update.write(data, len) != len) {
		Update.printError(Serial);
	}

	if (final) {
		if (!Update.end(true)){
			Update.printError(Serial);
		} else {
			Serial.println("Update complete");
			Serial.flush();
			ESP.restart();
		}
	}
}

void printProgress(size_t prg, size_t sz) {
  	Serial.printf("Progress: %d%%\n", (prg*100)/content_len);
}

void setup() {
	Serial.begin(115200);
	isSetUp = readPrefs();
	if (isSetUp) {
	  	mainSetup();
	} else {
		configSetup();
	}
	server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){handleUpdate(request);});
	server.on("/doUpdate", HTTP_POST,
		[](AsyncWebServerRequest *request) {},
		[](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data,
					size_t len, bool final) {
							handleDoUpdate(request, filename, index, data, len, final);
						}
	);
	server.onNotFound(notFound);
	server.begin();
	Update.onProgress(printProgress);
}

void loop() {
	if (isSetUp) {
		mainLoop();
	}
}
