#include "esphome.h"
#include <stdlib.h>
#include "time.h"
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/timers.h"
}
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#define serviceUUID "0xFFF0"
#define characteristicUUID "0xFFF2"
#define statusCharacteristicUUID "0xFFF3"

#define scanInterval 0x80 // Used to determine antenna sharing between Bluetooth and WiFi. Do not modify unless you are confident you know what you're doing
#define scanWindow 0x40 // Used to determine antenna sharing between Bluetooth and WiFi. Do not modify unless you are confident you know what you're doing

NimBLEScan* pScan;
NimBLEClient* nimBleClient;
TaskHandle_t nimScan;
NimBLEAddress lockAddress;
bool isSending;

uint16_t shortServiceUuid = strtol(serviceUUID, NULL, 0);
uint16_t shortCharactUuid = strtol(characteristicUUID, NULL, 0);

std::string capitalizeString(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                [](unsigned char c){ return std::toupper(c); });
    return s;
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

void scanLock(void* parameter) {
    while (1) {
        if (!pScan->isScanning()) {
            ESP_LOGD("airbnk_mqtt", "Start scanning...");
            pScan->start(0, nullptr, false);
        }
        ESP_LOGD("airbnk_mqtt", "BLE scan heartbeat");
        delay(5000);
    }
}

class AirbnkGatewayNodeComponent : public Component, public CustomMQTTDevice {

    bool reportDevice(NimBLEAdvertisedDevice& advertisedDevice) {
        NimBLEAddress address = advertisedDevice.getAddress();
        std::string mac_address = capitalizeString(address.toString().c_str());
        if (mac_address != lock_mac.c_str()) {
            return false;
        }

        lockAddress = address;

        std::string manData = advertisedDevice.getManufacturerData();
        char *pHex = NimBLEUtils::buildHexData(nullptr, (uint8_t*)manData.data(), manData.length());
        ESP_LOGD("airbnk_mqtt", "Sending adv");
        int RSSI = advertisedDevice.getRSSI();
        publish_json(advert_topic.c_str(), [=](JsonObject root) {
            root["mac"] = mac_address;
            root["rssi"] = RSSI;
            root["data"] = pHex;
        }, 1, false);
        delete pHex;
        return true;
    }

    void sendCommandResult(boolean success, std::string error, int sign, std::string status) {
        while (!is_connected()) { // WiFi most likely was disconnected during command write. Meh.
            delay(100);
        }
        publish_json(command_result_topic.c_str(), [=](JsonObject root) {
            root["success"] = success;
            root["error"] = error.c_str();
            root["sign"] = sign;
            root["mac"] = lock_mac;
            root["lockStatus"] = status.c_str();
        }, 1, false);
        if (success) {
            ESP_LOGD("airbnk_mqtt", "Operation successful");
        } else {
            ESP_LOGD("airbnk_mqtt", error.c_str());
        }
    }

    void sendBlePayload(JsonObject &root) {
        isSending = true;
        pScan->stop();
        while (pScan->isScanning()) {
            delay(100);
        }
        vTaskDelete(nimScan);
        bool result = false;
        std::string error = "";
        std::string status = "";
        int sign = root["sign"];
        const char* cmnd1 = root["command1"];
        const char* cmnd2 = root["command2"];
        uint8_t command1[20];
        uint8_t command2[20];
        int len1 = fromHex(command1, cmnd1, 20);
        int len2 = fromHex(command2, cmnd2, 20);
        ESP_LOGD("airbnk_mqtt", "Sending to:");
        ESP_LOGD("airbnk_mqtt", lockAddress.toString().c_str());
        int retry = 1;
        while (retry < 5 && result == false) {
            if (retry > 1) {
                delay(500);
                ESP_LOGD("airbnk_mqtt", "Retrying sending");
            }
            if (nimBleClient->connect(lockAddress, true)) {
                ESP_LOGD("airbnk_mqtt", "Connected to lock.");
                NimBLERemoteService* pRemoteService = nimBleClient->getService(BLEUUID(shortServiceUuid));
                if (pRemoteService == nullptr) {
                    ESP_LOGD("airbnk_mqtt", "Failed to get service.");
                    error = "FAILED TO GET SERVICE";
                } else {
                    NimBLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(NimBLEUUID(shortCharactUuid));
                    if (pRemoteCharacteristic == nullptr) {
                        ESP_LOGD("airbnk_mqtt_scanner", "Failed to get characteristic.");
                        error = "FAILED TO GET CHARACTERISTIC";
                    } else {
                            if (
                            pRemoteCharacteristic->writeValue(command1, len1, true)
                            && pRemoteCharacteristic->writeValue(command2, len2, true)
                            ) {
                                error = "";
                                result = true;
                            } else {
                                error = "FAILED TO WRITE";
                                ESP_LOGD("airbnk_mqtt_scanner", "Failed to write characteristic.");
                            }
                            delete pRemoteCharacteristic;
                        }
                        delete pRemoteService;
                }
            nimBleClient->disconnect();
            } else {
                ESP_LOGD("airbnk_mqtt", "Failed to connect to lock.");
                error = "FAILED TO CONNECT";
            }
            retry++;
        }
        isSending = false;
        sendCommandResult(result, error, sign, status);
        xTaskCreatePinnedToCore(scanLock, "BLE Scan", 4096, pScan, 1, &nimScan, 1);
    }

    void on_command(JsonObject root) {
        ESP_LOGD("airbnk_mqtt", "Got command");
        if (isSending) {
            ESP_LOGD("airbnk_mqtt", "Already sending something, abort");
            return;
        }
        sendBlePayload(root);
    }
    
    class BleAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {

        AirbnkGatewayNodeComponent& parent_;

        public:
            BleAdvertisedDeviceCallbacks(AirbnkGatewayNodeComponent& parent) : parent_(parent) {}

            void onResult(NimBLEAdvertisedDevice* device) {
                parent_.reportDevice(*device);
            }
    };
    
 public:
    std::string advert_topic;
    std::string command_topic;
    std::string command_result_topic;
    std::string lock_mac;
    AirbnkGatewayNodeComponent(std::string mac, std::string root_topic) {
        advert_topic = root_topic + "/adv";
        command_topic = root_topic + "/command";
        command_result_topic = root_topic + "/command_result";
        lock_mac = capitalizeString(mac);
    }

    void setup() override {
        NimBLEDevice::init("");
        NimBLEDevice::setPower(ESP_PWR_LVL_P9);
        pScan = NimBLEDevice::getScan();
        pScan->setAdvertisedDeviceCallbacks(new BleAdvertisedDeviceCallbacks(*this), true);
        pScan->setInterval(scanInterval);
        pScan->setWindow(scanWindow);
        pScan->setActiveScan(false);
        pScan->setMaxResults(0);
        nimBleClient = NimBLEDevice::createClient();
        subscribe_json(command_topic, &AirbnkGatewayNodeComponent::on_command);
        xTaskCreatePinnedToCore(scanLock, "BLE Scan", 4096, pScan, 1, &nimScan, 1);
    }
};
