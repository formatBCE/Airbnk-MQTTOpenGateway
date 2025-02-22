#include "airbnk_gateway.h"

namespace esphome {
namespace airbnk_gateway {

NimBLEScan* pScan;
NimBLEClient* nimBleClient;
TaskHandle_t nimScan;
NimBLEAddress lockAddress;
bool isSending = false;

uint16_t shortServiceUuid = serviceUUID;
uint16_t shortCharactUuid = characteristicUUID;
uint16_t statusCharactUuid = statusCharacteristicUUID;

char const hex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

std::string capitalizeString(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::toupper(c); });
    return s;
}

int fromHex(uint8_t *dest, const char *src, int maxlen) {
    int srclen = strlen(src) / 2;
    if (srclen > maxlen) return 0;
    memset(dest, 0, maxlen);

    for (int i = 0; i < srclen; i++) {
        char t[3] = { src[i * 2], src[i * 2 + 1], 0 };
        if (!isalnum(t[0]) || !isalnum(t[1])) return 0;
        int byte = strtol(t, NULL, 16);
        *dest++ = byte;
    }
    return srclen;
}

std::string toHex(std::string status) {
    char* bytes = (char *)status.c_str();
    int size = 20;
    std::string str;
    for (int i = 0; i < size; ++i) {
        const char ch = bytes[i];
        str.append(&hex[(ch & 0xF0) >> 4], 1);
        str.append(&hex[ch & 0xF], 1);
    }
    return str;
}

void scanLock(void* parameter) {
    while (1) {
        if (!pScan->isScanning()) {
            ESP_LOGD("airbnk_mqtt", "Start scanning...");
            pScan->start(0, false, true);
        }
        ESP_LOGD("airbnk_mqtt", "BLE scan heartbeat");
        delay(5000);
    }
}

AirbnkGateway::AirbnkGateway(std::string mac_address, std::string mqtt_topic) {
    advert_topic = mqtt_topic + "/adv";
    command_topic = mqtt_topic + "/command";
    command_result_topic = mqtt_topic + "/command_result";
    lock_mac = capitalizeString(mac_address);
}

void AirbnkGateway::setup() {
    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new BleAdvertisedDeviceCallbacks(*this), true);
    pScan->setInterval(scanInterval);
    pScan->setWindow(scanWindow);
    pScan->setActiveScan(false);
    pScan->setMaxResults(0);
    nimBleClient = NimBLEDevice::createClient();
    subscribe_json(command_topic, &AirbnkGateway::on_command);
    xTaskCreatePinnedToCore(scanLock, "BLE Scan", 4096, pScan, 1, &nimScan, 1);
}

bool AirbnkGateway::reportDevice(NimBLEAdvertisedDevice& advertisedDevice) {
    NimBLEAddress address = advertisedDevice.getAddress();
    std::string mac_address = capitalizeString(address.toString().c_str());
    if (mac_address != lock_mac) return false;

    lockAddress = address;
    std::string manData = advertisedDevice.getManufacturerData();
    std::string pHex = NimBLEUtils::dataToHexString((uint8_t*)manData.data(), manData.length());
    ESP_LOGD("airbnk_mqtt", "Sending adv");

    int RSSI = advertisedDevice.getRSSI();
    publish_json(advert_topic.c_str(), [=](JsonObject root) {
        root["mac"] = mac_address;
        root["rssi"] = RSSI;
        root["data"] = pHex;
    });

    return true;
}

void AirbnkGateway::sendCommandResult(bool success, std::string error, int sign, std::string status) {
    while (!esphome::network::is_connected) delay(100);
    
    publish_json(command_result_topic.c_str(), [=](JsonObject root) {
        root["success"] = success;
        root["error"] = error;
        root["sign"] = sign;
        root["mac"] = lock_mac;
        root["lockStatus"] = status;
    });

    ESP_LOGD("airbnk_mqtt", success ? "Operation successful" : error.c_str());
}

void AirbnkGateway::on_command(JsonObject root) {
    ESP_LOGD("airbnk_mqtt", "Got command");
    if (isSending) {
        ESP_LOGD("airbnk_mqtt", "Already sending something, abort");
        return;
    }
    sendBlePayload(root);
}

AirbnkGateway::BleAdvertisedDeviceCallbacks::BleAdvertisedDeviceCallbacks(AirbnkGateway& parent)
    : parent_(parent) {}

void AirbnkGateway::BleAdvertisedDeviceCallbacks::onResult(NimBLEAdvertisedDevice* device) {
    parent_.reportDevice(*device);
}

}  // namespace airbnk_gateway
}  // namespace esphome