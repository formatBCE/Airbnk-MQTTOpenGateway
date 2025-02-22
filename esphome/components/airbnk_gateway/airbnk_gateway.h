#pragma once

#include "esphome.h"
#include <mqtt_component.h>
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

namespace esphome {
namespace airbnk_gateway {

#define serviceUUID 0xFFF0
#define characteristicUUID 0xFFF2
#define statusCharacteristicUUID 0xFFF3

#define scanInterval 0x80
#define scanWindow 0x40

class AirbnkGateway : public esphome::mqtt::MQTTComponent {
public:
    AirbnkGateway(std::string mac_address, std::string mqtt_topic);

    void setup() override;

    void on_command(JsonObject root);
    void sendBlePayload(JsonObject &root);
    void sendCommandResult(bool success, std::string error, int sign, std::string status);
    bool reportDevice(NimBLEAdvertisedDevice& advertisedDevice);

    class BleAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
        AirbnkGateway& parent_;
    public:
        BleAdvertisedDeviceCallbacks(AirbnkGateway& parent);
        void onResult(NimBLEAdvertisedDevice* device) override;
    };

private:
    std::string advert_topic;
    std::string command_topic;
    std::string command_result_topic;
    std::string lock_mac;
};

// Utility functions
std::string capitalizeString(std::string s);
int fromHex(uint8_t *dest, const char *src, int maxlen);
std::string toHex(std::string status);
void scanLock(void* parameter);

// External variables
extern NimBLEScan* pScan;
extern NimBLEClient* nimBleClient;
extern TaskHandle_t nimScan;
extern NimBLEAddress lockAddress;
extern bool isSending;

}  // namespace airbnk_gateway
}  // namespace esphome