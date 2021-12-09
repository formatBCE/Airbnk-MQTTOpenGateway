
#define ssid "" // WiFi SSID
#define password "" // WiFi password

#define hostname "airbnk_lock" // will be used as MQTT root topic

#define mqttHost IPAddress(192, 168, 1, 1) // replace with your MQTT broker IP
#define mqttPort 1883
#define mqttUser "" // MQTT user
#define mqttPassword "" // MQTT password

#define lockMacAddress "AA:BB:CC:DD:00:11" // please fill in exactly, with uppercase letters

#define availabilityTopic hostname "/availability"
#define telemetryTopic hostname "/tele"
#define commandTopic hostname "/command"
#define commandResultTopic hostname "/command_result"



// Keep this section untouched, unless you really know what you're doing

#define LED_BUILTIN 2
#define LED_ON 0

#define serviceUUID "0xFFF0"
#define characteristicUUID "0xFFF2"

#define scanInterval 0 // Define the interval in seconds between scans
#define singleScanTime 10 // Define the duration of a single scan in seconds
#define bleScanInterval 0x80 // Used to determine antenna sharing between Bluetooth and WiFi. Do not modify unless you are confident you know what you're doing
#define bleScanWindow 0x10 // Used to determine antenna sharing between Bluetooth and WiFi. Do not modify unless you are confident you know what you're doing
