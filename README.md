# Airbnk-MQTTOpenGateway
Gateway to use Airbnk locks with HA custom integration https://github.com/rospogrigio/airbnk_mqtt

1.
Get the code from repository, and flash it to ESP32 using PlatformIO or other IDE.
Or, download only latest firmware_%VERSION%.bin file, and flash using ESP flasher or other flashing tool.
Use https://github.com/formatBCE/Airbnk-MQTTOpenGateway/blob/main/partitions_singleapp.csv for partitioning reference.

2.
Place chip near the lock and power it up. Make sure you use good power adapter!

3. 
After blue LED is on, go find new WiFi network, named "AirbnkOpenGateway".
Connect to it from your PC/phone, and go to 102.168.4.1 in your browser.

4.
Configure gateway according to your environment.
Make sure you did type everything correctly before saving configuration.
Use 2.4 WiFi network.
Make sure you choose correct MAC address for lock, and MQTT topic corresponding to HA integration parameters.

5. 
After saving configuration, ESP32 will restart, connect to WiFi and start sending telemetry and lock advertisements/commands to MQTT.
Continue setup with https://github.com/rospogrigio/airbnk_mqtt

6.
Although it doesn't really matter, you may want to make DHCP reservation for gateway on your router.
If you'll want to re-configure gateway in the future, you can go to gateway web interface using it's IP in browser, and reset it from there.
Also, gateway supports OTA updates over web interface, which can be useful.

Cheers.
