# WiFi Teleoperated model car

With this repo you can control a model car using an [ESP32 microcontroller](https://amzn.eu/d/duSHIUO) and a PS Joystick (or any other pygame compatibile controller) through your home WiFi and PC.

**This project is based on [_@arms22_ streaming software](https://github.com/arms22/esp32_camera_udp_streaming)**.

## Setup

- Set your WiFi SSID and password in the config file `sdkconfig` and upload the code on the ESP32 using ESP-IDF.
- Connect the PWM cables of the car motors for steering and throttle to the ESP32 PINS 32 and 33.
- Connect your joystick to your PC so to be detected by pygame library. Use a gaming steering wheel for the best experience.
- Identify your ESP32 IP (it should be 192.168.1.152 as specified in the IDF default config) and your PC IP on your LAN.

### Usage

On your PC run

```
python3 python_receiver\receiver.py <localpc_ip> <esp32_ip>
```

While looking at the camera steaming on your PC, press R2 and L2 of your joystick to move forward and backwards, use the right joystick to steer. Have fun!
