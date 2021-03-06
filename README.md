# ESPEnvSens
Arduino Sketches for ESP8266 which implement Environmental Monitoring with DHT22 / BMP180 and MQTT

The inteded use is to run on battery driven boxes, code implements ESP DeepSleep Function to maximize battery lifetime. Sensor data will be pushed to a remote MQTT broker every 5 minutes (by default).
Cheap ESP8266 boards have been used for my implementation, by now it works quite well, although i have implemented a software based watchdog timer, as the ESP hangs from time to time.
The Sensor readout interval can be configured by publishing an integer value between 5 and 120 to the "Interval" topic. The value corresponds to the ESP DeepSleep Time in minutes.
If the "Interval" topic is not available, the default DeepSleep Time is 15 minutes.

The sensor boxes use 4x AA Batteries / accumulators as a power source, a small DC/DC converter from Pololu (D24V5F3) is used to provide 3.3V to the ESP and sensors.
Note that due to this setup, the Battery voltage readout done by the code is pretty useless, as it will always report 3.3V until the battery is 100% dead and ESP will not start any longer. This could be avoided by directly reading the battery pack voltage through a suitable voltage divider (connected to the ESP ADC pin), but as i wanted to keep the hardware setup simple (see photos), i have not implemented this (yet).

The Code is pretty self-explaining and well commented (hopefully).
Both sensors are using a DHT22 for Temperature and Humidity readings, outdoor sensor uses an additional BMP180 sensor for barometric pressure reading.

Note on battery usage: The outdoor sensor (BMP180+DHT22) batteries last for ~ 5 weeks (using 4x Noname AA Alkaline Batteriesand Interval of 5 minutes). It might be possible to optimize this by powering the sensors through ESP GPIOs (instead of wiring them directly to the 3.3V power source), as this will completely power off the sensors while ESP is sleeping.

Have fun,
Juergen
