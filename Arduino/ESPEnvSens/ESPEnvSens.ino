/*  *****************   ESPEnvSens **************************************
 *  Author: jpichlbauer
 *  Date: 2016-09-26
 *  
 *  Arduino Sketch for environmental monitoring using
 *  ESP8266, DHT22 and BMP180 devices.
 *  Uses MQTT to push the Data to a broker over WLAN and
 *  ESPs DeepSleep functionality to maximise battery lifetime.
 *  Tested with cheap ESP-12 and ESP-201 Modules, but will
 *  probably work with any other ESP8266 device.
 *  
 *  NOTE concerning Hardware setup:
 *  ==================================
 *  To be able to use DeepSleep, you will have to wire GPIO16
 *  to RESET. RealTimeClock of ESP8266 will trigger a RESET
 *  after DeepSleepTime to wake the ESP.
 *  
 *  NOTE for flashing ESP modules (might only be true for my modules):
 *  ==================================================================
 *  o Flashing ESP-201 ONLY works with external power source!
 *  o Flashing ESP-12 ONLY works when using UART-Adapters power source!
 *  ********************************************************************
 */

// Include the libraries we need
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// ESP Function to read battery voltage
extern "C" {
  uint16 readvdd33(void);
  #include "user_interface.h"
}

/*
 * Defines for DeepSleep and Serial Output
 */

// ****** DEBUG Settings **********
// Enable (define) ESP deepSleep in normal Operation
#define DEEPSLEEP

// Enable (define) Serial Port for Debugging
#define SerialEnabled
// ********************************


/*
 * ****** INDOOR / OUTDOOR Sensor Selection and settings *************
 * INDOOR Sensor only uses DHT22 for Temperature and rel. humidity
 * OUTDOOR Sensor additionally uses BMP180 for barometric pressure
 */

#define OUTDOOR //if not defined, INDOOR sensor will be compiled

#ifdef OUTDOOR
  // OUTDOOR Sensor Settings
  // ===========================================
  #define mqtt_Client_Name "temp-out.mik"
  #define humidity_topic "HB7/Outdoor/RH"
  #define temperature_topic "HB7/Outdoor/Temp"
  #define airpress_topic "HB7/Outdoor/AirPress"
  #define voltage_topic "HB7/Outdoor/Vbat"
  #define status_topic "HB7/Outdoor/Status"
  #define Interval_topic "HB7/Outdoor/Interval"
  
  // Default DeepSleep Time in Minutes
  int DeepSleepTime = 15;

  // BMP180 barometric pressure sensor
  // I2C Pin definitions
  #define I2C_SDA 2
  #define I2C_SCL 14
  // Setup BMP instance
  Adafruit_BMP085 bmp;
  // Air Pressure Variable
  float QFE = 105000;

#else
  // INDOOR Sensor Settings
  // =============================================
  #define mqtt_Client_Name "temp-wz.mik"
  #define humidity_topic "HB7/Indoor/WZ/RH"
  #define temperature_topic "HB7/Indoor/WZ/Temp"
  #define voltage_topic "HB7/Indoor/WZ/Vbat"
  #define status_topic "HB7/Indoor/WZ/Status"
  #define Interval_topic "HB7/Indoor/WZ/Interval"
  
  // Default DeepSleep Time in Minutes
  int DeepSleepTime = 15;

  // Status LED on GPIO2 (LED inverted!)
  #define USELED //does not work on ESP-201 boards (no LED) - do not define this to disable LED signalling
  #define LED 2
  #define LEDON LOW
  #define LEDOFF HIGH

#endif

/* ***************************************************************
 * ********** More COMMON SETTINGS and Variables *****************
*/
// WLAN Network SSID and PSK
WiFiClient EnvSensWiFi;
const char* ssid = "***";
const char* password = "***";

// MQTT Settings
#define mqtt_server "192.168.152.7"
const char* OK_Status = "DataUpdated";
// Maximum connection attempts to MQTT broker before going to sleep
const int MaxConnAttempts = 3;
// Message buffer for incoming Data from MQTT subscriptions
char message_buff[20];

// Data wire of DHT22 is plugged into port GPIO4 on the ESP8266
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN 4

// Setup a DHT instance
DHT dht(DHTPIN, DHTTYPE);

// Temperature from DHT22
float Temp;

// Rel. Humidity from DHT22
float RH;

// Battery Voltage (milliVolt)
int vdd = 3123;

// OS Timer for Software Watchdog
os_timer_t WDTimer;
bool ProgramResponding = true;
// WDT will trigger every 10 seconds
#define WDTIMEOUT 10000


/*
 * Callback Functions
 * ========================================================================
 */

// Watchdog Timer Callback function
void WDTCallback(void *pArg)
{
  if (ProgramResponding)
  {
    // If ProgramResponding is not reset to true before next WDT trigger..
    ProgramResponding = false;
    return;
  }
  // ..program is probably dead, go to DeepSleep
  #ifdef USELED
  // signal SOS
  digitalWrite(LED, LEDOFF);
  delay(250);
  ToggleLed(LED,200,6);
  ToggleLed(LED,600,6);
  ToggleLed(LED,200,6);
  #endif
  #ifdef SerialEnabled
  Serial.println("Watchdog Timer detected program not responding, going to sleep!");
  #endif
  #ifdef DEEPSLEEP
  ESP.deepSleep(DeepSleepTime * 60000000);
  #endif
  delay(100);
}

//MQTT Subscription callback function
void MqttCallback(char* topic, byte* payload, unsigned int length)
{
  int i = 0;
  // create character buffer with ending null terminator (string)
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  String msgString = String(message_buff);

  #ifdef SerialEnabled
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(msgString);
  #endif

  // If message is an integer between 5 and 120, use it as new DeepSleepTime
  int IntPayLd = msgString.toInt();
  if ((IntPayLd >= 5) && (IntPayLd <= 120))
  {
    #ifdef SerialEnabled
    Serial.print("New DeepSleep interval [min]: ");
    Serial.println(IntPayLd);
    #endif
    DeepSleepTime = IntPayLd;
  }
}


/*
 * Setup PubSub Client instance
 * ===================================
 * must be done before setting up ConnectToBroker function and after MqttCallback Function
 * to avoid compilation errors
 */
PubSubClient mqttClt(mqtt_server,1883,MqttCallback,EnvSensWiFi);


/*
 * Common Functions
 * =================================================
 */

bool ConnectToBroker()
{
  bool RetVal = false;
  int ConnAttempt = 0;
  // Try to connect x times, then return error
  while (ConnAttempt < MaxConnAttempts)
  {
    #ifdef SerialEnabled
    Serial.print("Connecting to MQTT broker..");
    #endif
    // Attempt to connect
    if (mqttClt.connect(mqtt_Client_Name))
    {
      #ifdef SerialEnabled
      Serial.println("connected");
      #endif
      RetVal = true;
      break;
    } else {
      #ifdef SerialEnabled
      Serial.print("failed, rc=");
      Serial.println(mqttClt.state());
      Serial.println("Sleeping 5 seconds..");
      #endif
      // Program is still running
      ProgramResponding = true;
      // Wait 1 seconds before retrying
      delay(1000);
      ConnAttempt++;
    }
  }
  return RetVal;
}


void ToggleLed (int PIN,int WaitTime,int Count)
{
  // Toggle digital output
  for (int i=0; i < Count; i++)
  {
   digitalWrite(PIN, !digitalRead(PIN));
   delay(WaitTime); 
  }
}

/*
 * Setup function. Preparing WLAN, MQTT and sensors
 * ==================================================================================
 */
void setup(void)
{
  #ifdef SerialEnabled
  // start serial port and digital Outputs
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  #ifdef OUTDOOR
    Serial.println("ESP8266 WLAN Environmental Monitor - HB7 OUTDOOR");
  #else
    Serial.println("ESP8266 WLAN Environmental Monitor - HB7 INDOOR");
  #endif
  #endif
  #ifdef USELED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LEDOFF);
  #endif
  
  // Read Battery voltage from ESP - only works while WiFi not connected (?)
  // should't be an issue as setup runs after every DeepSleep
  vdd = readvdd33();

  // Assign function and arm Watchdog Timer
  os_timer_setfn(&WDTimer, WDTCallback, NULL);
  os_timer_arm(&WDTimer, WDTIMEOUT, true);
  ProgramResponding = true;

  // OUTDOOR Sensor only
  #ifdef OUTDOOR
    // Initialize I2C
    Wire.begin(I2C_SDA,I2C_SCL);
    
    // Initialize BMP180
    if (!bmp.begin())
    {
    #ifdef SerialEnabled
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    #endif
    delay(10000);
    }
  #endif

  // Initialize DHT22
  dht.begin();

  // Connect to WiFi network
  #ifdef SerialEnabled
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  #endif   
  WiFi.begin(ssid, password);
   
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef SerialEnabled
    Serial.print(".");
    #endif
  }
  #ifdef SerialEnabled
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Device IP Address: ");
  Serial.println(WiFi.localIP());
  #endif

  // Setup MQTT Connection to broker and subscribe to topic
  if (ConnectToBroker())
  {
    #ifdef SerialEnabled
    Serial.println("Connected to MQTT broker.");
    #endif    
    // Subscribe to Interval Topic
    if (mqttClt.subscribe(Interval_topic))
    {
      #ifdef SerialEnabled
      Serial.print("Subscribed to ");
      Serial.println(Interval_topic);
      #endif
      delay(200);
      // try to get a message from topic..
      mqttClt.loop();
    }
    else
    {
      #ifdef SerialEnabled
      Serial.print("Failed to subscribe to ");
      Serial.println(Interval_topic);
      #endif
      // This actually doesn't matter, program just can't dynamically update DeepSleepTime
      delay(100);
    }
  }
  else
  {
    // Failed to connect to broker
    #ifdef DEEPSLEEP
    ESP.deepSleep(DeepSleepTime * 60000000);
    #endif
    #ifdef SerialEnabled
    Serial.println("3 connection attempts to broker failed, sleeping 3 seconds..");
    #endif
    delay(3000);
    return;
  }
  #ifdef USELED
  // Setup finished, WLAN and broker connected - blink once
  ToggleLed(LED,200,2);
  #endif
}


/*
 * Main loop
 * ====================================================================
 */
void loop(void)
{
  // WDT: Program is running
  ProgramResponding = true;

  // Read Temperature from DHT22
  // Try 3 times max. if "nan" for any reading
  delay(500);
  for (int i=0; i <= 2; i++){
    // Read DHT22 Sensors
    RH = dht.readHumidity();
    Temp = dht.readTemperature();
    if (isnan(RH) || isnan(Temp)) {
      delay(1500);
      // WDT: Program is still running..
      ProgramResponding = true;
      continue;
    }
    else {
      break;
    }
  }

  // WDT: Program is still running..
  ProgramResponding = true;

  //OUTDOOR Sensor only
  // Read barometric pressure from BMP180
  #ifdef OUTDOOR
    QFE = bmp.readPressure();
    #ifdef SerialEnabled
    Serial.print("AirPress=");
    Serial.println(QFE); 
    #endif
  #endif

  #ifdef USELED
  // Sensor data gathered - blink twice
  ToggleLed(LED,200,4);
  #endif

  // calling the loop function also verifies connectivity to MQTT broker, return FALSE if disconnected
  // the loop function will also gather data from subscribed topics
  if (mqttClt.loop())
  {
    #ifdef SerialEnabled
    Serial.print("TempC = ");
    Serial.println(Temp);
    Serial.print("RH = ");
    Serial.println(RH);
    Serial.print("Vbat = ");
    Serial.println(vdd);
    Serial.println("Sending data to MQTT broker..");
    #endif
  
    // WDT: Program is still running..
    ProgramResponding = true;
    
    // Publish retained messages to broker
    mqttClt.publish(temperature_topic, String(Temp).c_str(), true);
    mqttClt.publish(humidity_topic, String(RH).c_str(), true);
    mqttClt.publish(voltage_topic, String(vdd).c_str(), true);
    mqttClt.publish(status_topic, String(OK_Status).c_str(), true);
    // OUTDOOR Sensor only
    #ifdef OUTDOOR
      mqttClt.publish(airpress_topic, String(QFE).c_str(), true);
    #endif
    // Done, disconnect from broker
    mqttClt.disconnect();
    #ifdef SerialEnabled
    Serial.print("Done, going to sleep for ");
    Serial.print(DeepSleepTime);
    Serial.println(" minutes.");
    #endif
    
    #ifdef USELED
    // Messages published - single long blink
    ToggleLed(LED,500,2);
    #else
    delay(500);
    #endif
    
    // initiate deep sleep
    #ifdef DEEPSLEEP
    // Disconnect WiFi and go to Sleep
    WiFi.disconnect();
    ESP.deepSleep(DeepSleepTime * 60000000);
    delay(100);
    #endif
    #ifdef SerialEnabled
    Serial.println("DeepSleep disabled, sleeping 20 seconds..");
    #endif
    delay(20000);
  }
  else
  {
    // not connected to broker, this should not happen
    #ifdef USELED
    // signal SOS
    digitalWrite(LED, LEDOFF);
    delay(250);
    ToggleLed(LED,200,6);
    ToggleLed(LED,600,6);
    ToggleLed(LED,200,6);
    #endif
    #ifdef SerialEnabled
    Serial.println("FATAL: connection to broker lost in main loop!");
    #endif
    #ifdef DEEPSLEEP
    // disconnect WiFi and go to sleep
    WiFi.disconnect();
    ESP.deepSleep(DeepSleepTime * 60000000);
    #endif
    delay(100);
  }
}
