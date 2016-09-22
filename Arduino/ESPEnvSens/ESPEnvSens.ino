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
 * Variables and configuration
 */

// ****** DEBUG Settings **********
// Enable (define) ESP deepSleep in normal Operation
#define DEEPSLEEP

// Enable (define) Serial Port for Debugging
#define SerialEnabled
// ********************************

// ****** INDOOR / OUTDOOR Sensor Selection and settings *************
#define OUTDOOR //if not defined, INDOOR sensor will be compiled

#ifdef OUTDOOR
  // OUTDOOR Sensor Settings
  // BMP180 barometric pressure sensor
  // I2C Pin definitions
  #define I2C_SDA 2
  #define I2C_SCL 14
  // Setup BMP instance
  Adafruit_BMP085 bmp;
  // Air Pressure Variable
  float QFE = 105000;
  
  //WLAN Configuration
  WiFiClient Outdoorpj;
  const char* ssid = "***";
  const char* password = "****";
  
  // MQTT Stuff
  PubSubClient mqttClt(Outdoorpj);
  #define mqtt_server "192.168.152.7"
  #define mqtt_Client "temp-out.mik"
  #define humidity_topic "HB7/Outdoor/RH"
  #define temperature_topic "HB7/Outdoor/Temp"
  #define airpress_topic "HB7/Outdoor/AirPress"
  #define voltage_topic "HB7/Outdoor/Vbat"
  #define status_topic "HB7/Outdoor/Status"
  #define Interval_topic "HB7/Indoor/WZ/Interval"
  const char* OK_Status = "DataUpdated";
  
  // Default DeepSleep Time in Minutes
  int DeepSleepTime = 15;


#else
  // INDOOR Sensor Settings
  //WLAN Configuration
  WiFiClient IndoorWZ;
  const char* ssid = "***";
  const char* password = "******";
  
  // MQTT Stuff
  PubSubClient mqttClt(IndoorWZ);
  #define mqtt_server "192.168.152.7"
  #define mqtt_Client "temp-wz.mik"
  #define humidity_topic "HB7/Indoor/WZ/RH"
  #define temperature_topic "HB7/Indoor/WZ/Temp"
  #define voltage_topic "HB7/Indoor/WZ/Vbat"
  #define status_topic "HB7/Indoor/WZ/Status"
  #define Interval_topic "HB7/Indoor/WZ/Interval"
  const char* OK_Status = "DataUpdated";
  
  // Default DeepSleep Time in Minutes
  int DeepSleepTime = 15;

#endif
// ********************************************************

// ********** More COMMON SETTINGS *****************

// Data wire of DHT22 is plugged into port GPIO4 on the ESP8266
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN 4

// Status LED on GPIO2 (LED inverted!)
//#define USELED //does not work on ESP-201 boards - do not define this to disable LED signalling
#define LED 2
#define LEDON LOW
#define LEDOFF HIGH

// OS Timer for Software Watchdog
os_timer_t WDTimer;
bool ProgramResponding = true;
// WDT will trigger every 5 seconds
#define WDTIMEOUT 5000

// Setup a DHT instance
DHT dht(DHTPIN, DHTTYPE);

// Temperature from DHT22
float Temp;

// Rel. Humidity from DHT22
float RH;

// Battery Voltage (milliVolt)
int vdd = 3123;

// Maximum connection attempts to MQTT broker before going to sleep
const int MaxConnAttempts = 3;

// *****************************************************


/*
 * Functions
 */

bool ConnectToBroker()
{
  bool RetVal = false;
  int ConnAttempt = 0;
  // Try to connect x times, then return error
  while (ConnAttempt < MaxConnAttempts)
  {
    #ifdef SerialEnabled
    Serial.print("Attempting MQTT connection...");
    #endif
    // Attempt to connect
    if (mqttClt.connect(mqtt_Client))
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


// Watchdog Timer Callback function
void WDTCallback(void *pArg)
{
  if (ProgramResponding)
  {
    // If ProgramResponding is not reset to true before next WDT trigger..
    ProgramResponding = false;
    return;
  }
  // ..program is proably dead, go to DeepSleep
  #ifdef USELED
  // signal SOS
  digitalWrite(LED, LEDOFF);
  delay(250);
  ToggleLed(LED,100,6);
  ToggleLed(LED,200,6);
  ToggleLed(LED,100,6);
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
  #ifdef SerialEnabled
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  #endif

  String s = String((char*)payload);
  int IntPayLd = s.toInt();
  if ((IntPayLd >= 5) && (IntPayLd <= 120))
  {
    #ifdef SerialEnabled
    Serial.print("New DeepSleep interval [min]: ");
    Serial.println(IntPayLd);
    #endif
    DeepSleepTime = IntPayLd;
  }
}


// ===================================================================================================================
/*
 * Setup function. Here we do the basics
 */
void setup(void)
{
  #ifdef SerialEnabled
  // start serial port and digital Outputs
  Serial.begin(115200);
  Serial.println("ESP8266 MQTT WLAN weather station");
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
  #endif
 
  // Print the IP address
  #ifdef SerialEnabled
  Serial.print("Device IP Address: ");
  Serial.println(WiFi.localIP());
  #endif

  // Setup MQTT
  mqttClt.setServer(mqtt_server, 1883);
  mqttClt.setCallback(MqttCallback);
  #ifdef USELED
  // Setup finished, WLAN connected - blink once
  ToggleLed(LED,100,2);
  #endif
}


// ===============================================================================================
/*
 * Main function.
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
      ProgramResponding = true;
      continue;
    }
    else {
      break;
    }
  }
  #ifdef SerialEnabled
  Serial.print("Temp=");
  Serial.println(Temp); 
  Serial.print("RH=");
  Serial.println(RH); 
  #endif

  #ifdef USELED
  // Sensor data gathered - blink twice
  ToggleLed(LED,100,4);
  #endif

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

  if (!mqttClt.connected())
  {
    if (!ConnectToBroker())
    {
      // Failed to connect to broker
      #ifdef DEEPSLEEP
      ESP.deepSleep(DeepSleepTime * 60000000);
      #endif
      #ifdef SerialEnabled
      Serial.println("3 connection attempts to broker failed, sleeping 10 seconds..");
      #endif
      delay(10000);
      return;
    }
  }

  #ifdef USELED
  // Connected to broker - blink three times
  ToggleLed(LED,100,6);
  #endif

  // Subscribe to Interval Topic
  mqttClt.subscribe(Interval_topic);
  
  #ifdef SerialEnabled
  Serial.print("TempC = ");
  Serial.println(Temp);
  Serial.print("RH = ");
  Serial.println(RH);
  Serial.print("Vbat = ");
  Serial.println(vdd);
  Serial.println("Sending data to MQTT broker..");
  #endif

  // sleep a little for ESP background tasks
  delay(500);

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
  mqttClt.disconnect();
  #ifdef SerialEnabled
  Serial.println("Done, going to sleep.");
  #endif
  
  #ifdef USELED
  // Messages published - long blink
  ToggleLed(LED,300,2);
  #endif
  
  // initiate deep sleep
  #ifdef DEEPSLEEP
  ESP.deepSleep(DeepSleepTime);
  #endif
  #ifdef SerialEnabled
  Serial.println("DeepSleep disabled, sleeping 20 seconds..");
  #endif
  delay(20000);
}
