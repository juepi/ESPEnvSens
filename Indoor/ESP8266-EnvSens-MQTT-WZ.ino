// Include the libraries we need
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

extern "C" {
  uint16 readvdd33(void);
  #include "user_interface.h"
}

/*
 * Variables and configuration
 */

// Enable (define) ESP deepSleep in normal Operation
#define DEEPSLEEP

// Enable (define) Serial Port for Debugging
//#define SerialEnabled

// Data wire is plugged into port GPIO4 on the ESP8266
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN 4

// Status LED on GPIO2 (LED inverted!)
#define USELED //does not work on white ESP-boards - do not define this to disable LED signalling
#define LED 2
#define LEDON LOW
#define LEDOFF HIGH

// DeepSleep Time MicroSec (5 minutes)
#define DeepSleepTime 306000000


// OS Timer for Software Watchdog
os_timer_t WDTimer;
bool ProgramResponding = true;
// WDT will trigger every 5 seconds
#define WDTIMEOUT 5000


//WLAN Configuration
WiFiClient IndoorWZ;
const char* ssid = "";
const char* password = "";

// MQTT Stuff
// MQTT_KEEP_ALIVE has been increased to 1800sec in PubSubClient.h !!!
PubSubClient mqttClt(IndoorWZ);
#define mqtt_server "192.168.152.7"
#define mqtt_Client "temp-wz.mik"
#define humidity_topic "HB7/Indoor/WZ/RH"
#define temperature_topic "HB7/Indoor/WZ/Temp"
#define voltage_topic "HB7/Indoor/WZ/Vbat"
#define status_topic "HB7/Indoor/WZ/Status"
const char* OK_Status = "online";
const char* LWT_Status = "offline";
const int LWT_QoS = 0;
const int LWT_Retain = 1;

// Setup a DHT instance
DHT dht(DHTPIN, DHTTYPE);

// Room Temperature from DHT22
float Temp;

// Rel. Humidity from DHT22
float RH;

// Battery Voltage (milliVolt)
int vdd = 3123;


// Maximum connection attempts to broker before going to sleep
const int MaxConnAttempts = 3;


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
    if (mqttClt.connect(mqtt_Client,status_topic,LWT_QoS,LWT_Retain,LWT_Status))
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
    ProgramResponding = false;
    return;
  }
  // ProgramResponding already false - program dead, go to DeepSleep
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
  ESP.deepSleep(DeepSleepTime);
  #endif
}



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

  #ifdef USELED
  // Setup finished, WLAN connected - blink once
  ToggleLed(LED,100,2);
  #endif
}


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
      delay(2500);
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

  if (!mqttClt.connected())
  {
    if (!ConnectToBroker())
    {
      // Failed to connect to broker
      #ifdef DEEPSLEEP
      ESP.deepSleep(DeepSleepTime);
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
  // do NOT disconnect from broker, or LWT settings will not work
  //mqttClt.disconnect();
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
