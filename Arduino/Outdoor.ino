// Include the libraries we need
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

extern "C" {
  uint16 readvdd33(void);
}

/*
 * Variables and configuration
 */

// Enable ESP deepSleep in normal Operation
#define DEEPSLEEP

// Configure Serial Port for Debugging
//#define SerialEnabled

// Data wire is plugged into port GPIO4 on the ESP8266
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN 4

// I2C Pin definitions
#define I2C_SDA 2
#define I2C_SCL 14

// BMP180 barometric pressure sensor
Adafruit_BMP085 bmp;

// Status LED on GPIO2 (LED inverted!)
//#define USELED //does not work on white ESP-boards - do not define this to disable LED signalling
#define LED 2
#define LEDON LOW
#define LEDOFF HIGH

// DeepSleep Time MicroSec (5 minutes)
#define DeepSleepTime 306000000

//WLAN Configuration
WiFiClient Outdoorpj;
const char* ssid = "";
const char* password = "";

// MQTT Stuff
// MQTT_KEEP_ALIVE has been increased to 1800sec in PubSubClient.h
PubSubClient mqttClt(Outdoorpj);
#define mqtt_server "192.168.152.7"
#define mqtt_Client "temp-out.mik"
//#define mqtt_user "mqtt"
//#define mqtt_password "mqttpass"
#define humidity_topic "HB7/Outdoor/RH"
#define temperature_topic "HB7/Outdoor/Temp"
#define airpress_topic "HB7/Outdoor/AirPress"
#define voltage_topic "HB7/Outdoor/Vbat"
#define status_topic "HB7/Outdoor/Status"
const char* OK_Status = "online";
const char* LWT_Status = "offline";
const int LWT_QoS = 0;
const int LWT_Retain = 1;

// Setup a DHT instance
DHT dht(DHTPIN, DHTTYPE);

// Temperature from DHT22
float Temp;

// Rel. Humidity from DHT22
float RH;

// Battery Voltage (milliVolt)
int vdd = 3123;

// Air Pressure (BMP180)
float QFE = 1050;


// Maximum connection attempts to broker before going to sleep
const int MaxConnAttempts = 2;


/*
 * Functions
 */

bool ConnectToBroker()
{
  bool RetVal = false;
  int ConnAttempt = 0;
  // Try to connect 3 times, then stop
  while (ConnAttempt <= MaxConnAttempts)
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    if (mqttClt.connect(mqtt_Client,status_topic,LWT_QoS,LWT_Retain,LWT_Status))
    {
      Serial.println("connected");
      RetVal = true;
      break;
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqttClt.state());
      Serial.println("Sleeping 5 seconds..");
      // Wait 5 seconds before retrying
      delay(5000);
      ConnAttempt++;
    }
  }
  return RetVal;
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
  digitalWrite(LED, LEDON);
  #endif

  // Read Battery voltage - only works while WiFi not connected (?)
  // should't be an issue as setup runs after every DeepSleep
  vdd = readvdd33();

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
  digitalWrite(LED, LEDOFF);
  #endif
}


/*
 * Main function.
 */
void loop(void)
{
  // Read Temperature and RH from DHT22
  // Try 3 times max. if "nan" is returned
  delay(500);
  for (int i=0; i <= 2; i++){
    // Read DHT22 Sensors
    RH = dht.readHumidity();
    Temp = dht.readTemperature();
    if (isnan(RH) || isnan(Temp)) {
      delay(2000);
      continue;
    }
    else {
      break;
    }
  }

  // Read barometric pressure from BMP180
  QFE = bmp.readPressure();
  
  #ifdef SerialEnabled
  Serial.print("Temp=");
  Serial.println(Temp); 
  Serial.print("RH=");
  Serial.println(RH); 
  Serial.print("AirPress=");
  Serial.println(QFE); 
  #endif

  if (!mqttClt.connected())
  {
    if (!ConnectToBroker())
    {
      // Failed to connect to broker
      #ifdef DEEPSLEEP
      ESP.deepSleep(DeepSleepTime);
      #else
      #ifdef SerialEnabled
      Serial.println("3 connection attempts to broker failed, sleeping 10 seconds..");
      #endif
      delay(10000);
      return;
      #endif      
    }
  }
  
  // Loop triggers ping/pong; not required
  //mqttClt.loop();

  #ifdef SerialEnabled
  Serial.print("TempC = ");
  Serial.println(Temp);
  Serial.print("RH = ");
  Serial.println(RH);
  Serial.print("AirPress = ");
  Serial.println(QFE);
  Serial.print("Vbat = ");
  Serial.println(vdd);
  Serial.println("Sending data to MQTT broker..");
  #endif
  // Pubish retained messages to broker
  mqttClt.publish(temperature_topic, String(Temp).c_str(), true);
  mqttClt.publish(airpress_topic, String(QFE).c_str(), true);
  mqttClt.publish(humidity_topic, String(RH).c_str(), true);
  mqttClt.publish(voltage_topic, String(vdd).c_str(), true);
  mqttClt.publish(status_topic, String(OK_Status).c_str(), true);
  // do NOT disconnect from broker, or LWT settings will not work
  //mqttClt.disconnect();
  #ifdef SerialEnabled
  Serial.println("Done, going to sleep.");
  #endif
  
  
  // initiate deep sleep
  #ifdef DEEPSLEEP
  ESP.deepSleep(DeepSleepTime);
  #else
  #ifdef SerialEnabled
  Serial.println("DeepSleep disabled, sleeping 20 seconds..");
  #endif
  delay(20000);
  #endif
}
