#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "Settings.h"
#include "WifiSettings.h"
#include "version.h"

#include "Adafruit_MCP9808.h"
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"

Adafruit_MCP9808 mcp = Adafruit_MCP9808();   // temp
Adafruit_BME280 bme;                         // temp, hum, press
Adafruit_SHT31 sht = Adafruit_SHT31();       // temp, hum
WiFiClient espClient;
PubSubClient mqtt(espClient);

String version = String(VERSION_SHORT);

// GLOBALS

unsigned long lastUpdateTime = 0;

// FUNCTIONS //

void ConnectToWifi()
{
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.hostname(DEVICE_NAME);

  while(WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
}

void HandleService(String serviceMessage)
{

}

void callback(char* topic, byte* payload, unsigned int length) 
{
  String data = "";

  Serial.print("Message arrived from MQTT [");
  Serial.print(topic);
  Serial.print("] ");

  for (uint i = 0; i < length; i++)
  {
    data += (char)payload[i];
  }
  
  Serial.println(data);


  if(String(topic) == HC12_SEND_TOPIC)
  {

  }
}

void ConnectToMQTT() 
{
  // Loop until we're reconnected
  while (!mqtt.connected()) 
  {
    mqtt.setServer(mqtt_server, 1883);
    mqtt.setCallback(callback);

    Serial.print("Attempting MQTT connection...");

    delay(1000);

    // Attempt to connect
    if(mqtt.connect(DEVICE_NAME, mqtt_user, mqtt_password)) 
    {
      Serial.println("Connected");

      mqtt.subscribe(SERVICE_TOPIC);
    } 
    else 
    {
      Serial.print("Failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" Try again in 1 second");
      // Wait 5 seconds before retrying
    }
  }
}

bool CheckMqttConnection()
{
  if(!mqtt.connected()) 
  {
    ConnectToMQTT();
  }

  return true;
}

void MqttSend(String data, bool hc12)
{ 
  CheckMqttConnection();

  char message[200];
  data.toCharArray(message, sizeof(message));

  mqtt.publish(LORA_TOPIC, message);

}

// MCP9808 functions

boolean InitialiseMPC9808(byte mode)
{
  if( mcp.begin(MCP9808_ADDR)) 
  {
    Serial.println("MCP9808 Error");
    return false;
  }
  else
  {
    Serial.println("MCP9808 Initialised");
   mcp.setResolution(mode); // sets the resolution mode of reading, the modes are defined in the table bellow:
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
    return true;
  }
}

float GetTemperatureMCP9808()
{
 mcp.wake();   // wake up, ready to read!
  float temperature = mcp.readTempC();
 mcp.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling

  return temperature;
}

String ComposeJSONmessage(int id, float temp, float hum, float press, float lux, float uv, int pm_25, int pm_10, float bat, float panel)
{
  String message;

  const size_t capacity = JSON_OBJECT_SIZE(10) + 90;
  DynamicJsonDocument doc(capacity);

    doc["id"] = id;

  serializeJson(doc,message);

  return message;
}

// BME280 functions

boolean InitialiseBME280()
{
  if (!bme.begin(BME280_ADDR, &Wire))
  //if (!bme.begin())  
  {
    Serial.println("BME280 Error");
    return false;
  }
  else
  {
    Serial.println("BME280 Initialised");
    return true;
  }
  
}

float GetTemperatureBME280()
{
  float temperature = bme.readTemperature();
  
  return temperature;
}

// Not used

/*float GetHumidityBME280()
{
  float humidity = bme.readHumidity();
  
  return humidity;
}

float GetPressureBME280()
{
  float pressure = bme.readPressure();
  pressure = bme.seaLevelForAltitude(ALTITUDE, pressure);
  pressure = pressure / 100.0F;
  
  return pressure;
}*/

// SHT31 functions

boolean InitialiseSHT31()
{
  if (!sht.begin(SHT31_ADDR))
  //if (!bme.begin())  
  {
    Serial.println("SHT31 Error");
    return false;
  }
  else
  {
    Serial.println("SHT31 Initialised");
    sht.heater(false);
    return true;
  }
}

float GetTemperatureSHT31()
{
  float temperature = sht.readTemperature();;
  
  return temperature;
}

// MAIN CODE //

void setup() 
{
  Serial.begin(115200);
  Wire.begin();

  ConnectToWifi();
  ConnectToMQTT();

  InitialiseMPC9808(3);
  InitialiseBME280();
  InitialiseSHT31();
}

void loop() 
{
  unsigned long actualTime = millis();

  float tempMPC9808 = 0;
  float tempBME280 = 0;
  float tempSHT31 = 0;

  if(actualTime - lastUpdateTime > UPDATE_INTERVAL * 1000)
  {
    tempMPC9808 = GetTemperatureMCP9808();
    tempBME280 = GetTemperatureBME280();
    tempSHT31 = GetTemperatureSHT31();

    Serial.println("MPC9808    BME280     SHT31");
    Serial.print(tempMPC9808);
    Serial.print("      ");
    Serial.print(tempBME280);
    Serial.print("      ");
    Serial.println(tempSHT31);

    lastUpdateTime = actualTime;
  }

  if(WiFi.status() != WL_CONNECTED)
  {
    ConnectToWifi();
  }

  mqtt.loop();
}