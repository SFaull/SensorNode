// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3
// see https://community.openenergymonitor.org/t/calibration-and-parameters-in-emonlib/6855
// also https://learn.openenergymonitor.org/electricity-monitoring/ctac/ct-and-ac-power-adaptor-installation-and-calibration-theory?redirected=true

#include "Config.h" // when compiling, make sure the correct config is uncluded (there is one for each device)
#include "CredentialsConfig.h"  // Ensure this file exists, it should be generated from CredentialsConig-template.h
#include "version.h"

#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <WifiUDP.h>
#include <mDNSResolver.h>
#include <ArduinoJson.h>
#include <String.h>
#include <stdio.h>
#include <string.h>

#ifdef WS2812_LED
  #include "FastLED.h"
  #include "ledcontroller.h"
#endif

#ifdef SENSOR_SCT_013_000
  #include "EmonLib.h"
#endif

#ifdef AD7680
  #include <SPI.h>
#endif

#ifdef SENSOR_BME280
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
#endif

#ifdef DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif

using namespace mDNSResolver;

// General variable declarations
WiFiClient espClient;
WiFiUDP udp;
Resolver resolver(udp);
PubSubClient client(espClient);
unsigned long publishTimer = 0;
bool initialWifiConfig = false;
uint8_t msg_counter = 12;

// Module specific variable declarations
#ifdef WS2812_LED
  CRGB leds[WS2812_NUM_LEDS];
  LEDController ledController(leds);
#endif

#ifdef SENSOR_SCT_013_000
  EnergyMonitor emon1;
  double power = 0;
  double current = 0;
  double offset = 0;
#endif

#ifdef SENSOR_BME280
  Adafruit_BME280 bme;
  double temperature = 0;
  double humidity = 0;
  double altitude = 0;
  double pressure = 0;
#endif

#ifdef DS18B20
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
  DeviceAddress tempDeviceAddress;
  float ds18b20Temp = 0.0;
#endif

// get an ip from a hostname on the connected network
IPAddress findMDNS(String mDnsHost) { 
  // the input mDnsHost is e.g. "mqtt-broker" from "mqtt-broker.local"
  Serial.println("Finding the mDNS details...");
  // Need to make sure that we're connected to the wifi first
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }

  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Resolving ");
  Serial.println(MQTT_SERVER);

  resolver.setLocalIP(WiFi.localIP());
  
  IPAddress ip = resolver.search(MQTT_SERVER);
  if(ip != INADDR_NONE) {
    Serial.print("Resolved: ");
    Serial.println(ip);
  } else {
    Serial.println("Not resolved");
    Serial.println("Assuming IP has been provided, rather than hostname");
    bool success = ip.fromString(MQTT_SERVER);
    if(!success)
      Serial.println("Failed to determine MQTT host");

  }
  
  return ip;
}


void initOTA(void)
{
  ArduinoOTA.setHostname(HOSTNAME);
  
  ArduinoOTA.onStart([]() {
#ifdef WS2812_LED
    ledController.setColour(WS2812_BRIGHTNESS,0,0);
#endif
    Serial.println("OTA Update Started");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Complete");
#ifdef WS2812_LED
    ledController.setColour(0,0,0);
#endif
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

/* pass this function a pointer to an unsigned long to store the start time for the timer */
void setTimer(unsigned long *startTime)
{
  *startTime = millis();  // store the current time
}

/* call this function and pass it the variable which stores the timer start time and the desired expiry time
   returns true fi timer has expired */
bool timerExpired(unsigned long startTime, unsigned long expiryTime)
{
  if ( (millis() - startTime) >= expiryTime )
    return true;
  return false;
}

// rounds a number to x decimal places
// example: round(3.14159, 2) -> 3.14
double roundMe(double value, unsigned int dp) {
   return (int)(value * (10*dp) + 0.5) / (10.0 * (float)dp) ;
}

void sendMqttDiscovery(String sensorName, String sensorFriendlyName, String sensorClass, String sensorUnit) {
  // This is the discovery topic for this specific sensor
  String discoveryTopic = String(MQTT_DISCOVERY) + "/" + sensorName + "/config";

  StaticJsonDocument<2048> doc; // create a JSON document
  char buffer[512];

  doc["name"] = sensorFriendlyName;
  doc["unique_id"] = String(HOSTNAME) + "_" + sensorName;
  doc["state_topic"] = MQTT_DATA;
  doc["state_class"] = "measurement";
  doc["unit_of_measurement"] = sensorUnit;
  doc["device_class"] = sensorClass;
  doc["force_update"] = true;
  doc["value_template"] = "{{ value_json." + sensorName + " }}";
  doc["device"]["identifiers"] = HOSTNAME; 
  doc["device"]["name"] = String(DEVICE_MODEL) + "_" + String(HOSTNAME); 
  doc["device"]["manufacturer"] = "SamFaull";
  doc["device"]["sw_version"] = VERSION_STRING; 
  doc["device"]["model"] = DEVICE_MODEL; 

  size_t n = serializeJson(doc, buffer);

  bool result = client.publish(discoveryTopic.c_str(), buffer, n);

  // debug
  //Serial.print("Bytes: ");
  //Serial.println(n);
  Serial.print("Topic: ");
  Serial.println(discoveryTopic.c_str());
  Serial.print("JSON: ");
  serializeJsonPretty(doc, Serial);
  Serial.println();
  Serial.println(result ? "Send: Success" : "Send: Failed");
}

void sendMqttDiscoveryAllSensor()
{
#ifdef SENSOR_SCT_013_000
  sendMqttDiscovery(JSON_PROP_CURRENT, "Current", "current", "A");
  sendMqttDiscovery(JSON_PROP_POWER, "Power", "power", "W");
#endif

#ifdef SENSOR_BME280
  sendMqttDiscovery(JSON_PROP_BME_TEMP, "Temperature", "temperature", "°C");
  sendMqttDiscovery(JSON_PROP_BME_HUM, "Humidity", "humidity", "%");
  sendMqttDiscovery(JSON_PROP_BME_ALT, "Altitude", "distance", "m");
  sendMqttDiscovery(JSON_PROP_BME_PRES, "Atmospheric Pressure", "atmospheric_pressure", "mBar");
#endif

#ifdef DS18B20
  sendMqttDiscovery(JSON_PROP_DALLAS_TEMP, "Probe Temperature", "temperature", "°C");
#endif

  sendMqttDiscovery(JSON_PROP_WIFI_RSSI, "WiFi RSSI", "signal_strength", "dBm");
}

void publishReadings(void)
{
  StaticJsonDocument<1024> doc; // create a JSON document
  char buffer[512]; // create a character buffer for the JSON serialised stream

  // copy the temparure readings into the JSON object as strings
#ifdef SENSOR_SCT_013_000
  doc[JSON_PROP_CURRENT] = roundMe(current, 4);
  doc[JSON_PROP_POWER] = roundMe(power, 4);
#endif

#ifdef SENSOR_BME280
  doc[JSON_PROP_BME_TEMP] = roundMe(temperature, 2);
  doc[JSON_PROP_BME_HUM] = roundMe(humidity, 2);
  doc[JSON_PROP_BME_ALT] = roundMe(altitude, 2);
  doc[JSON_PROP_BME_PRES] = roundMe(pressure, 2);
#endif

#ifdef DS18B20
  doc[JSON_PROP_DALLAS_TEMP] = roundMe(ds18b20Temp, 2);
#endif

  doc[JSON_PROP_WIFI_RSSI] = WiFi.RSSI();

  // now publish
  size_t n = serializeJson(doc, buffer);  // serialise the JSON doc
  client.publish(MQTT_DATA, buffer, n);  // pulish the stream

  Serial.print("Topic: ");
  Serial.println(MQTT_DATA);
  Serial.print("JSON: ");
  serializeJsonPretty(doc, Serial);
  Serial.println();
}

void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  /* --------------- Print incoming message to serial ------------------ */
  char input[length + 1];
  for (int i = 0; i < length; i++)
    input[i] = (char)payload[i];  // store payload as char array
  input[length] = '\0'; // dont forget to add a termination character

  Serial.print("MQTT message received: ");
  Serial.println(input);

  if (strcmp(topic, MQTT_CMD)==0)
  {    
    if(strcmp(input,"*IDN?")==0)
    {
      //TODO
      //client.publish(MQTTtopic_comms, "Shed -> Current Monitor");
    }
    if(strcmp(input,"*RST")==0)
    {
      // give info
      client.disconnect();
      WiFiManager wifiManager;
      wifiManager.resetSettings();
      wifiManager.autoConnect(DEVICE_NAME);
      //ESP.restart();
    }
  }  
}


#ifdef AD7680
/*  Data Format:
 *  [0][0][0][0][B15][B14][B13][B12][B11][B10][B9][B8][B7][B6][B5][B4][B3][B2][B1][B0][0][0][0][0] 
 *  |________|   |_________________________________________________________________|  |________|
 * 4 LEADING 0S                        16-bit SAMPLE DATA                             4 TRAILING 0S
 */
unsigned int AD7680_read()
{
  uint16_t data;
  unsigned int reading;
  
  // enable chip
  digitalWrite(AD7680_CS,LOW);
  
  // get the first byte, remove the leading zeros and shift.
  data = SPI.transfer(255);   // B2
  reading = (data &= 0x0F)  << 12;

  // get the second byte and shift
  data = SPI.transfer(255);   // B1
  reading |= (data << 4);

  // get the third byte, remove the trailing zeros and shift
  data = SPI.transfer(255);   // B0
  reading |= (data &= 0xF0) >> 4;

  // disable chip
  digitalWrite(AD7680_CS,HIGH);

  
  return reading;
}

/* returns the average of 4 sequential samples */
unsigned int AD7680_read_avg()
{
  unsigned long sum = 0;
  
  for (int i = 0; i < 4; i++)
    sum += AD7680_read();
  
  return (unsigned int)(sum >> 2);
}
#endif

void initSensors(void)
{

#ifdef SENSOR_SCT_013_000
  emon1.current(SCT_013_000_PIN, SCT_013_000_CAL_FACTOR);             // Current: input pin, calibration (the mains current that gives you 1 V at the ADC input.)  
#endif

#ifdef AD7680
  // setup pin modes
  pinMode(AD7680_CS,OUTPUT);
  pinMode(AD7680_MISO,INPUT);
  pinMode(AD7680_SCLK,OUTPUT);

  // disable device at startup
  digitalWrite(AD7680_CS,HIGH);
  digitalWrite(AD7680_SCLK,HIGH);  

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2); //divide the clock by 2 (40MHz?)

  emon1.setReadCallback(AD7680_read_avg);
  emon1.setAdcResolution(16);    
#endif


#ifdef SENSOR_SCT_013_000
  // throwaway the first few samples
  for(int i = 0; i < 10; i++)
    emon1.calcIrms(1000);
#endif 

#ifdef SENSOR_BME280
  bool status = bme.begin(BME280_I2C_ADDRESS);  
  if (!status) {
    ledController.setColour(WS2812_BRIGHTNESS,0,0);
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
#endif

#ifdef DS18B20
  sensors.begin();
  int deviceCount = sensors.getDeviceCount();
  Serial.print("Found: ");
  Serial.print(deviceCount);
  Serial.println(" DS18B20 sensors");
  if(deviceCount < 1)
    while(1);
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, 12);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
#endif
}

void readSensors(void)
{
#ifdef SENSOR_SCT_013_000
    current = emon1.calcIrms(SCT_013_000_SAMPLES);  // Calculate Irms only (arg is number of samples)
    current -= SCT_013_000_ZERO_OFFSET;
    if (current<0)    current = 0;
    power = current * SCT_013_000_VOLTAGE;         // Apparent power
    if (power<0)    power = 0;
#endif

#ifdef SENSOR_BME280
    do
    {
      temperature = bme.readTemperature();
    } while (temperature < -40 || temperature > 85);  // this is known to return a huge value occasionally, lets keep querying until we get an in-range temperature
    
    humidity = bme.readHumidity();
    pressure = bme.readPressure();
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);  // approximate
#endif

#ifdef DS18B20
  unsigned int retries = 10;
  bool sensorsReady = false;

  // a little blocking loop to wait for conversion to be complete
  do
  {
    sensorsReady = sensors.isConversionComplete();
    retries--;
    if(!sensorsReady)
    {
      delay(1);
      Serial.println("Waiting for conversion to complete");
    }
  } while (!sensorsReady && retries > 0);
  
  ds18b20Temp = sensors.getTempCByIndex(0);
  sensors.requestTemperatures();
#endif
}

void wifiProcess(void)
{  
  // try connecting to the last access point
  if(WiFi.status() != WL_CONNECTED)
    WiFi.mode(WIFI_STA);

  int seconds = 0;
  // dont give up unless no connection for 60 seconds.
  while(WiFi.status() != WL_CONNECTED)
  {
    seconds++;
    delay(1000);

    // its been 1 minute, probably not gonna reconnect, lets reset.
    if(seconds > 60)
    {
        ESP.reset(); // This is a bit crude. For some unknown reason webserver can only be started once per boot up 
        // so resetting the device allows to go back into config mode again when it reboots.
        delay(5000);
    }
  }

  // check mqtt connection (TODO should have some timout on this.)
  if (!client.connected())
  {
      Serial.print("Attempting MQTT connection... ");
      // Attempt to connect
      if (client.connect(DEVICE_NAME, MQTT_USERNAME, MQTT_PASSWORD))
      {
        Serial.println("Connected");
        client.subscribe(MQTT_CMD);
        client.setBufferSize(512);

        StaticJsonDocument<1024> doc; // create a JSON document
        doc["name"] = HOSTNAME;
        doc["description"] = DEVICE_DESCRIPTION;
        doc["version"] = VERSION_STRING;
        doc["core"] = "ESP8266";
        doc["ssid"] = String(WiFi.SSID().c_str());
        doc["ip"] = WiFi.localIP().toString();
        
        char buffer[512]; // create a character buffer for the JSON serialised stream
        size_t n = serializeJson(doc, buffer);  // serialise the JSON doc
        client.publish(MQTT_METADATA, buffer, n);  // publish the stream
        Serial.print("Publishing to ");
        Serial.println(MQTT_METADATA);
        serializeJsonPretty(doc, Serial);
        Serial.println();
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.print(" try again in 5 seconds");
#ifdef WS2812_LED
        ledController.setColour(WS2812_BRIGHTNESS,0,0);
#endif
        // Wait 5 seconds before retrying and flash LED red
        delay(3000);
#ifdef WS2812_LED
        ledController.setColour(0,0,0);
#endif
        delay(2000);
      }
  }

  client.loop();

  ArduinoOTA.handle();
}

void printReadings()
{
  #ifdef SENSOR_SCT_013_000
  Serial.print("Current: ");
  Serial.println(current);
  Serial.print("Power: ");
  Serial.println(power);
#endif

#ifdef SENSOR_BME280
  Serial.print("Temperatute: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("Altitude: ");
  Serial.println(altitude);
#endif

#ifdef DS18B20
  Serial.print("Temperatute: ");
  Serial.println(ds18b20Temp);
#endif

  Serial.println();

}


void initWifi(void)
{
  WiFi.hostname(HOSTNAME);
  WiFi.mode(WIFI_STA); // Force to station mode because if device was switched off while in access point mode it will start up next time
  WiFiManager wifiManager;
  wifiManager.setTimeout(WIFI_TIMEOUT);
  bool connectionSuccess = wifiManager.autoConnect(HOSTNAME);  
  if (!connectionSuccess) 
  {
    Serial.println("Connection failed... restarting");
    ESP.reset(); // This is a bit crude. For some unknown reason webserver can only be started once per boot up 
    // so resetting the device allows to go back into config mode again when it reboots.
    delay(5000);
  } 
}


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.print("SensorNode ");
  Serial.println(VERSION_STRING);

#ifdef WS2812_LED
  FastLED.addLeds<NEOPIXEL, WS2812_DATA_PIN>(leds, WS2812_NUM_LEDS);
  ledController.setColour(0,0,0);
#endif

  initOTA();
  initSensors();
  initWifi();

  /* init MQTT */
  IPAddress ip = findMDNS(MQTT_SERVER);
  client.setServer(ip, MQTT_PORT);
  client.setCallback(callback);

  readSensors();
  setTimer(&publishTimer);

#ifdef WS2812_LED
  ledController.setColourTarget(0,0,0);
#endif
}

void loop()
{  
  // keep wireless comms alive
  wifiProcess();
  
  // update the LEDs 
#ifdef WS2812_LED
  ledController.run();
#endif

  // if timer has expired, take some readings and publish
  if (timerExpired(publishTimer, MQTT_PUBLISH_INTERVAL))  // get the time every 5 seconds
  {
    setTimer(&publishTimer);  // reset timer
    readSensors();
    publishReadings();
#ifdef WS2812_LED
    ledController.pulse(0,WS2812_BRIGHTNESS,0);
#endif

    // discovery message (less frequent)
    msg_counter++;
    if(msg_counter >= 12)
    {
      sendMqttDiscoveryAllSensor();
      msg_counter = 0;
    }
  }
  
}
