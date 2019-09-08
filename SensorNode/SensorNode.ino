// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3
// see https://community.openenergymonitor.org/t/calibration-and-parameters-in-emonlib/6855
// also https://learn.openenergymonitor.org/electricity-monitoring/ctac/ct-and-ac-power-adaptor-installation-and-calibration-theory?redirected=true

#include "Config.h"
#include "CredentialsConfig.h"
#include "ledcontroller.h"
#include "version.h"

#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <WifiUDP.h>
#include <String.h>
#include <stdio.h>
#include <string.h>
#include "FastLED.h"

#ifdef SENSOR_SCT_013_000
  #include "EmonLib.h"
#endif

#ifdef SENSOR_BME280
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
#endif




// General variable declarations
WiFiClient espClient;
PubSubClient client(espClient);
CRGB leds[WS2812_NUM_LEDS];
LEDController ledController(leds);
unsigned long publishTimer = 0;
bool initialWifiConfig = false;


// Module specific variable declarations
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



void initOTA(void)
{
  ArduinoOTA.onStart([]() {
    ledController.setColour(WS2812_BRIGHTNESS,0,0);
    Serial.println("OTA Update Started");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Complete");
    ledController.setColour(0,0,0);
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

void publishReadings(void)
{
#ifdef SENSOR_SCT_013_000
  client.publish(MQTT_TOPIC_CURRENT, String(current).c_str());
  client.publish(MQTT_TOPIC_POWER, String(power).c_str());
#endif

#ifdef SENSOR_BME280
  client.publish(MQTT_TOPIC_TEMPERATURE, String(temperature).c_str());
  client.publish(MQTT_TOPIC_HUMIDITY, String(humidity).c_str());
  client.publish(MQTT_TOPIC_PRESSURE, String(pressure).c_str());
  client.publish(MQTT_TOPIC_ALTITUDE, String(altitude).c_str());
#endif
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

  if (strcmp(topic, MQTT_COMMS)==0)
  {    
    if(strcmp(input,"*IDN?")==0)
    {
      //TODO
      //client.publish(MQTTtopic_comms, "Shed -> Current Monitor");
    }
    if(strcmp(input,"*RST")==0)
    {
      // give info
      ledController.setColour(WS2812_BRIGHTNESS,0,0);
      client.disconnect();
      WiFiManager wifiManager;
      wifiManager.resetSettings();
      wifiManager.autoConnect(DEVICE_NAME);
      //ESP.restart();
    }
  }  
}

void readSensors(void)
{
#ifdef SENSOR_SCT_013_000
    current = emon1.calcIrms(SCT_013_000_SAMPLES);  // Calculate Irms only (arg is number of samples)
    power = current*SCT_013_000_VOLTAGE;         // Apparent power
    if (power<0)    power = 0;
#endif

#ifdef SENSOR_BME280
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure();
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);  // approximate
#endif
}

void initSensors(void)
{
#ifdef SENSOR_SCT_013_000
  emon1.current(SCT_013_000_PIN, SCT_013_000_CAL_FACTOR);             // Current: input pin, calibration (the mains current that gives you 1 V at the ADC input.)
#endif

#ifdef SENSOR_BME280
  bool status = bme.begin(BME280_I2C_ADDRESS);  
  if (!status) {
    ledController.setColour(WS2812_BRIGHTNESS,0,0);
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
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
    // Loop until we're reconnected
    while (!client.connected())
    {
      Serial.print("Attempting MQTT connection... ");
      // Attempt to connect
      if (client.connect(DEVICE_NAME, MQTT_USERNAME, MQTT_PASSWORD))
      {
        Serial.println("Connected");
        client.publish(MQTT_ROOM, "Connected");
        client.subscribe(MQTT_COMMS);
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.print(" try again in 5 seconds");
        ledController.setColour(WS2812_BRIGHTNESS,0,0);
        // Wait 5 seconds before retrying and flash LED red
        delay(3000);
        ledController.setColour(0,0,0);
        delay(2000);
      }
    }
  }

  client.loop();

  ArduinoOTA.handle();
}


void initWifi(void)
{
  WiFi.mode(WIFI_STA); // Force to station mode because if device was switched off while in access point mode it will start up next time
  WiFiManager wifiManager;
  wifiManager.setTimeout(WIFI_TIMEOUT);
  bool connectionSuccess = wifiManager.autoConnect(DEVICE_NAME);  
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
  
  FastLED.addLeds<NEOPIXEL, WS2812_DATA_PIN>(leds, WS2812_NUM_LEDS);
  ledController.setColour(0,0,0);

  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  initOTA();
  initSensors();
  initWifi();
  
  setTimer(&publishTimer);
  ledController.setColourTarget(0,0,0);
}

void loop()
{
  // keep wireless comms alive
  wifiProcess();
  
  // update the LEDs 
  ledController.run();
  
  // if timer has expired, take some readings and publish
  if (timerExpired(publishTimer, MQTT_PUBLISH_INTERVAL))  // get the time every 5 seconds
  {
    setTimer(&publishTimer);  // reset timer
    readSensors();
    publishReadings();
    ledController.pulse(0,WS2812_BRIGHTNESS,0);
  }
}
