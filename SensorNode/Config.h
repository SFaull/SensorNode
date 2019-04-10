// General
#define DEVICE_NAME             "PowerMonitor"
#define SERIAL_BAUD_RATE        115200

// MQTT
#define MQTT_PUBLISH_INTERVAL   5000 // 5 seconds
#define MQTT_ROOM               "Shed"
#define MQTT_COMMS              MQTT_ROOM "/" DEVICE_NAME

// LED
#define WS2812_DATA_PIN         4 
#define WS2812_NUM_LEDS         1
#define WS2812_BRIGHTNESS       255

// Sensors
#define SENSOR_SCT_013_000              // Power & Current sensor
#define SENSOR_BME280                   // Relative Humidity & Temperature sensor

// Current sensor options
#ifdef SENSOR_SCT_013_000
  #define SCT_013_000_PIN         A0
  #define SCT_013_000_SAMPLES     1480
  #define SCT_013_000_VOLTAGE     230.0
  #define SCT_013_000_CAL_FACTOR  20.40816
  #define emonTxV3                                // uncomment if ADC is 3V3 
  #define MQTT_TOPIC_POWER        MQTT_ROOM "/Power"
  #define MQTT_TOPIC_CURRENT      MQTT_ROOM "/Current"
#endif

// RHT options
#ifdef SENSOR_BME280
  #define BME280_I2C_ADDRESS      0x76
  #define SEALEVELPRESSURE_HPA    (1013.25)
  #define MQTT_TOPIC_TEMPERATURE  MQTT_ROOM "/Temperature"
  #define MQTT_TOPIC_HUMIDITY     MQTT_ROOM "/Humidity"
  #define MQTT_TOPIC_PRESSURE     MQTT_ROOM "/Pressure"
  #define MQTT_TOPIC_ALTITUDE     MQTT_ROOM "/Altitude"
#endif




   /*
    * Notes on power sensor
   * counts = (input pin voltage ÷ 3.3) × 1024
   * input pin voltage = secondary current × burden resistance
   * secondary current = primary current ÷ transformer ratio
   * 
   * This currents sensor is 50mA at 100A
   * The burden resistor is 98ohms
   * 
   * Isupply = count × a constant
   * a constant = current constant × (3.3 ÷ 1024)
   * current constant = (100 ÷ 0.050) ÷ 98 = 20.40816
   */
