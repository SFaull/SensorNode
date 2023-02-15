// General
#define DEVICE_NAME             "IOT_TEST"
#define HOSTNAME                DEVICE_NAME
#define DEVICE_DESCRIPTION      "IOT Test Sensor (development)"
#define SERIAL_BAUD_RATE        115200
#define WIFI_TIMEOUT            300           // device will reset if no connection via portal within this number of seconds

// MQTT
#define MQTT_PUBLISH_INTERVAL   5000 // 5 seconds
#define MQTT_ROOM               "room"
#define MQTT_DATA               MQTT_ROOM "/" HOSTNAME "/data"
#define MQTT_CMD                MQTT_ROOM "/" HOSTNAME "/cmd"
#define MQTT_METADATA           MQTT_ROOM "/" HOSTNAME "/meta"     
#define MQTT_DISCOVERY          "homeassistant/sensor/" HOSTNAME

// JSON

  #define JSON_PROP_CURRENT     "current"
  #define JSON_PROP_POWER       "power"
  #define JSON_PROP_BME_TEMP    "bme_temperature"
  #define JSON_PROP_BME_HUM     "bme_humidity"
  #define JSON_PROP_BME_ALT     "bme_altitude"
  #define JSON_PROP_BME_PRES    "bme_pressure"
  #define JSON_PROP_DALLAS_TEMP "dallas_temperature"
  #define JSON_PROP_WIFI_RSSI   "wifi_rssi"

// Hardware Options (uncomment all of the sensors that are connected)
#define WS2812_LED                      // Neopixel LED
#define SENSOR_SCT_013_000              // Power & Current sensor
#define AD7680
#define SENSOR_BME280                   // Relative Humidity & Temperature sensor
#define DS18B20                           // Dalas onewire temperature sensor


// LED
#ifdef WS2812_LED
  #define WS2812_DATA_PIN         2 // D4 on Wemos 
  #define WS2812_NUM_LEDS         1
  #define WS2812_BRIGHTNESS       255
#endif

// Current sensor options
#ifdef SENSOR_SCT_013_000  
  #define SCT_013_000_PIN         A0
  #define SCT_013_000_SAMPLES     5000  // note: when using AD7680, for each sample 4 are actually captured and averaged. Testing shows 10k samples can be retrieved in 120ms.
  #define SCT_013_000_VOLTAGE     230.0
  #define SCT_013_000_CAL_FACTOR  102 //  SHED: 20.40816  GARAGE:86.9565 (calculated) 102 actually works better
  #define SCT_013_000_ZERO_OFFSET 0
  #define emonTxV3                                // uncomment if ADC is 3V3 
#endif

// External ADC pin mapping
#ifdef AD7680
  #define AD7680_CS   15 // Pin D10 -> ADC Pin 8 (CS)
  #define AD7680_MISO 12 // Pin D12 -> ADC Pin 7 (SDATA = MISO)
  #define AD7680_SCLK 14 // Pin D13 -> ADC Pin 5 (SCLK)
#endif

// RHT options
#ifdef SENSOR_BME280
  #define BME280_I2C_ADDRESS      0x76
  #define SEALEVELPRESSURE_HPA    (1013.25)
#endif

// Dallas temperature probe options (only 1 sensor supported)
#ifdef DS18B20
  #define ONE_WIRE_BUS 2          // D4
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


   // burden resistor for second build is 68Rx3 -> 22.66666 (measured is 23R exactly) https://learn.openenergymonitor.org/electricity-monitoring/ct-sensors/interface-with-arduino
