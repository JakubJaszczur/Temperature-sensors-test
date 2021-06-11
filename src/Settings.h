// Misc
#define DEVICE_NAME         "Temperature"
#define UPDATE_INTERVAL     5

// MQTT
#define MQTT_MAX_PACKET_SIZE 256
#define LORA_TOPIC          "home/lora"
#define HC12_TOPIC          "home/433"
#define LORA_SEND_TOPIC     "gateway/send/lora"
#define HC12_SEND_TOPIC     "gateway/send/433"
#define SERVICE_TOPIC       "gateway/service"
#define DEBUG_TOPIC         "gateway/debug"

// Sensors
#define ALTITUDE        515.0 //define altitude of location
#define BME280_ADDR     0x76  //BME280 I2C Address
#define MCP9808_ADDR    0x18  //MCP9808 I2C Address
#define SHT31_ADDR      0x44


