#ifndef _TEMPERATURE_SENSOR_H__
#define _TEMPERATURE_SENSOR_H__

// Based on tuturial: https://esp32tutorials.com/dht22-esp32-esp-idf/

#include "driver/gpio.h"

#ifndef GPIO_OUTPUT_HIGH
#define GPIO_OUTPUT_HIGH 1
#endif // GPIO_OUTPUT_HIGH

#ifndef GPIO_OUTPUT_LOW
#define GPIO_OUTPUT_LOW 0
#endif // GPIO_OUTPUT_LOW


// DHT22 error codes
typedef enum dht22_error { DHT22_OK = 0, DHT22_CHECKSUM_ERROR = 1, DHT22_TIMEOUT_ERROR = 2 } dht22_error;

esp_err_t init_dht22(gpio_num_t pin);

dht22_error read_dht22();

float get_humidity();
float get_temperature();


#endif // _TEMPERATURE_SENSOR__