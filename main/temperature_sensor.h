#ifndef _TEMPERATURE_SENSOR_H__
#define _TEMPERATURE_SENSOR_H__

// Based on tuturial: https://esp32tutorials.com/dht22-esp32-esp-idf/

#include "driver/gpio.h"

// Define gpio high and low
#ifndef GPIO_OUTPUT_HIGH
#define GPIO_OUTPUT_HIGH 1
#endif // GPIO_OUTPUT_HIGH

#ifndef GPIO_OUTPUT_LOW
#define GPIO_OUTPUT_LOW 0
#endif // GPIO_OUTPUT_LOW


#define DHT22_MAX_TEMP_READ     80      // degrees
#define DHT22_MIN_TEMP_READ     -40     // degrees
#define DHT22_MAX_SAMPLING_RATE 2000    //us

// DHT22 error codes
typedef enum dht22_error { DHT22_OK = 0, DHT22_CHECKSUM_ERROR = 1, DHT22_TIMEOUT_ERROR = 2 } dht22_error;
//static const char* dht22_error_names[] = { "DHT22_OK", "DHT22_CHECKSUM_ERROR", "DHT22_TIMEOUT_ERROR" };

esp_err_t init_dht22(gpio_num_t pin);

dht22_error read_dht22();

float get_humidity();
float get_temperature();

char* error_to_name_dht22(dht22_error error);


#endif // _TEMPERATURE_SENSOR__