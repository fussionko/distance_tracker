#ifndef _TEMPERATURE_SENSOR_H__
#define _TEMPERATURE_SENSOR_H__

#include "driver/gpio.h"
#include "esp_check.h"

static const char* TAG = "DHT22"; 

esp_err_t init_temp_sensor(gpio_num_t pin);

#endif // _TEMPERATURE_SENSOR__