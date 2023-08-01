#ifndef _ULTRASONIC_SENSOR_H__
#define _ULTRASONIC_SENSOR_H__

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_timer.h"


#define ESP_ERR_ULTRASONIC_SENSOR_PING          0x200   
#define ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT  0x201
#define ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT  0x202
#define ESP_ERR_ULTRASONIC_SENSOR_ECHO_ERROR    0x203

// Max distance that sensor can read
#define MAX_DISTANCE 4  // [m]

// Event codes don't change echo start and end (gpio level)
typedef enum event_code_t { ECHO_END = 0, ECHO_START = 1, PING_TIMEOUT = 2, ECHO_TIMEOUT = 3} event_code_t;


typedef struct 
{
    event_code_t event_code;
    uint64_t time;
} event_t;

 
// 1 trigger 1 echo pin
typedef struct 
{
    gpio_num_t trigger;
    gpio_num_t echo;
} ultrasonic_sensor_t;


// Set sound speed
void set_sound_speed(float temperature, float humidity);

// Init sensor
void hc_sr04_init(const ultrasonic_sensor_t* sensor);

// Measure the distance from left and right sensor to target
esp_err_t measure(const ultrasonic_sensor_t* sensor, float* distance);

// Measure avg over time periode
esp_err_t measure_avg(const ultrasonic_sensor_t* sensor, float* distance, const uint64_t time_period_us, const uint32_t sampling_rate_per_second);


#endif /* _ULTRASONIC_SENSOR_H__ */