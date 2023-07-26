#ifndef _ULTRASONIC_SENSOR_H__
#define _ULTRASONIC_SENSOR_H__

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_timer.h"


#define ESP_ERR_ULTRASONIC_SENSOR_PING          0x200   
#define ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT  0x201
#define ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT  0x202


// Max distance that sensor can read [m]
#define MAX_DISTANCE 4

// Event codes don't change echo start and end (gpio level)
typedef enum event_code_t { ECHO_END = 0, ECHO_START = 1, PING_TIMEOUT = 2, ECHO_TIMEOUT = 3} event_code_t;


typedef struct 
{
    event_code_t event_code;
    uint8_t side;
    uint64_t time;
} event_t;

 
// 1 trigger 2 echo pins
typedef struct 
{
    gpio_num_t trigger;
    gpio_num_t echo_left, echo_right;
} ultrasonic_sensor_t;


// Set sound speed
void set_sound_speed(float temperature, float humidity);

// Init sensor
void ultrasonic_sensor_init(const ultrasonic_sensor_t* sensor);

// Measure the distance from left and right sensor to target
esp_err_t measure(const ultrasonic_sensor_t* sensor, float* distance_left, float* distance_right);

#endif /* _ULTRASONIC_SENSOR_H__ */