#ifndef _ULTRASONIC_SENSOR_H__
#define _ULTRASONIC_SENSOR_H__

#include "esp_log.h"
#include "esp_timer.h"
#include <driver/gpio.h>

#define ESP_ERR_ULTRASONIC_SENSOR_PING          0x200   
#define ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT  0x201
#define ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT  0x202

// Max distance that sensor can read [cm]
#define MAX_DISTANCE 400




// 1 trigger 2 echo pins
typedef struct 
{
    gpio_num_t trigger;
    gpio_num_t echo_left, echo_right;
} ultrasonic_sensor_t;


void ultrasonic_sensor_init(const ultrasonic_sensor_t* sensor);


//esp_err_t ultrasonic_sensor_measure_cm(const ultrasonic_sensor_t* sensor, uint32_t* distance);

esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t* sensor, uint32_t* distance_left, uint32_t* distance_right);

#endif /* _ULTRASONIC_SENSOR_H__ */