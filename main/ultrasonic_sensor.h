#ifndef _ULTRASONIC_SENSOR_H__
#define _ULTRASONIC_SENSOR_H__

#define ESP_ERR_ULTRASONIC_SENSOR_PING          0x200   
#define ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT  0x201
#define ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT  0x202

#include <driver/gpio.h>

typedef struct 
{
    gpio_num_t trigger;
    gpio_num_t echo;
} ultrasonic_sensor_t;


void ultrasonic_sensor_init(const ultrasonic_sensor_t* sensor);


esp_err_t ultrasonic_sensor_measure_cm(const ultrasonic_sensor_t* sensor, uint32_t max_distance, uint32_t* distance);

#endif /* _ULTRASONIC_SENSOR_H__ */