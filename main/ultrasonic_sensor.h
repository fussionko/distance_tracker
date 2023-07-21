#ifndef _ULTRASONIC_SENSOR_H__
#define _ULTRASONIC_SENSOR_H__

#include "esp_log.h"
#include <stdio.h>
#include <driver/gpio.h>
#include "esp_timer.h"


#define ESP_ERR_ULTRASONIC_SENSOR_PING          0x200   
#define ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT  0x201
#define ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT  0x202

#define EVENT_ULTRASONIC_SENSOR_ECHO_START      1
#define EVENT_ULTRASONIC_SENSOR_ECHO_END        2
#define EVENT_ULTRASONIC_SENSOR_PING_TIMEOUT    3

#define SIDE_NONE   0
#define LEFT_SIDE   1
#define RIGHT_SIDE  2

// Max distance that sensor can read [cm]
#define MAX_DISTANCE 400

typedef struct 
{
    uint8_t event_code;
    uint8_t side;
    uint32_t time;
} event_t;

// typedef struct 
// {
//     uint32_t time_start, time_end;
//     esp_err_t error;
// } time_data_t;



typedef uint8_t side_t;
 


// 1 trigger 2 echo pins
typedef struct 
{
    gpio_num_t trigger;
    gpio_num_t echo_left, echo_right;
} ultrasonic_sensor_t;




// ISR
void interrupt_init(const ultrasonic_sensor_t* sensor);
esp_err_t interrupt_enable(const ultrasonic_sensor_t* sensor);
esp_err_t interrupt_disable(const ultrasonic_sensor_t* sensor);

// Timers
void create_timer(esp_timer_handle_t* timer_handler, esp_timer_cb_t callback, char* name, void* callback_arg);
//void delete_timer(esp_timer_handle_t* timer_handler);
void timer_callback(void* args);


void ultrasonic_sensor_init(const ultrasonic_sensor_t* sensor);

esp_err_t trigger(const ultrasonic_sensor_t* sensor);

//esp_err_t ultrasonic_sensor_measure_cm(const ultrasonic_sensor_t* sensor, uint32_t* distance);

esp_err_t measure(esp_timer_handle_t* ping_timer, uint32_t* distance_left, uint32_t* distance_right);

esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t* sensor, uint32_t* distance_left, uint32_t* distance_right);

#endif /* _ULTRASONIC_SENSOR_H__ */