#include "ultrasonic_sensor.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/time.h>

#define TRIGGER_LOW_DELAY 4
#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT 6000

// Calc: 331.5+0.61*temperature[m/sec]
// TODO change with temp sensor -> humidity
#define ROUNDTRIP 58 // 20deg



static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


#define RETURN_CRITICAL(MUX, RES) do { portEXIT_CRITICAL(&MUX); return RES; } while (0)
static inline uint32_t get_time_us()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_usec;
}


void ultrasonic_sensor_init(const ultrasonic_sensor_t* sensor)
{
    gpio_reset_pin(sensor->trigger);
    gpio_reset_pin(sensor->echo);
    gpio_set_direction(sensor->trigger, GPIO_MODE_OUTPUT);
    gpio_set_direction(sensor->echo, GPIO_MODE_INPUT);

    gpio_set_level(sensor->trigger, 0);
}

esp_err_t ultrasonic_sensor_measure_cm(const ultrasonic_sensor_t* sensor, uint32_t max_distance, uint32_t* distance)
{
    if (distance == NULL)
        return ESP_ERR_INVALID_ARG;

    portENTER_CRITICAL(&mux);

    // Sets of ultrasonic sensor 
    gpio_set_level(sensor->trigger, 0);
    esp_rom_delay_us(TRIGGER_LOW_DELAY);
    gpio_set_level(sensor->trigger, 1);
    esp_rom_delay_us(TRIGGER_HIGH_DELAY);
    gpio_set_level(sensor->trigger, 0);

    // If previous ping hasn't ended
    if (gpio_get_level(sensor->echo))
        RETURN_CRITICAL(mux, ESP_ERR_ULTRASONIC_SENSOR_PING);
}