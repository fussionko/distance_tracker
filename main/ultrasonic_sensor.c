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

typedef struct 
{
    esp_err_t error;
    ultrasonic_sensor_t* ultrasonic_sensor;
    uint32_t distance;
} measure_param;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define timeout_expired(start, len) ((uint32_t)get_time_us() - start) >= len
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
    gpio_reset_pin(sensor->echo_left);
    gpio_reset_pin(sensor->echo_right);
    gpio_set_direction(sensor->trigger, GPIO_MODE_OUTPUT);
    gpio_set_direction(sensor->echo_left, GPIO_MODE_INPUT);
    gpio_set_direction(sensor->echo_right, GPIO_MODE_INPUT);

    gpio_set_level(sensor->trigger, 0);
}

esp_err_t ultrasonic_measure_left_cm(const ultrasonic_sensor_t* sensor, uint32_t* distance_left)
{
    portENTER_CRITICAL(&mux);

    // Sets of ultrasonic sensor 
    gpio_set_level(sensor->trigger, 0);
    esp_rom_delay_us(TRIGGER_LOW_DELAY);
    gpio_set_level(sensor->trigger, 1);
    esp_rom_delay_us(TRIGGER_HIGH_DELAY);
    gpio_set_level(sensor->trigger, 0);

    // If previous ping hasn't ended
    if (gpio_get_level(sensor->echo_left))
        RETURN_CRITICAL(mux, ESP_ERR_ULTRASONIC_SENSOR_PING);

    	
    // Wait for echo reply from single reciever
    uint32_t start = get_time_us();
    while (!gpio_get_level(sensor->echo_left))
    {
        if (timeout_expired(start, PING_TIMEOUT))
            RETURN_CRITICAL(mux, ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT);
    }

    // Echo recieved continue
    uint32_t echo_start = get_time_us();
    uint32_t time = echo_start;
    uint32_t meas_timeout = echo_start + MAX_DISTANCE + ROUNDTRIP;
    while (gpio_get_level(sensor->echo_left))
    {
        time = get_time_us();
        if (timeout_expired(start, meas_timeout))
            RETURN_CRITICAL(mux, ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT);
    }

    portEXIT_CRITICAL(&mux);

    *distance_left = (time - echo_start) / ROUNDTRIP;

    return ESP_OK;
}

esp_err_t ultrasonic_measure_right_cm(const ultrasonic_sensor_t* sensor, uint32_t* distance_right)
{

}


esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t* sensor, uint32_t* distance_left, uint32_t* distance_right)
{
    if (distance_left == NULL || distance_left == distance_right || sensor == NULL)
        return ESP_ERR_INVALID_ARG;

    TaskHandle_t measure_left = NULL;
    TaskHandle_t measure_right = NULL;

    xTaskCreate(ultrasonic_measure_left_cm, "Measure left sensor", 4096, NULL, 10, &measure_left);
    xTaskCreatePinnedToCore(ultrasonic_measure_right_cm, "Measure lright sensor", 4096, NULL, 10, &measure_right, 1);
}