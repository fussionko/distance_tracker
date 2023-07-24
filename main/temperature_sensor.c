#include "temperature_sensor.h"

esp_err_t init_temperature_sensor(gpio_num_t pin)
{
    ESP_RETURN_ON_ERROR(gpio_reset_pin(pin), TAG, "");
    ESP_RETURN_ON_ERROR(gpio_set_direction(pin, GPIO_MODE_INPUT), TAG, "");


    ESP_RETURN_ON_ERROR(gpio_pulldown_dis(pin), TAG, "");
    ESP_RETURN_ON_ERROR(gpio_pullup_dis(pin), TAG, "");
}