#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"

#include "esp_check.h"

#include "ultrasonic_sensor.h"
#include "temperature_sensor.h"

// Ultrasonic sensor GPIO pins
#define ULTRASONIC_GPIO_TRIGGER     25
#define ULTRASONIC_GPIO_ECHO        26

// Temperature sensor GPIO pin
#define TEMPERATURE_GPIO_DATA   33

#define UPDATE_SOUND_SPEED 5000 // update speed of sound at that interval in [us]

#define READ_ULTRASONIC_MS      50  // [ms]
#define READ_ULTRASONIC_AVG_MS  100 // [ms]

static const char* TAG = "Main script";

void update_sound_speed()
{
    // Init dht22 ultrasonic_sensor
    ESP_ERROR_CHECK(dht22_init(TEMPERATURE_GPIO_DATA));

    while(1)
    { 
        int ret = read_dht22();
        if (ret == 0)
        {
            set_sound_speed(get_temperature(), get_humidity());
        }
        else
        {
            ESP_LOGE(TAG, "ERROR update sound speed %d -> name:\n", ret);
        }
        vTaskDelay(UPDATE_SOUND_SPEED / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void ultrasonic_read()
{
    ultrasonic_sensor_t ultrasonic_sensor = { ULTRASONIC_GPIO_TRIGGER, ULTRASONIC_GPIO_ECHO };

    // Init ultrasonic sensor
    hc_sr04_init(&ultrasonic_sensor);

    while(true)
    {
        float distance;

        esp_err_t res = measure(&ultrasonic_sensor, &distance);

        // Handle error
        if (res != ESP_OK)
        {
            printf("Error: ");
		    switch (res) 
            {
		    	case ESP_ERR_ULTRASONIC_SENSOR_PING:
		    		printf("Cannot ping (device is in invalid state)\n");
		    		break;
		    	case ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT:
		    		printf("Ping timeout (no device found)\n");
		    		break;
		    	case ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT:
		    		printf("Echo timeout (i.e. distance too big)\n");
		    		break;
		    	default:
		    		printf("%d\n", res);
		    }
        }
        else
        {
            printf("distance [cm]: %0.5f\n", distance * 100);
     
        }
        printf("-------------------------------\n");

        vTaskDelay(READ_ULTRASONIC_MS / portTICK_PERIOD_MS);
    }    
}

void ultrasonic_read_avg()
{
    ultrasonic_sensor_t ultrasonic_sensor = { ULTRASONIC_GPIO_TRIGGER, ULTRASONIC_GPIO_ECHO };

    // Init ultrasonic sensor
    hc_sr04_init(&ultrasonic_sensor);

    while(true)
    {
        float distance;

        esp_err_t res = measure_avg(&ultrasonic_sensor, &distance, 1000000, 20);

        // Handle error
        if (res != ESP_OK)
        {
            printf("Error: ");
		    switch (res) 
            {
		    	case ESP_ERR_ULTRASONIC_SENSOR_PING:
		    		printf("Cannot ping (device is in invalid state)\n");
		    		break;
		    	case ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT:
		    		printf("Ping timeout (no device found)\n");
		    		break;
		    	case ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT:
		    		printf("Echo timeout (i.e. distance too big)\n");
		    		break;
                case ESP_ERR_TIMEOUT:
                    printf("Timeout\n");
                    break;
		    	default:
		    		printf("%d\n", res);
		    }
        }
        else
        {
            printf("distance [cm]: %0.5f\n", distance * 100);
     
        }
        printf("-------------------------------\n");

        vTaskDelay(READ_ULTRASONIC_AVG_MS / portTICK_PERIOD_MS);
    }    
}


void app_main(void)
{


    xTaskCreate(&update_sound_speed, "Update sound speed", 2048, NULL, 0, NULL);
    //xTaskCreatePinnedToCore(&update_sound_speed, "Update sound speed", 2048, NULL, 2, NULL, 1);
    // xTaskCreate(&ultrasonic_read, "Ultrasonic read", 8192, NULL, 2, NULL);
    xTaskCreate(&ultrasonic_read_avg, "Ultrasonic read avg", 8192, NULL, 2, NULL);
}
