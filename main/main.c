#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "sys/time.h"
#include "ultrasonic_sensor.h"
#include "temperature_sensor.h"
#include "esp_check.h"


// GPIO pins
#define GPIO_TRIGGER    25
#define GPIO_ECHO       26

#define GPIO_DHT22      33

#define READ_ULTRASONIC_MS 2000 // read [ms]
#define UPDATE_SOUND_SPEED 5000 //5000000 // update speed of sound at that interval in [us]

static const char* TAG = "Main script";

void update_sound_speed()
{
    // Init dht22 ultrasonic_sensor
    ESP_ERROR_CHECK(init_dht22(GPIO_DHT22));


    int ret = read_dht22();
    if (ret == 0)
    {
        ESP_LOGI(TAG, "123123");
        set_sound_speed(get_temperature(), get_humidity());
    }
    else
    {
        ESP_LOGI(TAG, "111111");
        ESP_LOGE(TAG, "ERROR update sound speed %d -> name:\n", ret);
    }
    while(1)
    {
        vTaskDelay(UPDATE_SOUND_SPEED / portTICK_PERIOD_MS);
        ret = read_dht22();
            if (ret == 0)
            {
                ESP_LOGI(TAG, "123123");
                set_sound_speed(get_temperature(), get_humidity());
            }
            else
            {
                ESP_LOGI(TAG, "111111");
                ESP_LOGE(TAG, "ERROR update sound speed %d -> name:\n", ret);
            }
    }
}



void ultrasonic_read()
{
    ESP_LOGI(TAG, "Start");

    ultrasonic_sensor_t ultrasonic_sensor = { GPIO_TRIGGER, GPIO_ECHO };

    ultrasonic_sensor_init(&ultrasonic_sensor);

    // Infinite loop -> prob change in future to certain amount of repetition
    ESP_LOGI(TAG, "Start main loop");
    while(true)
    {
        float distance;

        // ESP_LOGI(TAG, "Start measure");
        esp_err_t res = measure(&ultrasonic_sensor, &distance);

        // Handle error
        // ESP_LOGI(TAG, "Start error handle");
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


void app_main(void)
{


    //xTaskCreate(&update_sound_speed, "Update sound speed", 2048, NULL, 2, NULL);
    //xTaskCreatePinnedToCore(&update_sound_speed, "Update sound speed", 2048, NULL, 2, NULL, 1);
    xTaskCreate(&ultrasonic_read, "Ultrasonic read", 8192, NULL, 2, NULL);
}
