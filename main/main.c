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

// GPIO pins
#define GPIO_TRIGGER    4
#define GPIO_ECHO_LEFT  15
#define GPIO_ECHO_RIGHT 2

#define DHT22_SENSOR 33

#define READ_ULTRASONIC_MS 2000 // read [ms]
#define UPDATE_SOUND_SPEED 8000000 // update speed of sound at that interval in [us]

#define DISTANCE_TX_RX_M 0.0185f // distance from transmittor to reciever [m]


static const char* TAG = "Main script";

void update_sound_speed()
{
	ESP_LOGI(TAG, "update sound speed");
	int ret = read_dht22();
	if (ret == 0)
    {
        set_sound_speed(get_temperature(), get_humidity());
    }
    else
    {
        ESP_LOGE(TAG, "ERROR update sound speed %d\n", ret);
    }
}

void ultrasonic_read()
{
    ESP_LOGI(TAG, "Start");

    ultrasonic_sensor_t ultrasonic_sensor = { GPIO_TRIGGER, GPIO_ECHO_LEFT, GPIO_ECHO_RIGHT };

    // Init ultrasonic sensor
    ultrasonic_sensor_init(&ultrasonic_sensor);

    // Infinite loop -> prob change in future to certain amount of repetition
    ESP_LOGI(TAG, "Start main loop");
    while(true)
    {
        float distance_left, distance_right;
        ESP_LOGI(TAG, "Start measure");
        esp_err_t res = measure(&ultrasonic_sensor, &distance_left, &distance_right);

        // Handle error
        ESP_LOGI(TAG, "Start error handle");
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
            // [m/s]
            printf("Distance LEFT: %.05f cm\n", distance_left * 100);
            printf("Distance RIGHT: %.05f cm\n", distance_right * 100); 

            if (fabs(distance_left - distance_right) > (DISTANCE_TX_RX_M * 2))
                ESP_LOGW(TAG, "ERROR wrong read");
            else
            {
                float dis_left2 = pow(distance_left, 2);
                float dis_right2 = pow(distance_right, 2);

                float formula_x = (dis_left2 - dis_right2) / (DISTANCE_TX_RX_M * 4);
                float formula_y = sqrt(dis_right2 - pow(DISTANCE_TX_RX_M - formula_x, 2));

                if (pow(DISTANCE_TX_RX_M - formula_x, 2) < 0)
                    ESP_LOGW(TAG, "ERROR wrong read");
                else
                {
                    printf("formula_x: %f\n", formula_x);
                    printf("formula_y: %f\n", formula_y);

                    printf("data: %f %f\n", dis_left2, dis_right2);
                }
  
            }
        }
        printf("-------------------------------\n");

        vTaskDelay(READ_ULTRASONIC_MS / portTICK_PERIOD_MS);
    }    
}



void app_main(void)
{
    // Setup timer
    const esp_timer_create_args_t update_temp_timer_args = 
    {
        .callback = &update_sound_speed,
        .name = "Update sound speed",
    };

    esp_timer_handle_t update_temp_timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&update_temp_timer_args, &update_temp_timer_handle));

    // Init dht22 sensor
    init_dht22(DHT22_SENSOR);
    esp_timer_start_periodic(update_temp_timer_handle, UPDATE_SOUND_SPEED);

    
    xTaskCreate(&ultrasonic_read, "Ultrasonic read", 2048, NULL, 2, NULL);
}
