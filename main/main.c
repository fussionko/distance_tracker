#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

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
#include <string.h>
#include <math.h>


#include "temperature_sensor.h"

// Temp values
#define GPIO_TRIGGER    4
#define GPIO_ECHO_LEFT  15
#define GPIO_ECHO_RIGHT 2

// CMD codes
#define CMD_START   100 
#define CMD_STOP    200 
#define CMD_MEASURE 300
#define CMD_CLEAR   400

QueueHandle_t xQueueCmd;

// CMD structure
typedef struct 
{
    uint16_t command;
    uint32_t distance_left, distance_right;
    TaskHandle_t taskHandle;
} CMD_t;

void ultrasonic_read()
{
    ESP_LOGI(pcTaskGetName(0), "Start");
    CMD_t cmdBuf;
    cmdBuf.command = CMD_MEASURE;
    cmdBuf.taskHandle = xTaskGetCurrentTaskHandle();

    ultrasonic_sensor_t ultrasonic_sensor = { GPIO_TRIGGER, GPIO_ECHO_LEFT, GPIO_ECHO_RIGHT };

    ultrasonic_sensor_init(&ultrasonic_sensor);


    interrupt_init(&ultrasonic_sensor);
   // interrupt_enable(&ultrasonic_sensor);

    

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    // Create ping timeout timer
    esp_timer_handle_t ping_timer;
        struct timeval tv;
    gettimeofday(&tv, NULL);
    event_t timer_arg = { EVENT_ULTRASONIC_SENSOR_PING_TIMEOUT, 2, tv.tv_usec };
    create_timer(&ping_timer, timer_callback, "Ping timer", &timer_arg);


    // Infinite loop -> prob change in future to certain amount of repetition
    ESP_LOGI("loop", "START");
    while(true)
    {
        ESP_LOGI("trigger", "START");
        trigger(&ultrasonic_sensor);

        uint32_t distance_left, distance_right;
        ESP_LOGI("measure", "START");
        esp_err_t res = measure(&ping_timer, &distance_left, &distance_right);

        // 
        // esp_err_t res = ultrasonic_measure_cm(&ultrasonic_sensor, &distance_left, &distance_right);

        // Handle error
        ESP_LOGI("handle_error", "START");
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

            float dis_left = (float)(distance_left);
            float dis_right = (float)(distance_right);
            printf("Distance LEFT: %"PRIu32" cm, %.02f, float %.4f m\n", distance_left, distance_left / 100.0, dis_left);
            printf("Distance RIGHT: %"PRIu32" cm, %.02f, float %.4f m\n", distance_right, distance_right / 100.0, dis_right); 


            // float f = dis_left * dis_left - dis_right * dis_right;
            // float formula_x = -(f + 12.25)/7.0;
            // float formula_y = sqrt(dis_left * dis_left - formula_x * formula_x);

            if (fabs(dis_left - dis_right) > 3.5)
                ESP_LOGI("ERROR", "wrong read");
            else
            {
                // Improve with << and >> 
                float dis_left2 = pow(dis_left, 2);
                float dis_right2 = pow(dis_right, 2);

                float formula_x = (dis_left2 - dis_right2) / 7.0f;
                float formula_y = sqrt(dis_right2 - pow(1.75f - formula_x, 2));

                printf("formula_x: %f\n", formula_x);
                printf("formula_y: %f\n", formula_y);
            }



            cmdBuf.distance_left = distance_left;
            cmdBuf.distance_right = distance_right;
            xQueueSend(xQueueCmd, &cmdBuf, 0);
        }
        printf("-------------------------------\n");

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }    
}

void DHT_read_task(void* pvParams)
{
    init_dht22(GPIO_NUM_33);
    while(1) {
	
		printf("DHT Sensor Readings\n" );
		int ret = read_dht22();

		printf("Humidity %.2f %%\n", get_humidity());
		printf("Temperature %.2f degC\n\n", get_temperature());
        printf("Err: %d\n", ret);
		
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
}

void app_main(void)
{
	/* Create Queue */
	xQueueCmd = xQueueCreate(10, sizeof(CMD_t));
	configASSERT(xQueueCmd);

    //xTaskCreate(&ultrasonic_read, "ultrasonic read", 1024*2, NULL, 2, NULL);
    xTaskCreate(&DHT_read_task, "DHT_reader_task", 2048, NULL, 5, NULL);
}
