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

#include "ultrasonic_sensor.h"

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

    


    // Create ping timeout timer
    esp_timer_handle_t ping_timer;
    event_t timer_arg = { EVENT_ULTRASONIC_SENSOR_PING_TIMEOUT, SIDE_NONE };
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
            printf("Distance LEFT: %"PRIu32" cm, %.02f m\n", distance_left, distance_left / 100.0);
            printf("Distance RIGHT: %"PRIu32" cm, %.02f m\n", distance_right, distance_right / 100.0); 

            cmdBuf.distance_left = distance_left;
            cmdBuf.distance_right = distance_right;
            xQueueSend(xQueueCmd, &cmdBuf, 0);
        }
        printf("-------------------------------\n");

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }    
}

void app_main(void)
{
	/* Create Queue */
	xQueueCmd = xQueueCreate(10, sizeof(CMD_t));
	configASSERT(xQueueCmd);

    xTaskCreate(&ultrasonic_read, "ultrasonic read", 1024*2, NULL, 2, NULL);
}
