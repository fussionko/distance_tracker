#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "ultrasonic_sensor.h"

// Temp values
#define GPIO_TRIGGER    1
#define GPIO_ECHO_LEFT  2
#define GPIO_ECHO_RIGHT 3

// CMD codes
#define CMD_START   100 
#define CMD_STOP    200 
#define CMD_MEASURE 300
#define CMD_CLEAR   400

// CMD structure
typedef struct 
{
    uint16_t command;
    uint32_t distance;
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

    // Infinite loop -> prob change in future to certain amount of repetition
    while(true)
    {
        uint32_t distance_left, distance_right;
        esp_err_t res = ultrasonic_measure_cm(&ultrasonic_sensor, &distance_left, &distance_right);

        
    }    
}

void app_main(void)
{
	/* Create Queue */
	xQueueCmd = xQueueCreate( 10, sizeof(CMD_t) );
	configASSERT( xQueueCmd );
    	xTaskCreate(ultrasonic, "ultrasonic", 1024*2, NULL, 2, NULL);
	xTaskCreate(tft, "TFT", 1024*4, NULL, 2, NULL);
}
