#include "ultrasonic_sensor.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sys/time.h"

#include "esp_check.h"


#define LEFT_SIDE   0
#define RIGHT_SIDE  1
#define SIDE_NONE   2

#define TRIGGER_LOW_DELAY   2       // low delay [us]
#define TRIGGER_HIGH_DELAY  10      // high delay [us]
#define PING_TIMEOUT_TIME   50000   // ping timeout [us]



//#define timeout_expired(start, len) ((uint32_t)get_time_us() - start) >= len
//#define RETURN_CRITICAL(MUX, RES, STORE) do { portEXIT_CRITICAL(&MUX); STORE->error = RES; vTaskDelete(NULL); } while (0)

static const char* TAG = "HC-SR04";

static float soundSpeed;        // [m/s]
static uint64_t echoReplyTimeout;  // [us]

// Ping timeout timer handle
static esp_timer_handle_t ping_timeout_timer_handle;

// Echo timeout timer handels
static esp_timer_handle_t echo_timeout_timer_handle[2];

QueueHandle_t queueEventData;

void set_sound_speed(float temperature, float humidity)
{
    ESP_LOGI(TAG, "Set sound speed");
    // Calculate speed of sound
    soundSpeed = 331.3 + 0.6 * temperature;

    // Calculate max echo reply timeout
    // t = MAX_DISTANCE / soundSpeed * 10^6;
    echoReplyTimeout = (uint64_t)ceil((MAX_DISTANCE / soundSpeed) * 1e6) * 2;
}

static void IRAM_ATTR intr_handler_left(void* args)
{   
    const event_t event = { gpio_get_level(*(gpio_num_t*)args), LEFT_SIDE, esp_timer_get_time() };
    // ESP_EARLY_LOGI(TAG, "intr left callback");
    xQueueSendFromISR(queueEventData, &event, NULL);
}

static void IRAM_ATTR intr_handler_right(void* args)
{
    const event_t event = { gpio_get_level(*(gpio_num_t*)args), RIGHT_SIDE, esp_timer_get_time() };
    // ESP_EARLY_LOGI(TAG, "intr right callback");
    xQueueSendFromISR(queueEventData, &event, NULL);
}

esp_err_t interrupt_enable(const ultrasonic_sensor_t* sensor)
{
    ESP_RETURN_ON_ERROR(gpio_intr_enable(sensor->echo_left), "gpio_intr_enable echo left", "left");
    ESP_RETURN_ON_ERROR(gpio_intr_enable(sensor->echo_right), "gpio_intr_enable echo right", "right");
    return ESP_OK;
}

esp_err_t interrupt_disable(const ultrasonic_sensor_t* sensor)
{
    ESP_RETURN_ON_ERROR(gpio_intr_disable(sensor->echo_left), "gpio_intr_disable echo left", "left");
    ESP_RETURN_ON_ERROR(gpio_intr_disable(sensor->echo_right), "gpio_intr_disable echo right", "right");
    return ESP_OK;
}

void timeout_timer_callback(void* args)
{
    // ESP_EARLY_LOGI(TAG, "timer callback");
    xQueueSendToFrontFromISR(queueEventData, args, NULL);
}


esp_err_t trigger(const ultrasonic_sensor_t* sensor)
{
    // Triggers the ultrasonic sensor

    gpio_set_level(sensor->trigger, 0);
    esp_rom_delay_us(TRIGGER_LOW_DELAY);
    gpio_set_level(sensor->trigger, 1);               
    esp_rom_delay_us(TRIGGER_HIGH_DELAY);
    gpio_set_level(sensor->trigger, 0);


    return ESP_OK;
}

#define START 0
#define END 1

esp_err_t measure(const ultrasonic_sensor_t* sensor, float* distance_left, float* distance_right)
{
    ESP_LOGI(TAG, "Start measure");

  



    event_t event;

    // Init values to 0
    uint64_t time[2][2] = { 0 };

    // Start ping timeout timer
    esp_timer_start_once(ping_timeout_timer_handle, PING_TIMEOUT_TIME + TRIGGER_LOW_DELAY + TRIGGER_HIGH_DELAY);

    // Start trigger
    trigger(sensor);
    interrupt_enable(sensor);

    ESP_LOGI(TAG, "Start measure");
    while(1)
    {
        // Check if event was added to queue
        if (xQueueReceive(queueEventData, &event, 0))
        {
            ESP_LOGI(TAG, "queue recieved event code: %d,  side: %d", (int)event.event_code, event.side);

            if (event.event_code == ECHO_START)
            {    
                time[event.side][START] = event.time;      
                
                if (esp_timer_is_active(echo_timeout_timer_handle[event.side]))
                {
                    esp_timer_stop(echo_timeout_timer_handle[event.side]);

                    if (esp_timer_is_active(echo_timeout_timer_handle[!event.side]))
                    {
                        esp_timer_stop(echo_timeout_timer_handle[!event.side]);
                    } 

                    interrupt_disable(sensor);
                    return ESP_ERR_ULTRASONIC_SENSOR_ECHO_ERROR;
                }

                
                // Start echo timeout timer
                esp_timer_start_once(echo_timeout_timer_handle[event.side], echoReplyTimeout);
            }
            else if (event.event_code == ECHO_END)
            {
                time[event.side][END] = event.time;
                
                if (esp_timer_is_active(echo_timeout_timer_handle[event.side]))
                {
                    esp_timer_stop(echo_timeout_timer_handle[event.side]);
                }
                
                // Better solution???
                if (time[!event.side][END] != 0)   
                {
                    esp_timer_stop(ping_timeout_timer_handle);
                    break;
                }  
            }
            else if(event.event_code == PING_TIMEOUT)
            {
                // timeout

                // Kill both echo timers if active
                if (esp_timer_is_active(echo_timeout_timer_handle[LEFT_SIDE])) esp_timer_stop(echo_timeout_timer_handle[LEFT_SIDE]);
                if (esp_timer_is_active(echo_timeout_timer_handle[RIGHT_SIDE])) esp_timer_stop(echo_timeout_timer_handle[RIGHT_SIDE]);

                // Kill ping timer if active
                if (esp_timer_is_active(ping_timeout_timer_handle)) esp_timer_stop(ping_timeout_timer_handle);

                interrupt_disable(sensor);
                return ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT;
            }
            else if(event.event_code == ECHO_TIMEOUT)
            {
                // Kill both echo timers if active
                if (esp_timer_is_active(echo_timeout_timer_handle[LEFT_SIDE])) esp_timer_stop(echo_timeout_timer_handle[LEFT_SIDE]);
                if (esp_timer_is_active(echo_timeout_timer_handle[RIGHT_SIDE])) esp_timer_stop(echo_timeout_timer_handle[RIGHT_SIDE]);

                // Kill ping timer if active
                if (esp_timer_is_active(ping_timeout_timer_handle)) esp_timer_stop(ping_timeout_timer_handle);

                interrupt_disable(sensor);
                return ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT;    
            }
        }
    }
    
    interrupt_disable(sensor);


    // distance = speed of sound * (time_end - time_start)  / (2 * 10^6) -> [m/s]
    *distance_left = soundSpeed * (float)(time[LEFT_SIDE][END] - time[LEFT_SIDE][START]) / 2e6; 
    *distance_right = soundSpeed * (float)(time[RIGHT_SIDE][END] - time[RIGHT_SIDE][START]) / 2e6; 
    
    ESP_LOGI(TAG, "distance left: %f | distance right: %f", *distance_left, *distance_right);
    ESP_LOGI(TAG, "speed: %f time_left: %"PRIu64", %"PRIu64"", soundSpeed, time[LEFT_SIDE][START], time[LEFT_SIDE][END]);

    return ESP_OK;
}



void ultrasonic_sensor_init(const ultrasonic_sensor_t* sensor)
{
    ESP_LOGI(TAG, "Ultrasonic sensor init start");

    gpio_reset_pin(sensor->trigger);
    gpio_reset_pin(sensor->echo_left);
    gpio_reset_pin(sensor->echo_right);
    gpio_set_direction(sensor->trigger, GPIO_MODE_OUTPUT);
    gpio_set_direction(sensor->echo_left, GPIO_MODE_INPUT);
    gpio_set_direction(sensor->echo_right, GPIO_MODE_INPUT);
    gpio_set_level(sensor->trigger, 0);

    ESP_LOGI(TAG, "Ultrasonic sensor init end");

    ESP_LOGI(TAG, "Interrupt setup start");

    // Interrupt is triggerd on rising or falling edge
    gpio_set_intr_type(sensor->echo_left, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(sensor->echo_right, GPIO_INTR_ANYEDGE);

    gpio_pulldown_dis(sensor->echo_left);
    gpio_pullup_dis(sensor->echo_left);

    gpio_pulldown_dis(sensor->echo_right);
    gpio_pullup_dis(sensor->echo_right);

    // Allocate resources
    gpio_install_isr_service(0);

    // Add isr handlers
    gpio_isr_handler_add(sensor->echo_left, intr_handler_left, (void*)&sensor->echo_left);
    gpio_isr_handler_add(sensor->echo_right, intr_handler_right, (void*)&sensor->echo_right); // change to right

    interrupt_disable(sensor);

    //interrupt_disable(sensor);
    ESP_LOGI(TAG, "Interrupt setup end");

    // Create data queue
    ESP_LOGI(TAG, "Create queue");
    queueEventData = xQueueCreate(5, sizeof(event_t));

    // Create main ping timeout timer
    ESP_LOGI(TAG, "Create timers");
    event_t ping_timeout_timer_arg = { PING_TIMEOUT, SIDE_NONE, esp_timer_get_time() };
    const esp_timer_create_args_t ping_timeout_timer_args = 
    {
        .callback = &timeout_timer_callback,
        .name = "Ping timeout timer",
        .arg = &ping_timeout_timer_arg
    };
    ESP_ERROR_CHECK(esp_timer_create(&ping_timeout_timer_args, &ping_timeout_timer_handle));

    // Create 2 timeout timers for each echo reply
    event_t echo_timeout_timer_arg = { ECHO_TIMEOUT, SIDE_NONE, esp_timer_get_time() };
    const esp_timer_create_args_t echo_timeout_timer_args = 
    {
        .callback = &timeout_timer_callback,
        .name = "Echo timeout timer",
        .arg = &echo_timeout_timer_arg
    };

    event_t* event = (event_t*)echo_timeout_timer_args.arg;
    ESP_LOGI(TAG, "%d, %d", event->event_code, event->side);

    ESP_ERROR_CHECK(esp_timer_create(&echo_timeout_timer_args, &echo_timeout_timer_handle[LEFT_SIDE]));
    ESP_ERROR_CHECK(esp_timer_create(&echo_timeout_timer_args, &echo_timeout_timer_handle[RIGHT_SIDE]));


}

