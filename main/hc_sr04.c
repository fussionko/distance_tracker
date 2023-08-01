#include "ultrasonic_sensor.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_check.h"

#define TRIGGER_LOW_DELAY   2           // low delay [us]
#define TRIGGER_HIGH_DELAY  10          // high delay [us]
#define PING_TIMEOUT_TIME   100000      // ping timeout [us] ->0.1s

#define SOUND_SPEED_START           331.0f      // [m/s]
#define ECHO_REPLY_TIMEOUT_START    25000       // [us]

//#define timeout_expired(start, len) ((uint32_t)get_time_us() - start) >= len
//#define RETURN_CRITICAL(MUX, RES, STORE) do { portEXIT_CRITICAL(&MUX); STORE->error = RES; vTaskDelete(NULL); } while (0)

static const char* TAG = "HC-SR04";

static float soundSpeed;            // [m/s]
static uint64_t echoReplyTimeout;   // [us]

// // Ping timeout timer handle
static esp_timer_handle_t ping_timeout_timer_handle;

// // Echo timeout timer handle
static esp_timer_handle_t echo_timeout_timer_handle;

QueueHandle_t queueEventData;

void set_sound_speed(float temperature, float humidity)
{
    // ESP_LOGI(TAG, "Set sound speed");
    // Calculate speed of sound https://phys.libretexts.org/Bookshelves/University_Physics/Book%3A_University_Physics_(OpenStax)/Book%3A_University_Physics_I_-_Mechanics_Sound_Oscillations_and_Waves_(OpenStax)/17%3A_Sound/17.03%3A_Speed_of_Sound
    soundSpeed = SOUND_SPEED_START * sqrt(1 + (temperature / 273));//331.3 + 0.6 * temperature;

    // Calculate max echo reply timeout
    // Max sound speed ~ 380m/s     Min sound speed ~ 305m/s
    // t = ((MAX_DISTANCE * 10^6) / soundSpeed) * 2;
    echoReplyTimeout = ((uint64_t)ceil((MAX_DISTANCE * 1e6) / soundSpeed)) << 1;//(uint64_t)ceil((MAX_DISTANCE / soundSpeed) * 1e6) * 2;
}

static void IRAM_ATTR echo_intr_handler(void* args)
{   
    const event_t event = { gpio_get_level(*(gpio_num_t*)args), esp_timer_get_time() };
    xQueueSendFromISR(queueEventData, &event, NULL);
}


// esp_err_t interrupt_enable(const ultrasonic_sensor_t* sensor)
// {
//     ESP_RETURN_ON_ERROR(gpio_intr_enable(sensor->echo), TAG, "gpio_intr_enable echo left");
//     return ESP_OK;
// }

// esp_err_t interrupt_disable(const ultrasonic_sensor_t* sensor)
// {
//     ESP_RETURN_ON_ERROR(gpio_intr_disable(sensor->echo), TAG, "gpio_intr_disable echo");
//     return ESP_OK;
// }

void ping_timeout_timer_callback(void* args)
{
    event_t event = { PING_TIMEOUT, esp_timer_get_time() };
    xQueueSendFromISR(queueEventData, &event, NULL);
}

void echo_timeout_timer_callback(void* args)
{
    event_t event = { ECHO_TIMEOUT, esp_timer_get_time() };
    xQueueSendFromISR(queueEventData, &event, NULL);
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

esp_err_t measure(const ultrasonic_sensor_t* sensor, float* distance)
{
    xQueueReset(queueEventData);

    event_t event;

    // Init values to 0
    uint64_t time[2] = { 0 };

    // Start ping timeout timer
    esp_timer_start_once(ping_timeout_timer_handle, PING_TIMEOUT_TIME + TRIGGER_LOW_DELAY + TRIGGER_HIGH_DELAY);

    // Start trigger
    trigger(sensor);
    
    while(1)
    {
        // Check if event was added to queue
        if (xQueueReceive(queueEventData, &event, 0))
        {
            //ESP_LOGI(TAG, "queue recieved event code: %d", (int)event.event_code);
            if (event.event_code == ECHO_START)
            {    
                time[START] = event.time;      
                
                if (esp_timer_is_active(echo_timeout_timer_handle))
                {
                    esp_timer_stop(echo_timeout_timer_handle);

                    // Kill ping timer if active
                    if (esp_timer_is_active(ping_timeout_timer_handle)) esp_timer_stop(ping_timeout_timer_handle);
                    
                    return ESP_ERR_ULTRASONIC_SENSOR_ECHO_ERROR;
                }

                // Start echo timeout timer
                esp_timer_start_once(echo_timeout_timer_handle, echoReplyTimeout);
            }
            else if (event.event_code == ECHO_END)
            {
                time[END] = event.time;
                
                if (esp_timer_is_active(echo_timeout_timer_handle))
                {
                    esp_timer_stop(echo_timeout_timer_handle);
                }
                
                esp_timer_stop(ping_timeout_timer_handle);

                break;
            }
            else if(event.event_code == PING_TIMEOUT)
            {
                // timeout

                // Kill echo timer if active
                if (esp_timer_is_active(echo_timeout_timer_handle)) esp_timer_stop(echo_timeout_timer_handle);
        
                // Kill ping timer if active
                if (esp_timer_is_active(ping_timeout_timer_handle)) esp_timer_stop(ping_timeout_timer_handle);
                
                return ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT;
            }
            else if(event.event_code == ECHO_TIMEOUT)
            {
                // Kill echo timer if active
                if (esp_timer_is_active(echo_timeout_timer_handle)) esp_timer_stop(echo_timeout_timer_handle);
                

                // Kill ping timer if active
                if (esp_timer_is_active(ping_timeout_timer_handle)) esp_timer_stop(ping_timeout_timer_handle);
                
                return ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT;    
            }
        }
    }
    
    // distance = speed of sound * (time_end - time_start)  / (2 * 10^6) -> [m/s]
    *distance = soundSpeed * (float)(time[END] - time[START]) / 2e6; 
    
    return ESP_OK;
}

void ultrasonic_sensor_init(const ultrasonic_sensor_t* sensor)
{
    ESP_LOGI(TAG, "Ultrasonic sensor init start");

    // Reset gpio pins
    ESP_ERROR_CHECK(gpio_reset_pin(sensor->trigger));
    ESP_ERROR_CHECK(gpio_reset_pin(sensor->echo));

    // Set direction
    ESP_ERROR_CHECK(gpio_set_direction(sensor->trigger, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(sensor->echo, GPIO_MODE_INPUT));

    ESP_ERROR_CHECK(gpio_set_level(sensor->trigger, 0));

    ESP_LOGI(TAG, "Interrupt setup start");

    // Interrupt is triggerd on rising or falling edge
    ESP_ERROR_CHECK(gpio_set_intr_type(sensor->echo, GPIO_INTR_ANYEDGE));

    ESP_ERROR_CHECK(gpio_pulldown_dis(sensor->echo));
    ESP_ERROR_CHECK(gpio_pullup_dis(sensor->echo));

    ESP_ERROR_CHECK(gpio_pulldown_dis(sensor->trigger));
    ESP_ERROR_CHECK(gpio_pullup_dis(sensor->trigger));

    // Allocate resources
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Add isr handlers
    ESP_ERROR_CHECK(gpio_isr_handler_add(sensor->echo, echo_intr_handler, (void*)&sensor->echo));

    // Create data queue
    ESP_LOGI(TAG, "Create queue");
    queueEventData = xQueueCreate(5, sizeof(event_t));

    // Create main ping timeout timer
    ESP_LOGI(TAG, "Create timers");
    
    const esp_timer_create_args_t ping_timeout_timer_args = 
    {
        .callback = &ping_timeout_timer_callback,
        .name = "Ping timeout timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&ping_timeout_timer_args, &ping_timeout_timer_handle));

    // Create  timeout timer for echo reply
    const esp_timer_create_args_t echo_timeout_timer_args = 
    {
        .callback = &echo_timeout_timer_callback,
        .name = "Echo timeout timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&echo_timeout_timer_args, &echo_timeout_timer_handle));


    soundSpeed = SOUND_SPEED_START;
    echoReplyTimeout = ECHO_REPLY_TIMEOUT_START;
}

