#include "ultrasonic_sensor.h"

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
#define PING_TIMEOUT        500000  // ping timeout [us]



//#define timeout_expired(start, len) ((uint32_t)get_time_us() - start) >= len
//#define RETURN_CRITICAL(MUX, RES, STORE) do { portEXIT_CRITICAL(&MUX); STORE->error = RES; vTaskDelete(NULL); } while (0)

static const char* TAG = "HC-SR04";

static float soundSpeed; // [m/s]

// Ping timeout timer handle
static esp_timer_handle_t ping_timeout_timer_handle;

// Echo timeout timer handels
static esp_time_handle_t echo_timeout_timer_left_handle;
static esp_time_handle_t echo_timeout_timer_right_handle;

QueueHandle_t queueEventData;

void set_sound_speed(float temperature, float humidity)
{
    soundSpeed = 331.3 + 0.6 * temperature;
}

static void IRAM_ATTR intr_handler_left(void* args)
{   
    event_t event = { gpio_get_level(*(gpio_num_t*)args), LEFT_SIDE, esp_timer_get_time() };
    xQueueSendFromISR(queueEventData, &event, NULL);
}

static void IRAM_ATTR intr_handler_right(void* args)
{
    event_t event = { gpio_get_level(*(gpio_num_t*)args), RIGHT_SIDE, esp_timer_get_time() };
    xQueueSendFromISR(queueEventData, &event, NULL);
}

// esp_err_t interrupt_enable(const ultrasonic_sensor_t* sensor)
// {
//     ESP_RETURN_ON_ERROR(gpio_intr_enable(sensor->echo_left), "gpio_intr_enable echo left", "left");
//     ESP_RETURN_ON_ERROR(gpio_intr_enable(sensor->echo_right), "gpio_intr_enable echo right", "right");
//     return ESP_OK;
// }

// esp_err_t interrupt_disable(const ultrasonic_sensor_t* sensor)
// {
//     ESP_RETURN_ON_ERROR(gpio_intr_disable(sensor->echo_left), "gpio_intr_disable echo left", "left");
//     ESP_RETURN_ON_ERROR(gpio_intr_disable(sensor->echo_right), "gpio_intr_disable echo right", "right");
//     return ESP_OK;
// }



void timeout_timer_callback(void* args)
{
    xQueueSendToFrontFromISR(queueEventData, (event_t*)args, NULL);
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
    trigger(sensor);

    event_t event;

    // Init values to 0
    uint64_t time[2][2] = { 0 };

    // Start ping timeout timer
    esp_timer_start_once(ping_timeout_timer_handle, PING_TIMEOUT);

    ESP_LOGI(TAG, "Start measure");
    while(1)
    {
        // Check if event was added to queue
        if (xQueueReceive(queueEventData, &event, 0))
        {
            ESP_LOGI(TAG, "queue recieved event");
            if (event.event_code == 1)
            {    
                time[event.side][START] = event.time;      

            }
            else if (event.event_code == 0)
            {
                time[event.side][END] = event.time;

                if (time[LEFT_SIDE][START] != 0 && time[RIGHT_SIDE][START] != 0 && time[LEFT_SIDE][1] != 0 && time[RIGHT_SIDE][END] != 0)
                {
                    esp_timer_stop(ping_timeout_timer_handle);

                    break;
                } 
            }
            else if(event.event_code == 2)
            {
                // timeout
                return ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT;
            }
        }
    }
    
    // distance = speed of sound * ((time_end - time_start) / 2) / 10^6 -> [m/s]
    *distance_left = soundSpeed * (float)((time[LEFT_SIDE][START] - time[LEFT_SIDE][START]) >> 1) / 1e6; 
    *distance_right = soundSpeed * (float)((time[RIGHT_SIDE][END] - time[RIGHT_SIDE][END]) >> 1) / 1e6; 
    
    return ESP_OK;
}



void ultrasonic_sensor_init(const ultrasonic_sensor_t* sensor)
{
    ESP_LOGI("ultrasonic_sensor_init", "START");
    gpio_reset_pin(sensor->trigger);
    gpio_reset_pin(sensor->echo_left);
    gpio_reset_pin(sensor->echo_right);
    gpio_set_direction(sensor->trigger, GPIO_MODE_OUTPUT);
    gpio_set_direction(sensor->echo_left, GPIO_MODE_INPUT);
    gpio_set_direction(sensor->echo_right, GPIO_MODE_INPUT);

    gpio_set_level(sensor->trigger, 0);
    ESP_LOGI("ultrasonic_sensor_init", "END");

    ESP_LOGI("intr_init", "START");

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

    //interrupt_disable(sensor);
    ESP_LOGI("intr_init", "END");

    // Create data queue
    queueEventData = xQueueCreate(4, sizeof(event_t));

    // Create main ping timeout timer
    event_t ping_timeout_timer_arg = { EVENT_ULTRASONIC_SENSOR_PING_TIMEOUT, 2, esp_timer_get_time() };
    const esp_timer_create_args_t ping_timeout_timer_args = 
    {
        .callback = &timeout_timer_callback,
        .name = "Ping timeout timer",
        .arg = (void*)&ping_timeout_timer_arg
    };
    ESP_ERROR_CHECK(esp_timer_create(&ping_timeout_timer_args, ping_timeout_timer_handle));

    // Create 2 timeout timers for each echo reply
    event_t echo_timeout_timer_arg = { ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT, 2, esp_timer_get_time() };
    const esp_timer_create_args_t echo_timeout_timer_args = 
    {
        .callback = &timeout_timer_callback,
        .name = "Echo timeout timer",
        .arg = (void*)&echo_timeout_timer_arg
    };
    ESP_ERROR_CHECK(esp_timer_create(&echo_timeout_timer_args, echo_timeout_timer_left_handle));
    ESP_ERROR_CHECK(esp_timer_create(&echo_timeout_timer_args, echo_timeout_timer_right_handle));


}

