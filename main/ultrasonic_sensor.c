#include "ultrasonic_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sys/time.h"

#include "esp_check.h"

#define TRIGGER_LOW_DELAY 4
#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT 1000000//3000000 // 3s

// Calc: 331.5+0.61*temperature[m/sec]
// TODO change with temp sensor -> humidity
#define ROUNDTRIP 58 // 20deg

typedef struct 
{
    esp_err_t error;
    const ultrasonic_sensor_t* ultrasonic_sensor;
    uint32_t* distance;
} measure_param;

//static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//#define timeout_expired(start, len) ((uint32_t)get_time_us() - start) >= len
//#define RETURN_CRITICAL(MUX, RES, STORE) do { portEXIT_CRITICAL(&MUX); STORE->error = RES; vTaskDelete(NULL); } while (0)


QueueHandle_t queue_handle;

// Gets current time []
static inline uint32_t get_time_us()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_usec;
}

static void IRAM_ATTR intr_handler_left(void* args)
{   
    event_t event = { gpio_get_level(*(gpio_num_t*)args), LEFT_SIDE };
    xQueueSendFromISR(queue_handle, &event, NULL);
}

static void IRAM_ATTR intr_handler_right(void* args)
{
    event_t event = { gpio_get_level(*(gpio_num_t*)args), RIGHT_SIDE };
    xQueueSendFromISR(queue_handle, &event, NULL);
}

void interrupt_init(const ultrasonic_sensor_t* sensor)
{
    ESP_LOGI("intr_init", "START");
    // Interrupt is triggerd on rising or falling edge
    gpio_set_intr_type(sensor->echo_left, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(sensor->echo_right, GPIO_INTR_ANYEDGE);

    // gpio_pulldown_en(sensor->echo_left);
    // gpio_pullup_dis(sensor->echo_left);

    // gpio_pulldown_en(sensor->echo_right);
    // gpio_pullup_dis(sensor->echo_right);

    // Allocate resources
    gpio_install_isr_service(0);

    // Add isr handlers
    gpio_isr_handler_add(sensor->echo_left, intr_handler_left, (void*)&sensor->echo_left);
    gpio_isr_handler_add(sensor->echo_right, intr_handler_right, (void*)&sensor->echo_right); // change to right

    //interrupt_disable(sensor);
    ESP_LOGI("intr_init", "END");

    queue_handle = xQueueCreate(5, sizeof(event_t));
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


void create_timer(esp_timer_handle_t* timer_handler, esp_timer_cb_t callback, char* name, void* callback_arg)
{
    // Setup timer
    const esp_timer_create_args_t timer_args = 
    {
        .callback = *callback,
        .name = name,
        .arg = callback_arg
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, timer_handler));
}




void timer_callback(void* args)
{
    xQueueSendToFrontFromISR(queue_handle, (event_t*)args, NULL);
}

esp_err_t measure(esp_timer_handle_t* ping_timer, uint32_t* distance_left, uint32_t* distance_right)
{
    event_t event;

    // Init values to 0
    uint32_t time_left_start = 0, time_left_end = 0;
    uint32_t time_right_start = 0, time_right_end = 0;

    // Start timeout timer
    esp_timer_start_once(*ping_timer, PING_TIMEOUT);

    ESP_LOGI("measure", "start");
    while(1)
    {
        // Check if event was added to queue
        if (xQueueReceive(queue_handle, &event, 0))
        {
            ESP_LOGI("measure", "xQueueReceive");
            if (event.event_code == 1)
            {
                ESP_LOGI("xQueueReceive", "EVENT_ULTRASONIC_SENSOR_ECHO_START");
                uint32_t time_start = get_time_us();
                if (event.side == LEFT_SIDE)
                    time_left_start = time_start;
                else
                    time_right_start = time_start;
            }
            else if (event.event_code == 0)
            {
                ESP_LOGI("xQueueReceive", "EVENT_ULTRASONIC_SENSOR_ECHO_END");
                uint32_t time_end = get_time_us();
                if (event.side == LEFT_SIDE)
                    time_left_end = time_end;
                else
                    time_right_end = time_end;


                if (time_left_end != 0 && time_right_end != 0 && time_left_start != 0 && time_right_end != 0)
                {
                    esp_timer_stop(*ping_timer);

                    break;
                } 
            }
            else if(event.event_code == 2)
            {
                // timeout
                ESP_LOGI("xQueueReceive", "EVENT_ULTRASONIC_SENSOR_PING_TIMEOUT");
                return ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT;
            }
        }
    }
    
    *distance_left = (time_left_end - time_left_start) / ROUNDTRIP;
    *distance_right = (time_right_end - time_right_start) / ROUNDTRIP;
    printf("time left: start -> %"PRIu32" end -> %"PRIu32"\n", time_left_start, time_left_end);
    printf("time right: start -> %"PRIu32" end -> %"PRIu32"\n", time_right_start, time_right_end);
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
}

esp_err_t trigger(const ultrasonic_sensor_t* sensor)
{
    // Triggers of ultrasonic sensor

    gpio_set_level(sensor->trigger, 0);
    esp_rom_delay_us(TRIGGER_LOW_DELAY);
    gpio_set_level(sensor->trigger, 1);               
    esp_rom_delay_us(TRIGGER_HIGH_DELAY);
    gpio_set_level(sensor->trigger, 0);

    // maybe need to add critical

    return ESP_OK;
}


// esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t* sensor, uint32_t* distance_left, uint32_t* distance_right)
// {
//     if (distance_left == NULL || distance_left == distance_right || sensor == NULL)
//         return ESP_ERR_INVALID_ARG;

//     measure_param param_left, param_right;

//     param_left.ultrasonic_sensor = sensor;
//     param_left.distance = distance_left;
    
//     param_right.ultrasonic_sensor = sensor;
//     param_right.distance = distance_right;

//     TaskHandle_t measure_left = NULL;
//     TaskHandle_t measure_right = NULL;

//     xTaskCreate(&ultrasonic_measure_left_cm, "Measure left sensor", 2048, (void*)&param_left, 10, &measure_left);
//     xTaskCreatePinnedToCore(&ultrasonic_measure_right_cm, "Measure lright sensor", 2048, (void*)&param_right, 10, &measure_right, 1);

//     // temp;
//     return param_left.error;
// }