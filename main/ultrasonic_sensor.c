#include "ultrasonic_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sys/time.h"
#include "esp_timer.h"
#include "esp_check.h"

#define TRIGGER_LOW_DELAY 4
#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT 6000

// Calc: 331.5+0.61*temperature[m/sec]
// TODO change with temp sensor -> humidity
#define ROUNDTRIP 58 // 20deg

typedef struct 
{
    esp_err_t error;
    const ultrasonic_sensor_t* ultrasonic_sensor;
    uint32_t* distance;
} measure_param;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define timeout_expired(start, len) ((uint32_t)get_time_us() - start) >= len
#define RETURN_CRITICAL(MUX, RES, STORE) do { portEXIT_CRITICAL(&MUX); STORE->error = RES; vTaskDelete(NULL); } while (0)


// Gets current time []
static inline uint32_t get_time_us()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_usec;
}

typedef struct 
{
    uint32_t time_start, time_end;
    esp_err_t error;
} time_data_t;

typedef struct 
{ 
    gpio_num_t echo_pin;
    QueueHandle_t* queue;
    esp_timer_handle_t* timer;
} isr_arg_t;




void interrupt_init(const ultrasonic_sensor_t* sensor)
{
    // Interrupt is triggerd on rising or falling edge
    gpio_set_intr_type(sensor->echo_left, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(sensor->echo_right, GPIO_INTR_ANYEDGE);

    // Allocate resources
    gpio_install_isr_service(0);

    /*
        Args:
            -> pin
            -> queue
            -> timer
    */
    isr_arg_t isr_arg_left;
    // Add isr handlers
    gpio_isr_handler_add(sensor->echo_left, intr_handler_left, (void*)&isr_arg_left);
    gpio_isr_handler_add(sensor->echo_right, intr_handler_left, (void*)PARAMS); // change to right

    interrupt_disable(sensor);
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


void create_timer(esp_timer_handle_t* timer_handler, esp_timer_cb_t* callback, char* name)
{
    // Setup timer
    const esp_timer_create_args_t timer_args = 
    {
        .callback = callback,
        .name = name
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, timer_handler));
}





static void IRAM_ATTR intr_handler_left(void* args)
{   
    // Cast to arg
    isr_arg_t* isr_arg = (isr_arg_t*)args;

    // check if its RISING or FALLING -> send to xQueue (start or end time)

    // Check high
    if (gpio_get_level(isr_arg->echo_pin))
    {
        // Stop timer
        esp_timer_stop(isr_arg->timer);

        // Get passed time from timer
        uint64_t time;
        esp_err_t error = esp_timer_get_expiry_time(isr_arg->timer, &time); 

        time_data_t time_data = { time, 0, error};

        // Send data to queue
        xQueueSendFromISR(*(isr_arg->queue), &time_data, NULL);  
    }
    else
    {
        xQueueSendFromISR(isr_arg->queue, ITEM_TO_QUEUE, NULL);
    }   
}


void temp(QueueHandle_t* queue, esp_timer_handle_t* meas_timer)
{
    time_data_t time_data_left;

    while(1)
    {
        // Check if data was added to queue
        if (xQueueReceive(queue, &time_data_left, 0))
        {
            // Check if data was 0
            if (time_data_left == 0)

            // Prepare meas timer
            uint64_t meas_timeout = MAX_DISTANCE + ROUNDTRIP;
            

            // Start meas timer
            esp_timer_start_once(*meas_timer, meas_timeout);

            break;
        }
    }
    
    // Measure echo reply
    while(1)
    {
        // Check if data was added to queue
        if (xQueueReceive(queue, &time_data_left, 0))
        {
            break;
        }
    }
}

void timer_callback_left(void* args)
{
    timer_report_t* timer_report = (timer_report_t*)args;
    timer_report->error = 
} 

void timer_callback_meas_left(void* args)
{

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
}

void ultrasonic_measure_left_cm(void* parameter)
{
    ESP_LOGI("ultrasonic_measure_left_cm", "start");
    measure_param* param = (measure_param*)parameter;

    ESP_LOGI("ultrasonic_measure_left_cm", "portENTER_CRITICAL");
    portENTER_CRITICAL(&mux);

    ESP_LOGI("ultrasonic_measure_left_cm", "trigger");
    // Sets of ultrasonic sensor 
    gpio_set_level(param->ultrasonic_sensor->trigger, 0);
    esp_rom_delay_us(TRIGGER_LOW_DELAY);
    gpio_set_level(param->ultrasonic_sensor->trigger, 1);
    esp_rom_delay_us(TRIGGER_HIGH_DELAY);
    gpio_set_level(param->ultrasonic_sensor->trigger, 0);

    ESP_LOGI("ultrasonic_measure_left_cm", "check_ping");
    // If previous ping hasn't ended
    if (gpio_get_level(param->ultrasonic_sensor->echo_left))
        RETURN_CRITICAL(mux, ESP_ERR_ULTRASONIC_SENSOR_PING, param);

    	
    // Wait for echo reply from single reciever
    ESP_LOGI("ultrasonic_measure_left_cm", "wait echo reply");
    uint32_t start = get_time_us();
    while (!gpio_get_level(param->ultrasonic_sensor->echo_left))
    {
        if (timeout_expired(start, PING_TIMEOUT))
            RETURN_CRITICAL(mux, ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT, param);
    }

    // Echo recieved continue
    ESP_LOGI("ultrasonic_measure_left_cm", "echo measure reply");
    uint32_t echo_start = get_time_us();
    uint32_t time = echo_start;
    uint32_t meas_timeout = echo_start + MAX_DISTANCE + ROUNDTRIP;
    while (gpio_get_level(param->ultrasonic_sensor->echo_left))
    {
        time = get_time_us();
        if (timeout_expired(start, meas_timeout))
            RETURN_CRITICAL(mux, ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT, param);
    }

    portEXIT_CRITICAL(&mux);

    *(param->distance) = (time - echo_start) / ROUNDTRIP;

    // End task
    param->error = ESP_OK;
    vTaskDelete(NULL);
}

void ultrasonic_measure_right_cm(void* parameter)
{
    measure_param* param = (measure_param*)parameter;

    portENTER_CRITICAL(&mux);

    // Sets of ultrasonic sensor 
    gpio_set_level(param->ultrasonic_sensor->trigger, 0);
    esp_rom_delay_us(TRIGGER_LOW_DELAY);
    gpio_set_level(param->ultrasonic_sensor->trigger, 1);
    esp_rom_delay_us(TRIGGER_HIGH_DELAY);
    gpio_set_level(param->ultrasonic_sensor->trigger, 0);

    // If previous ping hasn't ended
    if (gpio_get_level(param->ultrasonic_sensor->echo_right))
        RETURN_CRITICAL(mux, ESP_ERR_ULTRASONIC_SENSOR_PING, param);

    	
    // Wait for echo reply from single reciever
    uint32_t start = get_time_us();
    while (!gpio_get_level(param->ultrasonic_sensor->echo_right))
    {
        if (timeout_expired(start, PING_TIMEOUT))
            RETURN_CRITICAL(mux, ESP_ERR_ULTRASONIC_SENSOR_PING_TIMEOUT, param);
    }

    // Echo recieved continue
    uint32_t echo_start = get_time_us();
    uint32_t time = echo_start;
    uint32_t meas_timeout = echo_start + MAX_DISTANCE + ROUNDTRIP;
    while (gpio_get_level(param->ultrasonic_sensor->echo_right))
    {
        time = get_time_us();
        if (timeout_expired(start, meas_timeout))
            RETURN_CRITICAL(mux, ESP_ERR_ULTRASONIC_SENSOR_ECHO_TIMEOUT, param);
    }

    portEXIT_CRITICAL(&mux);

    *(param->distance) = (time - echo_start) / ROUNDTRIP;

    // End task
    param->error = ESP_OK;
    vTaskDelete(NULL);
}


esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t* sensor, uint32_t* distance_left, uint32_t* distance_right)
{
    if (distance_left == NULL || distance_left == distance_right || sensor == NULL)
        return ESP_ERR_INVALID_ARG;

    measure_param param_left, param_right;

    param_left.ultrasonic_sensor = sensor;
    param_left.distance = distance_left;
    
    param_right.ultrasonic_sensor = sensor;
    param_right.distance = distance_right;

    TaskHandle_t measure_left = NULL;
    TaskHandle_t measure_right = NULL;

    xTaskCreate(&ultrasonic_measure_left_cm, "Measure left sensor", 2048, (void*)&param_left, 10, &measure_left);
    xTaskCreatePinnedToCore(&ultrasonic_measure_right_cm, "Measure lright sensor", 2048, (void*)&param_right, 10, &measure_right, 1);

    // temp;
    return param_left.error;
}