#include "temperature_sensor.h"

#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "rom/ets_sys.h"

// All values are defined in microseconds [us]
#define REQUEST_LOW_PULSE_DELAY     3000    // 1~10ms -> 1000~10000us  
#define REQUEST_HIGH_PULSE_DELAY    25      // 20~40us   
#define REQUEST_SENSOR_TIME_LOW     85      // >80us      
#define REQUEST_SENSOR_TIME_HIGH    85      // >80us      
#define SEND_START_DELAY            56      // >50us
#define SEND_TIMEOUT_DELAY          75      // >70us     
#define SEND_ONE_TIME               40      // >26~28us

#define SEND_NUM_DATA_BITS      40  // 40 data bits
#define SEND_MAX_DATA_ARRAY     5   // SEND_NUM_DATA_BITS / 8 (uint8_t array)   


static const char* TAG = "Temperature sensor DHT22"; 
static gpio_num_t sensor_gpio_pin;

float temperature;  // [%]
float humidity;     // [C]

esp_err_t init_dht22(gpio_num_t pin)
{
    ESP_RETURN_ON_ERROR(gpio_reset_pin(pin), TAG, "");
    ESP_RETURN_ON_ERROR(gpio_set_direction(pin, GPIO_MODE_INPUT), TAG, "");


    ESP_RETURN_ON_ERROR(gpio_pulldown_dis(pin), TAG, "");
    ESP_RETURN_ON_ERROR(gpio_pullup_dis(pin), TAG, "");

    sensor_gpio_pin = pin;

    temperature = 0.0f;
    humidity = 0.0f;

    return ESP_OK;
}


float get_humidity()
{
    return humidity;
}

float get_temperature()
{
    return temperature;
}


// Probably needs improvement (interrupts?)
int get_signal_level_time(int time_out_us, bool state)
{
    int sec = 0;

    while (gpio_get_level(sensor_gpio_pin) == state)
    {
        if (sec > time_out_us)
            return -1;
            
        ++sec;
        ets_delay_us(1);
    }

    return sec;
}

/*----------------------------------------------------------------------------
;
;	read DHT22 sensor
copy/paste from AM2302/DHT22 Docu:
DATA: Hum = 16 bits, Temp = 16 Bits, check-sum = 8 Bits
Example: MCU has received 40 bits data from AM2302 as
0000 0010 1000 1100 0000 0001 0101 1111 1110 1110
16 bits RH data + 16 bits T data + check sum
1) we convert 16 bits RH data from binary system to decimal system, 0000 0010 1000 1100 → 652
Binary system Decimal system: RH=652/10=65.2%RH
2) we convert 16 bits T data from binary system to decimal system, 0000 0001 0101 1111 → 351
Binary system Decimal system: T=351/10=35.1°C
When highest bit of temperature is 1, it means the temperature is below 0 degree Celsius. 
Example: 1000 0000 0110 0101, T= minus 10.1°C: 16 bits T data
3) Check Sum=0000 0010+1000 1100+0000 0001+0101 1111=1110 1110 Check-sum=the last 8 bits of Sum=11101110
Signal & Timings:
The interval of whole process must be beyond 2 seconds.
To request data from DHT:
1) Sent low pulse for > 1~10 ms (MILI SEC)
2) Sent high pulse for > 20~40 us (Micros).
3) When DHT detects the start signal, it will pull low the bus 80us as response signal, 
   then the DHT pulls up 80us for preparation to send data.
4) When DHT is sending data to MCU, every bit's transmission begin with low-voltage-level that last 50us, 
   the following high-voltage-level signal's length decide the bit is "1" or "0".
	0: 26~28 us
	1: 70 us
;----------------------------------------------------------------------------*/

//#define RETURN_ERROR_ON_TIMEOUT(time, timeout_us, state) ({time = get_signal_level_time(timeout_us, state);if (sec < 0){return DHT22_TIMEOUT_ERROR;}})

portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

#include "freertos/portmacro.h"

//critical section

dht22_error read_dht22()
{
    ESP_LOGI(TAG, "start read");
    uint8_t data[SEND_MAX_DATA_ARRAY] = { 0 };
    // Maybe use i in for loop to calculate indices
    uint8_t byteIndex = 0, bitIndex = 7;

    //portDISABLE_INTERRUPTS();
    // taskENTER_CRITICAL(&myMutex);
    ESP_LOGI(TAG, "change dir");
    // Change gpio direction to output
    gpio_set_direction(sensor_gpio_pin, GPIO_MODE_OUTPUT);

ESP_LOGI(TAG, "1");
    // Send signal to DHT22 sensor
    gpio_set_level(sensor_gpio_pin, GPIO_OUTPUT_LOW);
    ets_delay_us(REQUEST_LOW_PULSE_DELAY);

ESP_LOGI(TAG, "2");
    gpio_set_level(sensor_gpio_pin, GPIO_OUTPUT_HIGH);
    ets_delay_us(REQUEST_HIGH_PULSE_DELAY);

ESP_LOGI(TAG, "3");
    // Change gpio direction to input
    gpio_set_direction(sensor_gpio_pin, GPIO_MODE_INPUT);

    // DHT22 keeps line low for 80us and then high for 80us
    // if line doesn't change in that timeframe return error
    ESP_LOGI(TAG, "start info");
    int sec = 0;
    sec = get_signal_level_time(REQUEST_SENSOR_TIME_LOW, GPIO_OUTPUT_LOW);
    if (sec < 0) return DHT22_TIMEOUT_ERROR;
ESP_LOGI(TAG, "4");
    sec = get_signal_level_time(REQUEST_SENSOR_TIME_HIGH, GPIO_OUTPUT_HIGH);
    if (sec < 0) return DHT22_TIMEOUT_ERROR;
ESP_LOGI(TAG, "5");
    // No errors -> DHT22 sends data
    for (int i = 0; i < SEND_NUM_DATA_BITS; ++i)
    {
        // Starts with 50us low voltage
        sec = get_signal_level_time(SEND_START_DELAY, GPIO_OUTPUT_LOW);
        if (sec < 0) return DHT22_TIMEOUT_ERROR;

        // Check if signal is still high after 70us -> 1 else 0
        sec = get_signal_level_time(SEND_TIMEOUT_DELAY, GPIO_OUTPUT_HIGH);
        if (sec < 0) return DHT22_TIMEOUT_ERROR;
        
        // Data bits flows from left to right

        // If sec > 26~28us -> 1 else 0
        if (sec > SEND_ONE_TIME)
        {
            data[byteIndex] |= (1 << bitIndex);
        }

        // If full move to next byte
        if (bitIndex == 0) 
        {
            bitIndex = 7;
            ++byteIndex;
        }
        else --bitIndex;
    }

    // portENABLE_INTERRUPTS();

    // Verify checksum
    // & 0xff is because you need only 8 bits and overflow is ignored
    // all uint8_t types are promoted to int thats why there can be more than 8 bits stored
    ESP_LOGI(TAG, "%d, %d, %d, %d, %d", data[4], data[0], data[1], data[2], data[3]);
    if (data[4] != (data[0] + data[1] + data[2] + data[3]) && 0xff)
        return DHT22_CHECKSUM_ERROR;

    // Set humidity from data[0] and data[1]
    humidity = (float)(((uint16_t)data[0] << 8) | data[1]) / 10.0f; 

    // Set temperature from data[2] and data[3] but ignore first bit in data[2]( & 0x7f set first bit to 0)
    temperature = (float)((((uint16_t)data[2] & 0x7f) << 8) | data[3]) / 10.0f; 

    // Check first bit in temperature for negative value and change sign
    if (data[2] & 0x80)
        temperature *= -1;

    return DHT22_OK;
}