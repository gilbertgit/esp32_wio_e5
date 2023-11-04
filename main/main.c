#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "string.h"
#include "esp_log.h"
#include "bme280.h"

#define LED_PIN 13
#define TXD_PIN 17
#define RXD_PIN 16

static uint8_t s_led_state = 0;
static const int RX_BUF_SIZE = 512;

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(LED_PIN, s_led_state);
}

static void configure_led(void)
{
    // ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(LED_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t bme280_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, bme280_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// static esp_err_t bme280_register_write_byte(uint8_t reg_addr, uint8_t data)
// {
//     int ret;
//     uint8_t write_buf[2] = {reg_addr, data};

//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, bme280_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

//     return ret;
// }

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static int atSendAndCheckResp(char *p_ack, int timeout_ms, char *p_cmd)
{
    int startMillis = 0;
    static const char *RX_TX_TASK_TAG = "atSendAndCheckResp";
    esp_log_level_set(RX_TX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    
    sendData(RX_TX_TASK_TAG, p_cmd);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    startMillis = xTaskGetTickCount();

    while (xTaskGetTickCount() - startMillis < timeout_ms) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TX_TASK_TAG,"Read %d bytes: '%s'\n", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }

        if (strcmp((const char *)data, p_ack) != 0)
        {
            // success
            free(data);
            return 1;
        }
    }

    free(data);
    return 0;
}

void hello_world_lora_task(void *arg)
{
    while (1) {
        atSendAndCheckResp("+TEST: TXLRSTR \"HELLO WORLD\"", 1000, "AT+TEST=TXLRSTR,\"HELLO WORLD\"");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void bme280_task(void *args) {

    static const char *BME280_TASK_TAG = "bme280_task";
    esp_log_level_set(BME280_TASK_TAG, ESP_LOG_INFO);
    int32_t raw_temperature;
    int32_t raw_pressure;

    struct bme280_calib_param bme280_params;
    bme280_get_calib_params(&bme280_params);

    while (1) {
        bme280_read_raw(&raw_temperature, &raw_pressure);
        int32_t temperature = bme280_convert_temp(raw_temperature, &bme280_params);
        int32_t pressure = bme280_convert_pressure(raw_pressure, raw_temperature, &bme280_params);
        ESP_LOGI(BME280_TASK_TAG, "Pressure = %.3f kPa\n", pressure / 1000.f);
        float temp_cel = temperature / 100.f;
        float temp_fahr = (temp_cel * 9.f / 5.f) + 32.f;
        // printf("Temp = %.2f C\n", temp_cel);
        ESP_LOGI(BME280_TASK_TAG, "Temp = %.2f F\n", temp_fahr);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    static const char *INIT_TAG = "INIT";
    configure_led();
    init_uart();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(INIT_TAG, "I2C initialized successfully");

    uint8_t data[2];
    ESP_ERROR_CHECK(bme280_register_read(0x73, data, 1));
    ESP_LOGI(INIT_TAG, "WHO_AM_I = %X", data[0]);

    bme280_init();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    atSendAndCheckResp("+AT: OK", 100, "AT\r\n");
    atSendAndCheckResp("+MODE: TEST", 1000, "AT+MODE=TEST\r\n");
    atSendAndCheckResp("+TEST=RFCFG F:915000000, SF12, 125, 12, 15, 14, ON, OFF, OFF", 1000, "AT+TEST=RFCFG,915,SF12,125,12,15,14,ON,OFF,OFF\r\n");

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    xTaskCreate(hello_world_lora_task, "hello_world_lora_task", 1024 * 2, NULL, configMAX_PRIORITIES -1, NULL);
    xTaskCreate(bme280_task, "bme280_task", 1024 * 2, NULL, configMAX_PRIORITIES -1, NULL);

    int i = 0;
    while (1) {
        blink_led();
        i++;
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
