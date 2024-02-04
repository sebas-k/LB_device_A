//Includes
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "driver/rmt_types.h"
#include "driver/rmt_tx.h"

//Pin configuration
#define Red_LED GPIO_NUM_46 //Red LED, ON = LOW
#define Green_LED GPIO_NUM_0 //Green LED, ON = LOW
#define Blue_LED GPIO_NUM_45 //Blue LED, ON = LOW
#define Yellow_LED GPIO_NUM_48 //Yellow LED, OFF = High
#define DSHOT_ESC_GPIO_NUM GPIO_NUM_21 //D10 (Arduino Nano ESP32 Layout)
#define Test_button GPIO_NUM_18 //D9 (Arduino Nano ESP32 Layout)

//Pin select
#define LED_OUTPUT_PIN_SEL  ((1ULL<<Green_LED) | (1ULL<<Blue_LED) | (1ULL<<Red_LED) | (1ULL<<Yellow_LED))
#define GPIO_INTERRUPT_PIN_SEL  ((1ULL<<Test_button))
#define ESP_INTR_FLAG_DEFAULT 0

//Global
static const char *TAG = "LB_device_A";
volatile bool IR_test_break = false;

static const rmt_symbol_word_t ir_signal ={
        .level0 = 0,
        .duration0 = 360, //360
        .level1 = 1,
        .duration1 = 360 //360
    };

void GPIO_config (void)
{
gpio_config_t led_io_conf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = LED_OUTPUT_PIN_SEL,
    .pull_down_en = 0,
    .pull_up_en = 0
    };
    gpio_config(&led_io_conf);

    gpio_set_level (Red_LED, 1); //OFF
    gpio_set_level (Green_LED, 1); //OFF
    gpio_set_level (Blue_LED, 1); //OFF
    gpio_set_level (Yellow_LED, 1); //ON
    ESP_LOGI(TAG, "GPIO LED config - done");
}

void GPIO_interrupt_config (void){
    gpio_config_t inter_io_conf = {
    .intr_type = GPIO_INTR_NEGEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = GPIO_INTERRUPT_PIN_SEL,
    .pull_down_en = 0,
    .pull_up_en = 0
    };
    gpio_config(&inter_io_conf);
    ESP_LOGI(TAG, "GPIO interrupt config - done");
}

static void IRAM_ATTR Test_button_interrupt_isr_handler(void* arg)
{
IR_test_break = true;
}

void app_main(void)
{
GPIO_config();
GPIO_interrupt_config();
gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //Install interrupt isr service routinne
gpio_isr_handler_add(Test_button, Test_button_interrupt_isr_handler, (void*) Test_button); //Hook isr handler for Test_button gpio pin

ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1 * 1000 * 1000,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = DSHOT_ESC_GPIO_NUM,
    };
    rmt_channel_handle_t tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));
ESP_LOGI(TAG, "Create RMT TX channel - done");

ESP_LOGI(TAG, "Modulate carrier to TX channel");
    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.5,
        .frequency_hz = 56000, // 56KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
ESP_LOGI(TAG, "Modulate carrier to TX channel - done");

ESP_LOGI(TAG, "Set channel to invinity");
    rmt_transmit_config_t transmit_config = {
        .loop_count = -1, // invinity
    };
ESP_LOGI(TAG, "Set channel to invinity - done");

ESP_LOGI(TAG, "Set encoder and start");
    rmt_copy_encoder_config_t config_encoder = {};
    rmt_encoder_handle_t encoder = NULL;
    rmt_new_copy_encoder(&config_encoder, &encoder);
    rmt_transmit(tx_channel, encoder, &ir_signal, sizeof(ir_signal), &transmit_config);
ESP_LOGI(TAG, "Set encoder and start - done");

while (1) {
//for ( int cnt = 1; cnt <= 10; cnt++) {
    if (IR_test_break) {
        ESP_LOGI(TAG, "Test break - start");
        ESP_ERROR_CHECK(rmt_disable(tx_channel));
        vTaskDelay(500 / portTICK_PERIOD_MS); 
        ESP_ERROR_CHECK(rmt_enable(tx_channel));
        rmt_transmit(tx_channel, encoder, &ir_signal, sizeof(ir_signal), &transmit_config);
        IR_test_break = false;
        ESP_LOGI(TAG, "Test break - ends");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);  
}
}