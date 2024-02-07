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
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"

//Pin configuration
#define Red_LED GPIO_NUM_46 //Red LED, ON = LOW
#define Green_LED GPIO_NUM_0 //Green LED, ON = LOW
#define Blue_LED GPIO_NUM_45 //Blue LED, ON = LOW
#define Yellow_LED GPIO_NUM_48 //Yellow LED, OFF = High
#define LOW_IR_LED GPIO_NUM_17//D8 (Arduino Nano ESP32 Layout)
#define DSHOT_ESC_GPIO_NUM GPIO_NUM_21 //D10 (Arduino Nano ESP32 Layout)
#define Test_button GPIO_NUM_18 //D9 (Arduino Nano ESP32 Layout)
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#define CONFIG_ESPNOW_CHANNEL 5
static uint8_t broadcastAddress[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

//Pin select
#define LED_OUTPUT_PIN_SEL  ((1ULL<<Green_LED) | (1ULL<<Blue_LED) | (1ULL<<Red_LED) | (1ULL<<Yellow_LED) | (1ULL<<LOW_IR_LED))
#define GPIO_INTERRUPT_PIN_SEL  ((1ULL<<Test_button))
#define ESP_INTR_FLAG_DEFAULT 0

//Meta data
#define SYNC_DEVICES 0
#define TIME_MEASURE 1

//Global
static const char *TAG = "LB_device_A";
volatile bool IR_test_break = false;
uint64_t sync_seconds = 5;
volatile bool state_red_LED = false;

typedef struct PayLoad {
    char meta_data;
    int intVal_ms;
} PayLoad;

PayLoad myPayLoad; 

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
    gpio_set_level (LOW_IR_LED, 0); //ON
    ESP_LOGI(TAG, "GPIO LED config - done");
}

void GPIO_interrupt_config (void){
    gpio_config_t inter_io_conf = {
    .intr_type = GPIO_INTR_ANYEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = GPIO_INTERRUPT_PIN_SEL,
    .pull_down_en = 0,
    .pull_up_en = 0
    };
    gpio_config(&inter_io_conf);
    ESP_LOGI(TAG, "GPIO interrupt config - done");
}

void RMT_IR_Signal (void){
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
}

static void app_wifi_init()
{
ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
}

static void IRAM_ATTR Test_button_interrupt_isr_handler(void* arg)
{
IR_test_break = true;
gpio_set_level (LOW_IR_LED, 1); //OFF
}

static bool IRAM_ATTR timer_1ms_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
BaseType_t high_task_awoken = pdFALSE;
myPayLoad.meta_data = SYNC_DEVICES;
myPayLoad.intVal_ms = sync_seconds;
esp_now_send(broadcastAddress, (uint8_t *) &myPayLoad, sizeof(myPayLoad));
sync_seconds = sync_seconds + 5;
gpio_set_level(Red_LED, state_red_LED);
state_red_LED = !state_red_LED;
return (high_task_awoken == pdTRUE);// return whether we need to yield at the end of ISR
}

static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
ESP_LOGI(TAG, "Sync sending - done");
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    memcpy(&myPayLoad, data, sizeof(myPayLoad));
    if (myPayLoad.meta_data == TIME_MEASURE){
        ESP_LOGI(TAG, "MS since started the Timer %i", myPayLoad.intVal_ms);
    }
}

void app_main(void)
{
GPIO_config();
GPIO_interrupt_config();
gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //Install interrupt isr service routinne
gpio_isr_handler_add(Test_button, Test_button_interrupt_isr_handler, (void*) Test_button); //Hook isr handler for Test_button gpio pin
RMT_IR_Signal();

//Config Timer
gptimer_handle_t gptimer_1ms = NULL;
gptimer_config_t timer_config = { 
    .clk_src = GPTIMER_CLK_SRC_XTAL,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000,};
ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_1ms));
gptimer_event_callbacks_t cbs = {
    .on_alarm = timer_1ms_on_alarm, };
ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_1ms, &cbs, NULL)); // Check if it is allowed to put the NULL pointer as parameter
ESP_ERROR_CHECK(gptimer_enable(gptimer_1ms));
gptimer_alarm_config_t alarm_config_1ms = { 
    .reload_count = 0,
    .alarm_count = 5000000, 
    .flags.auto_reload_on_alarm = true,};   
ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_1ms, &alarm_config_1ms));
ESP_ERROR_CHECK(gptimer_start(gptimer_1ms));

//Config ESP_Now
ESP_LOGI(TAG, "Wifi Config");
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK( nvs_flash_erase() );
    ret = nvs_flash_init();}
ESP_ERROR_CHECK( ret );
app_wifi_init();
esp_now_init();
ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
esp_now_peer_info_t peerInfo = {};
memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
if(!esp_now_is_peer_exist(broadcastAddress))
{
    esp_now_add_peer(&peerInfo);}

// Main Loop
while (1) {
    if (IR_test_break) {
        vTaskDelay(10 / portTICK_PERIOD_MS); 
        gpio_set_level (LOW_IR_LED, 0); //ON
        IR_test_break = false;
        ESP_LOGI(TAG, "Test break - done");
    } 
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
}
}