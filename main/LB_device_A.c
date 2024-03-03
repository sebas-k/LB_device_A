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
#include "driver/uart.h"
#include "esp_system.h"
#include "string.h"

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

//Device Role status
#define DEVICE_NOT_INITALIZED 0
#define DEVICE_IS_TIMER_SYNC_MASTER 1
#define DEVICE_IS_NOT_TIMER_SYNC_MASTER 2
#define DEVICE_IS_IR_RECEIVER 3

//Pin select
#define LED_OUTPUT_PIN_SEL  ((1ULL<<Green_LED) | (1ULL<<Blue_LED) | (1ULL<<Red_LED) | (1ULL<<Yellow_LED) | (1ULL<<LOW_IR_LED))
#define GPIO_INPUT_PIN_SEL  ((1ULL<<Test_button))

//UART
static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_43) //TX (Arduino Nano ESP32 Layout)
#define RXD_PIN (GPIO_NUM_44) //RX (Arduino Nano ESP32 Layout)

//Meta data
#define SYNC_DEVICES 0
#define TIME_MEASURE 1

//LED Indicator
#define RGB_on 0
#define RGB_off 1
volatile bool one_shot = 0;
volatile bool LED_Indicator_Arr [6] = {1,1,1,1,1,1};

//Global Variables
static const char *TAG = "LB_device_A";
uint64_t sync_seconds = 5;
volatile char device_role = 0;


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

void GPIO_input_config (void){
    gpio_config_t input_io_conf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = GPIO_INPUT_PIN_SEL,
    .pull_down_en = 0,
    .pull_up_en = 0
    };
    gpio_config(&input_io_conf);
    ESP_LOGI(TAG, "GPIO input config - done");
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

void sendData_uart(const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(TAG, "Wrote %d bytes", txBytes);
}

static bool IRAM_ATTR timer_5s_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
BaseType_t high_task_awoken = pdFALSE;
myPayLoad.meta_data = SYNC_DEVICES;
myPayLoad.intVal_ms = sync_seconds;
esp_now_send(broadcastAddress, (uint8_t *) &myPayLoad, sizeof(myPayLoad));
sync_seconds = sync_seconds + 5;
return (high_task_awoken == pdTRUE);// return whether we need to yield at the end of ISR
}

static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
ESP_LOGI(TAG, "Sync sending - done");
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    char text[20];
    memcpy(&myPayLoad, data, sizeof(myPayLoad));

    switch(myPayLoad.meta_data){
        case TIME_MEASURE:
        sprintf(text, "%i", myPayLoad.intVal_ms); //rework 
        strcat(text, "\n"); //rework
        sendData_uart (text); //rework
        ESP_LOGI(TAG, "MS since started the Timer %i", myPayLoad.intVal_ms);
        break;
        case SYNC_DEVICES:
        device_role = DEVICE_IS_NOT_TIMER_SYNC_MASTER;
        break;
    }
}

void timer_1ms_config (){
//Config Timer
gptimer_handle_t gptimer_5s = NULL;
gptimer_config_t timer_config = { 
    .clk_src = GPTIMER_CLK_SRC_XTAL,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000,};
ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_5s));
gptimer_event_callbacks_t cbs = {
    .on_alarm = timer_5s_on_alarm, };
ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_5s, &cbs, NULL)); // Check if it is allowed to put the NULL pointer as parameter
ESP_ERROR_CHECK(gptimer_enable(gptimer_5s));
gptimer_alarm_config_t alarm_config_5s = { 
    .reload_count = 0,
    .alarm_count = 5000000, 
    .flags.auto_reload_on_alarm = true,};   
ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_5s, &alarm_config_5s));
ESP_ERROR_CHECK(gptimer_start(gptimer_5s));
}

void ESPnow_config (){
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
}

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Buffer not used for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void uart_rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            switch (data[0]){
                case 1: 
                ESP_LOGI(RX_TASK_TAG, "Start as Timer Sync Master"); 
                device_role = DEVICE_IS_TIMER_SYNC_MASTER;
                break;
            }
        }
    }
    free(data);
}

void Button_Task(void *params){
    
bool button_activated = 1;
while (1){
if (gpio_get_level(Test_button)){ 
    if (button_activated) {
        button_activated = 0;
        gpio_set_level (LOW_IR_LED, 1); //IR LED deactivated
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level (LOW_IR_LED, 0); //IR LED activated
    }
} else {

    button_activated = 1;
}
vTaskDelay(100 / portTICK_PERIOD_MS);
}
}

void LED_Indicator_Task(void *params) {
char count = 0;

while (1){

if (one_shot){
    gpio_set_level (Red_LED, 0); 
    gpio_set_level (Green_LED, 1); 
    gpio_set_level (Blue_LED, 1);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    count = 11;
    one_shot = 0;
}

switch (count) {
    case 1:
    gpio_set_level (Red_LED, LED_Indicator_Arr[0]); 
    gpio_set_level (Green_LED, LED_Indicator_Arr[1]); 
    gpio_set_level (Blue_LED, LED_Indicator_Arr[2]);
    break;
    case 11:
    gpio_set_level (Red_LED, LED_Indicator_Arr[3]); 
    gpio_set_level (Green_LED, LED_Indicator_Arr[4]); 
    gpio_set_level (Blue_LED, LED_Indicator_Arr[5]); 
    break;
    case 20:
    count = 0;
    break;
}
vTaskDelay(100 / portTICK_PERIOD_MS);
count++;
}
}

void app_main(void)
{

TaskHandle_t task_handle = NULL;

GPIO_config();
GPIO_input_config();
RMT_IR_Signal();
ESPnow_config();
uart_init();

LED_Indicator_Arr [0] = RGB_off; LED_Indicator_Arr [1] = RGB_off; LED_Indicator_Arr [2] = RGB_on;
LED_Indicator_Arr [3] = RGB_off; LED_Indicator_Arr [4] = RGB_off; LED_Indicator_Arr [5] = RGB_off;
xTaskCreatePinnedToCore(LED_Indicator_Task, "LED_Indicator_Task", 2048, NULL, 1, NULL, 0);

xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 1024 * 2, NULL, 2, &task_handle, 0);
vTaskDelay(5500 / portTICK_PERIOD_MS); //Wait till a sync signal is received

while (device_role == DEVICE_NOT_INITALIZED) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

vTaskDelete(task_handle);

switch (device_role)
{
case DEVICE_IS_TIMER_SYNC_MASTER:
    timer_1ms_config();
    LED_Indicator_Arr [0] = RGB_on; LED_Indicator_Arr [1] = RGB_on; LED_Indicator_Arr [2] = RGB_on;
    LED_Indicator_Arr [3] = RGB_on; LED_Indicator_Arr [4] = RGB_on; LED_Indicator_Arr [5] = RGB_on;
    ESP_LOGI(TAG, "Device is Timer Sync Master");
    break;
case DEVICE_IS_NOT_TIMER_SYNC_MASTER:
    ESP_LOGI(TAG, "Device is NOT Timer Sync Master");
    LED_Indicator_Arr [0] = RGB_off; LED_Indicator_Arr [1] = RGB_on; LED_Indicator_Arr [2] = RGB_off;
    LED_Indicator_Arr [3] = RGB_off; LED_Indicator_Arr [4] = RGB_on; LED_Indicator_Arr [5] = RGB_off;
    break;
}

xTaskCreatePinnedToCore(Button_Task, "Button_Task", 2048, NULL, 1, NULL, 0);

}