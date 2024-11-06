// esp-idf includes
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_timer.h>
#include "esp_log.h"
// private includes
#include <bb_uart.h>

// Include FreeRTOS for delay
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * Example description:
 * ============================================================================
 * This example demonstrates the ability to receive and send data over
 * UART using a single pin configured for Input and Output.
 * 
 * To demonstrate this ability, the user can send any number between 0 and 9
 * and will receive the number incremented by one back, except in the case
 * of 9 where a 0 is returned.
 * 
 * Wiring diagram for this example
 *                                     +--------------+
 *                                     |              |
 *                                   +-+----+         |
 *                              +----|  TX  |         |
 *                              |    +-+----+         |
 *                             +-+     |              |
 *                          R: | |     |  e.g. CH340  |
 * +-------------+          1k | |     |  or FT232RL  |
 * |             |             +-+     |              |
 * |        +----|-+            |    +-+----+         |
 * |    MCU |in/out|------------+----|  RX  |         |
 * |        +----|-+                 +-+----+         |
 * |             |                     |              |
 * +-------------+                     +--------------+
 * 
 * This example was written using a Seeed Studio XIAO ESP32C3.
 */

/**
 * For the UART to function a timer interrupt is necessary,
 * the frequency of which is calculated by multiplying the desired
 * baudrate with the oversampling setting.
 * In this example, that would be 9600 baud * 3.
 */
#define OVERSAMPLING (BB_UART_OVERSAMPLE_5)
#define TIMER_INTERVAL_US (((float)1000000)/9600/OVERSAMPLING)

// Single-Wire Interface
#define TRANSCEIVE_PIN GPIO_NUM_7

static const char* TAG = "BB-UART_Example";

/**
 * These functions tell the bitbang UART how to interact with this hardware.
 * The pins are hardcoded per function.
 * If you need multiple bitbang UARTs you have to define these functions
 * for each one with different pins.
 */
void bbuart_pinWriteFunc(uint8_t val){
    gpio_set_level(TRANSCEIVE_PIN, val);
}
uint8_t bbuart_pinReadFunc(void){
    return gpio_get_level(TRANSCEIVE_PIN);
}
// Create required ringbuffers
RING_BUFFER_DEF(bbuart_tx_buf, 128);
RING_BUFFER_DEF(bbuart_rx_buf, 128);
// create BitBang UART struct and configure it
BB_UART_t bbuart = {
    // baud rate can not be controlled from the UART library but is instead
    // determined by the frequency of the timer interrupt
    .wordLen = BB_UART_WORDLENGTH_8,
    .parity = BB_UART_PARITY_NONE,
    .stopBits = BB_UART_STOPBITS_1,
    .tx_ringBuf = &bbuart_tx_buf,
    .rx_ringBuf = &bbuart_rx_buf,
    .writePinFunc = bbuart_pinWriteFunc,
    .readPinFunc = bbuart_pinReadFunc,
    .mode = BB_UART_ONE_WIRE,
    .oversampling = OVERSAMPLING
};

static bool IRAM_ATTR bbuart_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx){
    BB_UART_service(&bbuart);
    return false;
}

gptimer_handle_t bbuart_timer_handle = NULL;

void app_main() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    if(RC_SUCCESS != BB_UART_validateConfig(&bbuart)){
        ESP_LOGE(TAG, "Invalid BBUART configuration");
    }
    // configure pin as input/output with open-drain
    gpio_config_t bbuart_pin_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        // !!! GPIO MUST BE CONFIGURED AS OPEN DRAIN
        // otherwise no data reception is possible
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pin_bit_mask = (1ULL << TRANSCEIVE_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&bbuart_pin_cfg);
    // set pin to idle level
    gpio_set_level(TRANSCEIVE_PIN, 1);

    // configure timer for interrupt
    gptimer_config_t bbuart_timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 4000000, // 1 timer tick is equal to 0.25us
    };
    esp_err_t err = gptimer_new_timer(&bbuart_timer_cfg, &bbuart_timer_handle);
    if(err != ESP_OK) ESP_LOGE(TAG, "Timer creation failed, code: %u", err);

    gptimer_event_callbacks_t bbuart_timer_cb_cfg = {
        .on_alarm = bbuart_isr
    };
    err = gptimer_register_event_callbacks(bbuart_timer_handle, &bbuart_timer_cb_cfg, NULL);
    if(err != ESP_OK) ESP_LOGE(TAG, "Timer callback config failed, code: %u", err);

    ESP_LOGI(TAG, "Enabling timer");
    err = gptimer_enable(bbuart_timer_handle);
    if(err != ESP_OK) ESP_LOGE(TAG, "Timer enable failed, code: %u", err);

    gptimer_alarm_config_t bbuart_timer_alarm_cfg = {
        .alarm_count = (TIMER_INTERVAL_US * 4) - 1, // set interval between interrupts
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = 1
        }
    };
    // add interrupt to alarm
    err = gptimer_set_alarm_action(bbuart_timer_handle, &bbuart_timer_alarm_cfg);
    if(err != ESP_OK) ESP_LOGE(TAG, "Timer interrupt config failed, code: %u", err);

    // finally start timer
    err = gptimer_start(bbuart_timer_handle);
    if(err != ESP_OK) ESP_LOGE(TAG, "Timer start failed, code: %u", err);

    ESP_LOGI(TAG, "Starting main loop");
    for(;;){
        uint8_t c;
        while(BB_UART_get(&bbuart, &c, 1) > 0){
            // reply to received data if it is a number with the next higher one.
            // if the received number is 9, send a 0 instead.
            char answer = c;
            if('0' <= c && c <= '8') answer++;
            else if(c == '9') answer = '0';
            BB_UART_putc(&bbuart, answer);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}