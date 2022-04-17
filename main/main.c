/* @file
 * Daisy Dimmer code.
 *
 * Derived from ESP8266_RTOS_SDK example
 * examples/peripherals/gpio/main/user_main.c, which is licensed as Public
 * Domain.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "main";

/*
 * GPIO Layout:
 *
 * GPIO4 (D2):  output
 * GPIO5 (D1):  input, pulled down, interrupt from rising edge and falling edge
 *
 */

#define GPIO_OUTPUT    GPIO_NUM_4

#define GPIO_INPUT     GPIO_NUM_5

#define GPIO_TASK_PRIORITY  ((UBaseType_t)(configMAX_PRIORITIES - 1))

static xQueueHandle gpio_queue = NULL;

static void gpio_task(void *arg);

/**
 * ISR handler - submit that we got ann edge to the queue
 * 
 * @param arg ISR argument. Ignored.
 */
static void gpio_isr_handler(void *arg)
{
    uint8_t edge = 1;

    // post that we got an edge. We just post a 1 here which is just a
    // placeholder for "edge".
    xQueueSendFromISR(gpio_queue, &edge, NULL);
}

/**
 * Main - configure everything and start our GPIO task.
 */
void app_main(void)
{
    // output GPIO config
    // - no interrupts
    // - pin 4 only
    // - input
    // - no pull down
    // - no pull up
    gpio_config_t output_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = GPIO_Pin_4,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&output_conf));

    // input GPIO config
    // - interrupt on any edge
    // - pin 5 only
    // - input only
    // - pull down
    // - no pull up
    gpio_config_t input_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = GPIO_Pin_5,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&input_conf));

    // create a queue to handle events posted from the ISR
    gpio_queue = xQueueCreate(10, sizeof(uint8_t));

    if (gpio_queue == NULL) {
        ESP_LOGE(TAG, "Error creating queue. We're screwed.");
        esp_restart();
    }

    // start gpio task
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, GPIO_TASK_PRIORITY, NULL);

    // register the GPIO handler
    gpio_isr_register(gpio_isr_handler, NULL, 0, NULL);

    // At this point, we're done, and gpio_task takes over.
}

/**
 * GPIO monitoring task
 * 
 * Wait for an edge to be received from our queue. If so, enable the output. If
 * we time out after 1 second, then turn off our output.
 * 
 * @param arg Argument, unused.
 * 
 */
static void gpio_task(void *arg)
{
    uint8_t edge;

    ESP_LOGI(TAG, "Monitoring for PWM input.");

    while(1) {
        if (xQueueReceive(gpio_queue, &edge, 1000 / portTICK_PERIOD_MS)) {
            // received an edge
            ESP_LOGI(TAG,
                     "Got an edge (level is %d) - turning on output.",
                     gpio_get_level(GPIO_INPUT));

            // we got some wiggles, set output high to dim the radio
            gpio_set_level(GPIO_OUTPUT, 1);
        }
        else {
            // timeout - no wiggles
            ESP_LOGI(TAG, "Timeout waiting for edge.");

            if (gpio_get_level(GPIO_INPUT) == 0) {
                ESP_LOGI(TAG,
                         "No edges and input is low - turning off output.");

                // No PWM and the signal is 0, which means the lights are off -
                // turn the radio dimmer signal off too.
                gpio_set_level(GPIO_OUTPUT, 0);
            }
            else {
                ESP_LOGI(TAG,
                         "No edges and input is high - turning off output.");

                // No PWM and the signal is 1, which means the lights are on but
                // the console is at full brightness, so turn the radio dimmer
                // on.
                gpio_set_level(GPIO_OUTPUT, 1);
            }
        }
    }
}
