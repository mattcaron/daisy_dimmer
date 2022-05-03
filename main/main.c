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

// How many edges we need to get in NOISE_DETECT_TIME_MS to turn on the output.
#define NOISE_EDGE_COUNT 5

// We need to get NOISE_EDGE_COUNT rising edges within this time period to turn
// on the output. Otherwise, we discard event and go back to sleep.
#define NOISE_DETECT_TIME_MS 10

// Time after which we turn the output off if we see no pulses
#define INACTIVITY_TIME_MS 1000

static xQueueHandle gpio_queue = NULL;

static void gpio_task(void *arg);

/**
 * ISR handler - submit that we got an edge to the queue
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
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_conf));

    // input GPIO config
    // - interrupt on rising edge
    // - pin 5 only
    // - input only
    // - pull down
    // - no pull up
    gpio_config_t input_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = GPIO_Pin_5,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
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

#if 0
    // register the GPIO handler
    ESP_ERROR_CHECK(gpio_isr_register(gpio_isr_handler, NULL, 0, NULL));
#else
    // alternative version of above.

    // The above doesn't seem to work. This does. Not sure why, but this works,
    // so we're using it.

    // install gpio ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // add the ISR handler for our input pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_INPUT, gpio_isr_handler, NULL));
#endif
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
    TickType_t first_edge_time_ticks = 0;
    uint8_t edge_count = 0;
    bool stay_on = false;

    ESP_LOGI(TAG, "Monitoring for PWM input.");

    while(1) {
        if (xQueueReceive(gpio_queue,
                          &edge,
                          INACTIVITY_TIME_MS / portTICK_PERIOD_MS)) {

            // received an edge
            ESP_LOGI(TAG, "Got an edge.");

            if (!stay_on) {
                // record our first edge
                if (first_edge_time_ticks == 0) {
                    first_edge_time_ticks = xTaskGetTickCount();
                    edge_count = 1;
                }
                else if (edge_count >= NOISE_EDGE_COUNT) {
                    // okay, we've counted enough edges - but did they happen //
                    // fast enough?
                    if (xTaskGetTickCount() - first_edge_time_ticks <
                        NOISE_DETECT_TIME_MS / portTICK_PERIOD_MS) {

                        ESP_LOGI(TAG,
                                "Got enough edges fast enough, "
                                "turning on output");

                        gpio_set_level(GPIO_OUTPUT, 1);
                        stay_on = true;
                    }
                    else {
                        // we have enough edges, but they were too slow - must
                        // just be noise - reset everything and start over.
                        first_edge_time_ticks = 0;
                        edge_count = 0;
                    }
                }
                else {
                    // else not enough edges yet - count this one and keep going
                    ++edge_count;
                }
            }
            // else - we've detected that it's real PWM and not noise, just
            // leave the output alone until we time out from lack of edges.
        }
        else {
            // timeout - no wiggles
            ESP_LOGI(TAG, "Timeout waiting for edge.");

            first_edge_time_ticks = 0;
            edge_count = 0;
            stay_on = false;

            if (gpio_get_level(GPIO_INPUT) == 0) {
                ESP_LOGI(TAG,
                         "No edges and input is low - turning off output.");

                // No PWM and the signal is 0, which means the lights are off -
                // turn the radio dimmer signal off too.
                gpio_set_level(GPIO_OUTPUT, 0);
            }
            else {
                ESP_LOGI(TAG,
                         "No edges and input is high - turning on output.");

                // No PWM and the signal is 1, which means the lights are on but
                // the console is at full brightness, so turn the radio dimmer
                // on.
                gpio_set_level(GPIO_OUTPUT, 1);
            }
        }
    }
}
