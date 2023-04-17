/* IR protocols example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "esp_task_wdt.h"
#include "ir_tools.h"

static const char *TAG = "example";

static rmt_channel_t example_tx_channel = CONFIG_EXAMPLE_RMT_TX_CHANNEL;
static rmt_channel_t example_rx_channel = CONFIG_EXAMPLE_RMT_RX_CHANNEL;

#define IR_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

/**
 * @brief IR Parser timings and functions
 *
 */
typedef struct {
    // ir_parser_t parent;
    // uint32_t flags;
    // uint32_t leading_code_high_ticks;
    // uint32_t leading_code_low_ticks;
    // uint32_t message_high_ticks;
    // uint32_t message_low_ticks;
    // uint32_t repeat_code_high_ticks;
    // uint32_t repeat_code_low_ticks;
    // uint32_t payload_logic0_high_ticks;
    // uint32_t payload_logic0_low_ticks;
    // uint32_t payload_logic1_high_ticks;
    // uint32_t payload_logic1_low_ticks;
    uint32_t margin_ticks;
    rmt_item32_t *buffer;
    uint32_t cursor;
    // uint32_t last_address;
    // uint32_t last_command;
    // bool repeat;
    // bool inverse;
} ir_protocol_parser_t;

typedef struct {
    ir_builder_t parent;
    uint32_t buffer_size;
    uint32_t cursor;
    uint32_t flags;
    bool inverse;
    rmt_item32_t buffer[0];
} ir_protocol_builder_t;

static ir_protocol_parser_t ir_protocol_parser;
static ir_protocol_builder_t *ir_protocol_builder;
size_t length;

void store(rmt_item32_t *items)
{
    
    ir_protocol_parser.buffer = items;
    for(int i = 0; i < length; i++)
    {
        ir_protocol_parser.cursor = i;
        //ESP_LOGI("INFO", "NO HEADER MATCH");
        ESP_LOGI("TIME INFO","bit %d High timing: %d", i, ir_protocol_parser.buffer[ir_protocol_parser.cursor].duration0);
        ESP_LOGI("TIME INFO","bit %d Low timing: %d", i, ir_protocol_parser.buffer[ir_protocol_parser.cursor].duration1);
    } 
}

esp_err_t build(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    IR_CHECK(config, "IR_PROTOCOL configuration can't be null", err, NULL);
    IR_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(ir_protocol_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    ir_protocol_builder = calloc(1, builder_size);
    IR_CHECK(ir_protocol_builder, "request memory for ir_builder failed", err, NULL);

    ir_protocol_builder->buffer_size = config->buffer_size;
    ir_protocol_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        ir_protocol_builder->inverse = true;
    }
    uint32_t counter_clk_hz = 0;
    IR_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    if(length)
    {
        for(int i = 0; i < length; i++)
        {
            ESP_LOGE("ERROR","BUILDING FRAME");
            ir_protocol_builder->cursor = i;
            ir_protocol_builder->buffer[ir_protocol_builder->cursor].level0 = !ir_protocol_builder->inverse;
            ir_protocol_builder->buffer[ir_protocol_builder->cursor].duration0 = (uint32_t)(ratio * ir_protocol_parser.buffer[ir_protocol_parser.cursor].duration0);
            ir_protocol_builder->buffer[ir_protocol_builder->cursor].level1 = ir_protocol_builder->inverse;
            ir_protocol_builder->buffer[ir_protocol_builder->cursor].duration1 = (uint32_t)(ratio * ir_protocol_parser.buffer[ir_protocol_parser.cursor].duration1);
            //esp_task_wdt_reset();   
        }
        ir_protocol_builder->cursor += 1;
        ir_protocol_builder->buffer[ir_protocol_builder->cursor].val = 0;
        ir_protocol_builder->cursor += 1;

        return ESP_OK;
    }
    else
    {
        err:
        return ESP_FAIL;
    }
    
}

/**
 * @brief RMT Transmit Task
 *
 */
static void example_ir_tx_task(void *arg)
{
    uint32_t addr = 0x10;
    uint32_t cmd = 0x20;
    rmt_item32_t *items = NULL;
   // size_t *len = (size_t*)length;
    //ir_builder_t *ir_builder = NULL;

    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, example_tx_channel);
    rmt_tx_config.tx_config.carrier_en = true;
    rmt_config(&rmt_tx_config);
    rmt_driver_install(example_tx_channel, 0, 0);
    ir_builder_config_t ir_builder_config = IR_BUILDER_DEFAULT_CONFIG((ir_dev_t)example_tx_channel);
    ir_builder_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT; // Using extended IR protocols (both NEC and RC5 have extended version)
// #if CONFIG_EXAMPLE_IR_PROTOCOL_NEC
 //   ir_builder = ir_builder_rmt_new_nec(&ir_builder_config);
// #elif CONFIG_EXAMPLE_IR_PROTOCOL_RC5
//     ir_builder = ir_builder_rmt_new_rc5(&ir_builder_config);
// #endif

    if(build(&ir_builder_config) == ESP_OK)
    {
        ESP_LOGI("INFO","FRAME BUILD SUCCESSFULLY");
    }

    if(rmt_write_items(example_tx_channel, ir_protocol_builder->buffer, length, false) == ESP_OK)
    {
        ESP_LOGI("INFO", "DATA WRITTEN ON CHANNEL");
    }
    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    //     // ESP_LOGI(TAG, "Send command 0x%x to address 0x%x", cmd, addr);
    //     // // Send new key code
    //     // ESP_ERROR_CHECK(ir_builder->build_frame(ir_builder, addr, cmd));
    //     // ESP_ERROR_CHECK(ir_builder->get_result(ir_builder, &items, &length));
    //     // //To send data according to the waveform items.
    //     // rmt_write_items(example_tx_channel, items, length, false);
    //     // // Send repeat code
    //     // vTaskDelay(pdMS_TO_TICKS(ir_builder->repeat_period_ms));
    //     // ESP_ERROR_CHECK(ir_builder->build_repeat_frame(ir_builder));
    //     // ESP_ERROR_CHECK(ir_builder->get_result(ir_builder, &items, &length));
    //     // rmt_write_items(example_tx_channel, items, length, false);
    //     // cmd++;
    // }
   // ir_builder->del(ir_builder);
    rmt_driver_uninstall(example_tx_channel);
    vTaskDelete(NULL);
}
// static void example_ir_tx_task(void *arg);

/**
 * @brief RMT Receive Task
 *
 */
static void example_ir_rx_task(void *arg)
{
    uint32_t addr = 0;
    uint32_t cmd = 0;
    //size_t length = 0;
   // size_t prev_length = 0;
    bool repeat = false;
    RingbufHandle_t rb = NULL;
    rmt_item32_t *items = NULL;
    


    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(CONFIG_EXAMPLE_RMT_RX_GPIO, example_rx_channel);
    rmt_config(&rmt_rx_config);
    rmt_driver_install(example_rx_channel, 3000, 0);
    ir_parser_config_t ir_parser_config = IR_PARSER_DEFAULT_CONFIG((ir_dev_t)example_rx_channel);
    ir_parser_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT; // Using extended IR protocols (both NEC and RC5 have extended version)
    ir_parser_t *ir_parser = NULL;
// #if CONFIG_EXAMPLE_IR_PROTOCOL_NEC
//    ir_parser = ir_parser_rmt_new_nec(&ir_parser_config);
// #elif CONFIG_EXAMPLE_IR_PROTOCOL_RC5
//     ir_parser = ir_parser_rmt_new_rc5(&ir_parser_config);
// #endif

    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(example_rx_channel, &rb);
    assert(rb != NULL);
    // Start receive
    rmt_rx_start(example_rx_channel, true);
    while (1) {
        items = (rmt_item32_t *) xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items) {
            length /= 4; // one RMT = 4 Bytes
            ESP_LOGI("INFO", "RECEIVED LENGTH = %d", length);
            
           // len = length;
            store(items);

            vTaskDelay(pdMS_TO_TICKS(2000));
            if(length > 5)
            {
                example_ir_tx_task(length);
            }

            
            // uint32_t data[length];
            // for(i=0; i<length; i++)
            // {
            //     data[i] = 
            // }
            // if (ir_parser->input(ir_parser, items, length) == ESP_OK) {
            //     if (ir_parser->get_scan_code(ir_parser, &addr, &cmd, &repeat) == ESP_OK) {
            //         ESP_LOGI(TAG, "Scan Code %s --- addr: 0x%04x cmd: 0x%04x", repeat ? "(repeat)" : "", addr, cmd);
            //     }
            // }
            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void *) items);
        }
    }
    ir_parser->del(ir_parser);
    rmt_driver_uninstall(example_rx_channel);
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(example_ir_rx_task, "ir_rx_task", 2048*8, NULL, 10, NULL);
   // xTaskCreate(example_ir_tx_task, "ir_tx_task", 2048, NULL, 10, NULL);
}
