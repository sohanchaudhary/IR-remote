/* SAMSUNG IR protocols example

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
#include "ir_tools.h"

static const char *TAG = "example";

static rmt_channel_t example_tx_channel = CONFIG_EXAMPLE_RMT_TX_CHANNEL;
static rmt_channel_t example_rx_channel = CONFIG_EXAMPLE_RMT_RX_CHANNEL;

/**
 * @brief RMT Receive Task
 *
 */
static void example_ir_rx_task(void *arg)
{
    uint32_t addr = 0;
    uint32_t cmd = 0;
    uint32_t cksum = 0;
    uint32_t ftr = 0;
    size_t length = 0;
    bool repeat = false;
    RingbufHandle_t rb = NULL;
    rmt_item32_t *items = NULL;

#if CONFIG_EXAMPLE_IR_PROTOCOL_GREE
        /**
     * @brief Default configuration for RX channel
     *
     */
    #define RMT_DEFAULT_CONFIG_RX(gpio, channel_id) \
        {                                           \
            .rmt_mode = RMT_MODE_RX,                \
            .channel = channel_id,                  \
            .gpio_num = gpio,                       \
            .clk_div = 80,                          \
            .mem_block_num = 1,                     \
            .flags = 0,                             \
            .rx_config = {                          \
                .idle_threshold = 21000,            \
                .filter_ticks_thresh = 100,         \
                .filter_en = true,                  \
            }                                       \
        }
    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(CONFIG_EXAMPLE_RMT_RX_GPIO, example_rx_channel);
    ESP_LOGI("RECEIVER INFO", "GREE CONFIGURED");
    
#elif CONFIG_EXAMPLE_IR_PROTOCOL_SHARP 

        /**
     * @brief Default configuration for RX channel
     *
     */
    #define RMT_DEFAULT_CONFIG_RX(gpio, channel_id) \
        {                                           \
            .rmt_mode = RMT_MODE_RX,                \
            .channel = channel_id,                  \
            .gpio_num = gpio,                       \
            .clk_div = 80,                          \
            .mem_block_num = 1,                     \
            .flags = 0,                             \
            .rx_config = {                          \
                .idle_threshold = 155000,            \
                .filter_ticks_thresh = 100,         \
                .filter_en = true,                  \
            }                                       \
        }
    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(CONFIG_EXAMPLE_RMT_RX_GPIO, example_rx_channel);
    ESP_LOGI("RECEIVER INFO", "SHARP CONFIGURED");
    
#else
    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(CONFIG_EXAMPLE_RMT_RX_GPIO, example_rx_channel);
    ESP_LOGI("INFO"," rx config OK");
#endif

    rmt_config(&rmt_rx_config);
    rmt_driver_install(example_rx_channel, 3000, 0);

#if CONFIG_EXAMPLE_IR_PROTOCOL_LEGO
    /**
     * @brief Default configuration for IR parser
     *
     */
    #define IR_PARSER_DEFAULT_CONFIG(dev) \
        {                                 \
            .dev_hdl = dev,               \
            .flags = 0,                   \
            .margin_us = 100,             \
        }
    ir_parser_config_t ir_parser_config = IR_PARSER_DEFAULT_CONFIG((ir_dev_t)example_rx_channel);
    ESP_LOGI("INFO","LEGO rx parser config OK");
#else
    ir_parser_config_t ir_parser_config = IR_PARSER_DEFAULT_CONFIG((ir_dev_t)example_rx_channel);
    ESP_LOGI("INFO","Other than LEGO rx parser config OK");
#endif

    ir_parser_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT; // Using extended IR protocols (both NEC and RC5 have extended version)
    ir_parser_t *ir_parser = NULL;

#if CONFIG_EXAMPLE_IR_PROTOCOL_NEC | CONFIG_EXAMPLE_IR_PROTOCOL_LGTV | CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBA_TV | CONFIG_EXAMPLE_IR_PROTOCOL_EPSON
    ir_parser = ir_parser_rmt_new_nec(&ir_parser_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_SONY
    ir_parser = ir_parser_rmt_new_sony(&ir_parser_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_RC5
    ir_parser = ir_parser_rmt_new_rc5(&ir_parser_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_SAMSUNG | CONFIG_EXAMPLE_IR_PROTOCOL_SAMSUNGAC
    ir_parser = ir_parser_rmt_new_samsung(&ir_parser_config);  
#elif CONFIG_EXAMPLE_IR_PROTOCOL_LGAC
    ir_parser = ir_parser_rmt_new_lgac(&ir_parser_config);  
#elif CONFIG_EXAMPLE_IR_PROTOCOL_GREE
    ir_parser = ir_parser_rmt_new_gree(&ir_parser_config);  
#elif CONFIG_EXAMPLE_IR_PROTOCOL_PANASONIC
    ir_parser = ir_parser_rmt_new_panasonic(&ir_parser_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_SHARP
    ir_parser = ir_parser_rmt_new_sharp(&ir_parser_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_DISH
    ir_parser = ir_parser_rmt_new_dish(&ir_parser_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_LEGO
    ir_parser = ir_parser_rmt_new_lego(&ir_parser_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBA_AC_72 | CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBAAC50
    ir_parser = ir_parser_rmt_new_toshibaAC(&ir_parser_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_JVC
    ir_parser = ir_parser_rmt_new_jvc(&ir_parser_config);
#endif

    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(example_rx_channel, &rb);
    assert(rb != NULL);
    // Start receive
    //ESP_LOGI("INFO", "START");
    rmt_rx_start(example_rx_channel, true);
    while (1) {
        items = (rmt_item32_t *) xRingbufferReceive(rb, &length, portMAX_DELAY);
       // ESP_LOGI("INFO"," Received raw length = %d", length);
        if (items) {
            length /= 4; // one RMT = 4 Bytes
            ESP_LOGI("INFO"," Received RMT BYTE length = %d", length);
            if (ir_parser->input(ir_parser, items, length) == ESP_OK) {
                 ESP_LOGI("INFO"," Input ok");
#if CONFIG_EXAMPLE_IR_PROTOCOL_GREE
                if (ir_parser->get_scan_code_gree(ir_parser, &addr, &ftr, &cmd, &repeat) == ESP_OK) {
                    //ESP_LOGI("INFO"," scan code OK");
                    ESP_LOGI(TAG, "Scan Code %s --- data0: 0x%x footer: 0x%x data1: 0x%08x", repeat ? "(repeat)" : "", addr, ftr, cmd);
                }
                else {
                    ESP_LOGI("INFO", "scan code NOT OK");
                }
#elif CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBA_AC_72
                if (ir_parser->get_scan_code_toshibaAC(ir_parser, &addr, &cmd, &cksum, &repeat) == ESP_OK) {
                    ESP_LOGI(TAG, "Scan Code %s --- addr: 0x%04x cmd: 0x%04x checksum: 0x%04x", repeat ? "(repeat)" : "", addr, cmd, cksum);
                }
#else
                if (ir_parser->get_scan_code(ir_parser, &addr, &cmd, &repeat) == ESP_OK) {
                    ESP_LOGI(TAG, "Scan Code %s --- address: 0x%04x command: 0x%04x", repeat ? "(repeat)" : "", addr, cmd);
                }
#endif
            }
            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void *) items);
        }
    }
    //delete ring buffer
    vRingbufferDelete(rb);
    ir_parser->del(ir_parser);
    rmt_driver_uninstall(example_rx_channel);
    vTaskDelete(NULL);
}

/**
 * @brief RMT Transmit Task
 *
 */
static void example_ir_tx_task(void *arg)
{

#if CONFIG_EXAMPLE_IR_PROTOCOL_GREE
    uint32_t addr = 0x50200471;
    uint32_t cmd = 0x40004211;
    uint32_t addr1 = 0x50600479;
    uint32_t cmd1 = 0xc0004611;
#elif CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBA_AC_72
    uint32_t addr = 0x50200471;
    uint32_t cmd = 0x40004211;
    uint32_t cksum = 0x11;
    uint32_t addrR = 0xF20D03FC;
    uint32_t cmdR = 0x01504000;
    uint32_t cksumR = 0x11;
#elif CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBAAC50
    uint32_t addr = 0xF20D03;
    uint32_t cmd = 0x015040;
#else 
    uint32_t addr = 0x10;
    uint32_t cmd = 0x20;
#endif

    rmt_item32_t *items = NULL;
    size_t length = 0;
    ir_builder_t *ir_builder = NULL;

    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, example_tx_channel);
    rmt_tx_config.tx_config.carrier_en = true;
    rmt_config(&rmt_tx_config);
    rmt_driver_install(example_tx_channel, 0, 0);
    ir_builder_config_t ir_builder_config = IR_BUILDER_DEFAULT_CONFIG((ir_dev_t)example_tx_channel);
    ir_builder_config.flags |= IR_TOOLS_FLAGS_PROTO_EXT; // Using extended IR protocols (both NEC and RC5 have extended version)

#if CONFIG_EXAMPLE_IR_PROTOCOL_NEC | CONFIG_EXAMPLE_IR_PROTOCOL_LGTV | CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBA_TV | CONFIG_EXAMPLE_IR_PROTOCOL_EPSON
    ir_builder = ir_builder_rmt_new_nec(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_RC5
    ir_builder = ir_builder_rmt_new_rc5(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_SAMSUNG | CONFIG_EXAMPLE_IR_PROTOCOL_SAMSUNGAC
    ir_builder = ir_builder_rmt_new_samsung(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_SONY_12 | CONFIG_EXAMPLE_IR_PROTOCOL_SONY_15 | CONFIG_EXAMPLE_IR_PROTOCOL_SONY_20
    ir_builder = ir_builder_rmt_new_sony(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_LGAC
    ir_builder = ir_builder_rmt_new_lgac(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_GREE
    ir_builder = ir_builder_rmt_new_gree(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_PANASONIC
    ir_builder = ir_builder_rmt_new_panasonic(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_SHARP
    ir_builder = ir_builder_rmt_new_sharp(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_DISH
    ir_builder = ir_builder_rmt_new_dish(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_LEGO
    ir_builder = ir_builder_rmt_new_lego(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBA_AC_72 | CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBAAC50
    ir_builder = ir_builder_rmt_new_toshibaAC(&ir_builder_config);
#elif CONFIG_EXAMPLE_IR_PROTOCOL_JVC
    ir_builder = ir_builder_rmt_new_jvc(&ir_builder_config);
#endif

    while (1) {
#if CONFIG_EXAMPLE_IR_PROTOCOL_GREE
        vTaskDelay(pdMS_TO_TICKS(20000));
#endif
        vTaskDelay(pdMS_TO_TICKS(2000));
#if CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBA_AC_72
        ESP_LOGI(TAG, "Send command 0x%x to address 0x%x with checksum 0x%x", cmd, addr, cksum);
        // Send new key code
        ESP_ERROR_CHECK(ir_builder->build_frame_toshibaAC(ir_builder, addr, cmd, cksum, addrR, cmdR, cksumR));
#else
        ESP_LOGI(TAG, "Send command 0x%x to address 0x%x", cmd, addr);
        // Send new key code
        ESP_ERROR_CHECK(ir_builder->build_frame(ir_builder, addr, cmd));
#endif
        ESP_ERROR_CHECK(ir_builder->get_result(ir_builder, &items, &length));
        ESP_LOGI("INFO"," transmitted length = %d", length);
        //To send data according to the waveform items.
        rmt_write_items(example_tx_channel, items, length, false);
        ESP_LOGI("INFO"," data written on channel");

#if CONFIG_EXAMPLE_IR_PROTOCOL_GREE
        vTaskDelay(pdMS_TO_TICKS(20000));
        ESP_LOGI(TAG, "Send command 0x%x to address 0x%x", cmd1, addr1);
        // Send new key code
        ESP_ERROR_CHECK(ir_builder->build_frame(ir_builder, addr1, cmd1));
        ESP_ERROR_CHECK(ir_builder->get_result(ir_builder, &items, &length));
        //ESP_LOGI("INFO"," transmitted length = %d", length);
        //To send data according to the waveform items.
        rmt_write_items(example_tx_channel, items, length, false);
#endif
#if (!(CONFIG_EXAMPLE_IR_PROTOCOL_GREE | CONFIG_EXAMPLE_IR_PROTOCOL_SHARP | CONFIG_EXAMPLE_IR_PROTOCOL_LEGO | CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBA_AC_72 | CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBAAC50 | CONFIG_EXAMPLE_IR_PROTOCOL_EPSON))       
        // Send repeat code
        vTaskDelay(pdMS_TO_TICKS(ir_builder->repeat_period_ms));
        ESP_ERROR_CHECK(ir_builder->build_repeat_frame(ir_builder));
        ESP_ERROR_CHECK(ir_builder->get_result(ir_builder, &items, &length));
        rmt_write_items(example_tx_channel, items, length, false);
#endif
        cmd++;
    }
    ir_builder->del(ir_builder);
    rmt_driver_uninstall(example_tx_channel);
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(example_ir_rx_task, "ir_rx_task", 2048*8, NULL, 10, NULL);
    xTaskCreate(example_ir_tx_task, "ir_tx_task", 2048*8, NULL, 10, NULL);
}
