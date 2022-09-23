// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "sony_parser";
#define SONY_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#if CONFIG_EXAMPLE_IR_PROTOCOL_SONY_12
    #define SONY_DATA_FRAME_RMT_WORDS (13)
#elif CONFIG_EXAMPLE_IR_PROTOCOL_SONY_15
    #define SONY_DATA_FRAME_RMT_WORDS (16)
#if CONFIG_EXAMPLE_IR_PROTOCOL_SONY_20
    #define SONY_DATA_FRAME_RMT_WORDS (21)
#define SONY_REPEAT_FRAME_RMT_WORDS (2)

typedef struct {
    ir_parser_t parent;
    uint32_t flags;
    uint32_t leading_code_high_ticks;
    uint32_t leading_code_low_ticks;
    uint32_t repeat_code_high_ticks;
    uint32_t repeat_code_low_ticks;
    uint32_t payload_logic0_high_ticks;
    uint32_t payload_logic0_low_ticks;
    uint32_t payload_logic1_high_ticks;
    uint32_t payload_logic1_low_ticks;
    uint32_t margin_ticks;
    rmt_item32_t *buffer;
    uint32_t cursor;
    uint32_t last_address;
    uint32_t last_command;
    bool repeat;
    bool inverse;
} sony_parser_t;

static inline bool sony_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

static bool sony_parse_head(sony_parser_t *sony_parser)
{
    sony_parser->cursor = 0;
    rmt_item32_t item = sony_parser->buffer[sony_parser->cursor];
    bool ret = (item.level0 == sony_parser->inverse) && (item.level1 != sony_parser->inverse) &&
               sony_check_in_range(item.duration0, sony_parser->leading_code_high_ticks, sony_parser->margin_ticks) &&
               sony_check_in_range(item.duration1, sony_parser->leading_code_low_ticks, sony_parser->margin_ticks);
    sony_parser->cursor += 1;
    return ret;
}

static bool sony_parse_logic0(sony_parser_t *sony_parser)
{
    rmt_item32_t item = sony_parser->buffer[sony_parser->cursor];
    bool ret = (item.level0 == sony_parser->inverse) && (item.level1 != sony_parser->inverse) &&
               sony_check_in_range(item.duration0, sony_parser->payload_logic0_high_ticks, sony_parser->margin_ticks) &&
               sony_check_in_range(item.duration1, sony_parser->payload_logic0_low_ticks, sony_parser->margin_ticks);
    return ret;
}

static bool sony_parse_logic1(sony_parser_t *sony_parser)
{
    rmt_item32_t item = sony_parser->buffer[sony_parser->cursor];
    bool ret = (item.level0 == sony_parser->inverse) && (item.level1 != sony_parser->inverse) &&
               sony_check_in_range(item.duration0, sony_parser->payload_logic1_high_ticks, sony_parser->margin_ticks) &&
               sony_check_in_range(item.duration1, sony_parser->payload_logic1_low_ticks, sony_parser->margin_ticks);
    return ret;
}

static esp_err_t sony_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    sony_parser_t *sony_parser = __containerof(parser, sony_parser_t, parent);
    if (sony_parse_logic0(sony_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (sony_parse_logic1(sony_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    sony_parser->cursor += 1;
    return ret;
}

static bool sony_parse_repeat_frame(sony_parser_t *sony_parser)
{
    sony_parser->cursor = 0;
    rmt_item32_t item = sony_parser->buffer[sony_parser->cursor];
    bool ret = (item.level0 == sony_parser->inverse) && (item.level1 != sony_parser->inverse) &&
               sony_check_in_range(item.duration0, sony_parser->repeat_code_high_ticks, sony_parser->margin_ticks) &&
               sony_check_in_range(item.duration1, sony_parser->repeat_code_low_ticks, sony_parser->margin_ticks);
    sony_parser->cursor += 1;
    return ret;
}

static esp_err_t sony_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    sony_parser_t *sony_parser = __containerof(parser, sony_parser_t, parent);
    SONY_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    sony_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
    if (length == SONY_DATA_FRAME_RMT_WORDS) {
        sony_parser->repeat = false;
    } else if (length == SONY_REPEAT_FRAME_RMT_WORDS) {
        sony_parser->repeat = true;
    } else {
        ret = ESP_FAIL;
    }
    return ret;
err:
    return ret;
}

static esp_err_t sony_parser_get_scan_code(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    bool logic_value = false;
    sony_parser_t *sony_parser = __containerof(parser, sony_parser_t, parent);
    SONY_CHECK(address && command && repeat, "address, command and repeat can't be null", out, ESP_ERR_INVALID_ARG);
    if (sony_parser->repeat) {
        if (sony_parse_repeat_frame(sony_parser)) {
            *address = sony_parser->last_address;
            *command = sony_parser->last_command;
            *repeat = true;
            ret = ESP_OK;
        }
    } else {
        if (sony_parse_head(sony_parser)) {
            #if CONFIG_EXAMPLE_IR_PROTOCOL_SONY_12
                for (int i = 0; i < 7; i++) {
                    if (sony_parse_logic(parser, &logic_value) == ESP_OK) {
                        cmd |= (logic_value << i);
                    }
                }
                for (int i = 0; i < 5; i++) {
                    if (sony_parse_logic(parser, &logic_value) == ESP_OK) {
                        addr |= (logic_value << i);
                    }
                }
            #elif CONFIG_EXAMPLE_IR_PROTOCOL_SONY_15
                for (int i = 0; i < 7; i++) {
                    if (sony_parse_logic(parser, &logic_value) == ESP_OK) {
                        cmd |= (logic_value << i);
                    }
                }
                for (int i = 0; i < 8; i++) {
                    if (sony_parse_logic(parser, &logic_value) == ESP_OK) {
                        addr |= (logic_value << i);
                    }
                }   
            #elif CONFIG_EXAMPLE_IR_PROTOCOL_SONY_20
                for (int i = 0; i < 7; i++) {
                    if (sony_parse_logic(parser, &logic_value) == ESP_OK) {
                        cmd |= (logic_value << i);
                    }
                }
                for (int i = 0; i < 13; i++) {
                    if (sony_parse_logic(parser, &logic_value) == ESP_OK) {
                        addr |= (logic_value << i);
                    }
                }
            #endif                 
            *address = addr;
            *command = cmd;
            *repeat = false;
            // keep it as potential repeat code
            sony_parser->last_address = addr;
            sony_parser->last_command = cmd;
            ret = ESP_OK;
        }
    }
out:
    return ret;
}

static esp_err_t sony_parser_del(ir_parser_t *parser)
{
    sony_parser_t *sony_parser = __containerof(parser, sony_parser_t, parent);
    free(sony_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_sony(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    SONY_CHECK(config, "sony configuration can't be null", err, NULL);

    sony_parser_t *sony_parser = calloc(1, sizeof(sony_parser_t));
    SONY_CHECK(sony_parser, "request memory for sony_parser failed", err, NULL);

    sony_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        sony_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    SONY_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    sony_parser->leading_code_high_ticks = (uint32_t)(ratio * SONY_LEADING_CODE_HIGH_US);
    sony_parser->leading_code_low_ticks = (uint32_t)(ratio * SONY_LEADING_CODE_LOW_US);
    sony_parser->repeat_code_high_ticks = (uint32_t)(ratio * SONY_REPEAT_CODE_HIGH_US);
    sony_parser->repeat_code_low_ticks = (uint32_t)(ratio * SONY_REPEAT_CODE_LOW_US);
    sony_parser->payload_logic0_high_ticks = (uint32_t)(ratio * SONY_PAYLOAD_ZERO_HIGH_US);
    sony_parser->payload_logic0_low_ticks = (uint32_t)(ratio * SONY_PAYLOAD_ZERO_LOW_US);
    sony_parser->payload_logic1_high_ticks = (uint32_t)(ratio * SONY_PAYLOAD_ONE_HIGH_US);
    sony_parser->payload_logic1_low_ticks = (uint32_t)(ratio * SONY_PAYLOAD_ONE_LOW_US);
    sony_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    sony_parser->parent.input = sony_parser_input;
    sony_parser->parent.get_scan_code = sony_parser_get_scan_code;
    sony_parser->parent.del = sony_parser_del;
    return &sony_parser->parent;
err:
    return ret;
}
