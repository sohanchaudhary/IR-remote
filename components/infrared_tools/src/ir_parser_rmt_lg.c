//==============================================================================
//                               L       GGGG
//                               L      G
//                               L      G  GG
//                               L      G   G
//                               LLLLL   GGG
//==============================================================================
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

static const char *TAG = "lgac_parser";
#define LGAC_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define LGAC_DATA_FRAME_RMT_WORDS (34)
#define LGAC_REPEAT_FRAME_RMT_WORDS (2)
//#define LGAC_DATA_FRAME_RMT_WORDS (30)

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
} lgac_parser_t;

static inline bool lgac_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

static bool lgac_parse_head(lgac_parser_t *lgac_parser)
{
    lgac_parser->cursor = 0;
    rmt_item32_t item = lgac_parser->buffer[lgac_parser->cursor];
    bool ret = (item.level0 == lgac_parser->inverse) && (item.level1 != lgac_parser->inverse) &&
               lgac_check_in_range(item.duration0, lgac_parser->leading_code_high_ticks, lgac_parser->margin_ticks) &&
               lgac_check_in_range(item.duration1, lgac_parser->leading_code_low_ticks, lgac_parser->margin_ticks);
    lgac_parser->cursor += 1;
    return ret;
}

static bool lgac_parse_logic0(lgac_parser_t *lgac_parser)
{
    rmt_item32_t item = lgac_parser->buffer[lgac_parser->cursor];
    bool ret = (item.level0 == lgac_parser->inverse) && (item.level1 != lgac_parser->inverse) &&
               lgac_check_in_range(item.duration0, lgac_parser->payload_logic0_high_ticks, lgac_parser->margin_ticks) &&
               lgac_check_in_range(item.duration1, lgac_parser->payload_logic0_low_ticks, lgac_parser->margin_ticks);
    return ret;
}

static bool lgac_parse_logic1(lgac_parser_t *lgac_parser)
{
    rmt_item32_t item = lgac_parser->buffer[lgac_parser->cursor];
    bool ret = (item.level0 == lgac_parser->inverse) && (item.level1 != lgac_parser->inverse) &&
               lgac_check_in_range(item.duration0, lgac_parser->payload_logic1_high_ticks, lgac_parser->margin_ticks) &&
               lgac_check_in_range(item.duration1, lgac_parser->payload_logic1_low_ticks, lgac_parser->margin_ticks);
    return ret;
}

static esp_err_t lgac_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    lgac_parser_t *lgac_parser = __containerof(parser, lgac_parser_t, parent);
    if (lgac_parse_logic0(lgac_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (lgac_parse_logic1(lgac_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    lgac_parser->cursor += 1;
    return ret;
}

static bool lgac_parse_repeat_frame(lgac_parser_t *lgac_parser)
{
    lgac_parser->cursor = 0;
    rmt_item32_t item = lgac_parser->buffer[lgac_parser->cursor];
    bool ret = (item.level0 == lgac_parser->inverse) && (item.level1 != lgac_parser->inverse) &&
               lgac_check_in_range(item.duration0, lgac_parser->repeat_code_high_ticks, lgac_parser->margin_ticks) &&
               lgac_check_in_range(item.duration1, lgac_parser->repeat_code_low_ticks, lgac_parser->margin_ticks);
    lgac_parser->cursor += 1;
    return ret;
}

static esp_err_t lgac_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    lgac_parser_t *lgac_parser = __containerof(parser, lgac_parser_t, parent);
    LGAC_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    lgac_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
    if (length == LGAC_DATA_FRAME_RMT_WORDS) {
        lgac_parser->repeat = false;
    } else if (length == LGAC_REPEAT_FRAME_RMT_WORDS) {
        lgac_parser->repeat = true;
    } else {
        ret = ESP_FAIL;
    }
    return ret;
err:
    return ret;
}

static esp_err_t lgac_parser_get_scan_code(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    bool logic_value = false;
    lgac_parser_t *lgac_parser = __containerof(parser, lgac_parser_t, parent);
    LGAC_CHECK(address && command && repeat, "address, command and repeat can't be null", out, ESP_ERR_INVALID_ARG);
    /*
    if (lgac_parser->repeat) {
        if (lgac_parse_repeat_frame(lgac_parser)) {
            *address = lgac_parser->last_address;
            *command = lgac_parser->last_command;
            *repeat = true;
            ret = ESP_OK;
        }
    } else {
        if (lgac_parse_head(lgac_parser)) {
            for (int i = 15; i > 0; i--) {
                if (lgac_parse_logic(parser, &logic_value) == ESP_OK) {
                    addr |= (logic_value << i);
                }
            }
            for (int i = 15; i > 0; i--) {
                if (lgac_parse_logic(parser, &logic_value) == ESP_OK) {
                    cmd |= (logic_value << i);
                }
            }
            *address = addr;
            *command = cmd;
            *repeat = false;
            // keep it as potential repeat code
            lgac_parser->last_address = addr;
            lgac_parser->last_command = cmd;
            ret = ESP_OK;
        }
    }
    */

    if (lgac_parse_head(lgac_parser)) {
            for (int i = 15; i >= 0; i--) {
                if (lgac_parse_logic(parser, &logic_value) == ESP_OK) {
                    addr |= (logic_value << i);
                }
            }
            for (int i = 15; i >= 0; i--) {
                if (lgac_parse_logic(parser, &logic_value) == ESP_OK) {
                    cmd |= (logic_value << i);
                }
            }
            *address = addr;
            *command = cmd;
            *repeat = false;
            // keep it as potential repeat code
            lgac_parser->last_address = addr;
            lgac_parser->last_command = cmd;
            ret = ESP_OK;
        }
out:
    return ret;
}
/*
     * My guess of the 4 bit checksum
     * Addition of all 4 nibbles of the 16 bit command
     * 
    uint8_t tChecksum = 0;
    uint16_t tTempForChecksum = aCommand;
    for (int i = 0; i < 4; ++i) {
        tChecksum += tTempForChecksum & 0xF; // add low nibble
        tTempForChecksum >>= 4; // shift by a nibble
    }
    return (tRawData | (tChecksum & 0xF));

*/
static esp_err_t lgac_parser_del(ir_parser_t *parser)
{
    lgac_parser_t *lgac_parser = __containerof(parser, lgac_parser_t, parent);
    free(lgac_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_lgac(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    LGAC_CHECK(config, "lgac configuration can't be null", err, NULL);

    lgac_parser_t *lgac_parser = calloc(1, sizeof(lgac_parser_t));
    LGAC_CHECK(lgac_parser, "request memory for lgac_parser failed", err, NULL);

    lgac_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        lgac_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    LGAC_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    lgac_parser->leading_code_high_ticks = (uint32_t)(ratio * LGAC_LEADING_CODE_HIGH_US);
    lgac_parser->leading_code_low_ticks = (uint32_t)(ratio * LGAC_LEADING_CODE_LOW_US);
    lgac_parser->repeat_code_high_ticks = (uint32_t)(ratio * LGAC_REPEAT_CODE_HIGH_US);
    lgac_parser->repeat_code_low_ticks = (uint32_t)(ratio * LGAC_REPEAT_CODE_LOW_US);
    lgac_parser->payload_logic0_high_ticks = (uint32_t)(ratio * LGAC_PAYLOAD_ZERO_HIGH_US);
    lgac_parser->payload_logic0_low_ticks = (uint32_t)(ratio * LGAC_PAYLOAD_ZERO_LOW_US);
    lgac_parser->payload_logic1_high_ticks = (uint32_t)(ratio * LGAC_PAYLOAD_ONE_HIGH_US);
    lgac_parser->payload_logic1_low_ticks = (uint32_t)(ratio * LGAC_PAYLOAD_ONE_LOW_US);
    lgac_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    lgac_parser->parent.input = lgac_parser_input;
    lgac_parser->parent.get_scan_code = lgac_parser_get_scan_code;
    lgac_parser->parent.del = lgac_parser_del;
    return &lgac_parser->parent;
err:
    return ret;
}

 