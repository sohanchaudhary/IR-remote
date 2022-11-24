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

static const char *TAG = "airton_parser";
#define AIRTON_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define AIRTON_DATA_FRAME_RMT_WORDS (58)
#define AIRTON_REPEAT_FRAME_RMT_WORDS (2)

typedef struct {
    ir_parser_t parent;
    uint32_t flags;
    uint32_t leading_code_high_ticks;
    uint32_t leading_code_low_ticks;
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
} airton_parser_t;

static inline bool airton_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

static bool airton_parse_head(airton_parser_t *airton_parser)
{
    airton_parser->cursor = 0;
    rmt_item32_t item = airton_parser->buffer[airton_parser->cursor];
    bool ret = (item.level0 == airton_parser->inverse) && (item.level1 != airton_parser->inverse) &&
               airton_check_in_range(item.duration0, airton_parser->leading_code_high_ticks, airton_parser->margin_ticks) &&
               airton_check_in_range(item.duration1, airton_parser->leading_code_low_ticks, airton_parser->margin_ticks);
    airton_parser->cursor += 1;
    return ret;
}

static bool airton_parse_logic0(airton_parser_t *airton_parser)
{
    rmt_item32_t item = airton_parser->buffer[airton_parser->cursor];
    bool ret = (item.level0 == airton_parser->inverse) && (item.level1 != airton_parser->inverse) &&
               airton_check_in_range(item.duration0, airton_parser->payload_logic0_high_ticks, airton_parser->margin_ticks) &&
               airton_check_in_range(item.duration1, airton_parser->payload_logic0_low_ticks, airton_parser->margin_ticks);
    return ret;
}

static bool airton_parse_logic1(airton_parser_t *airton_parser)
{
    rmt_item32_t item = airton_parser->buffer[airton_parser->cursor];
    bool ret = (item.level0 == airton_parser->inverse) && (item.level1 != airton_parser->inverse) &&
               airton_check_in_range(item.duration0, airton_parser->payload_logic1_high_ticks, airton_parser->margin_ticks) &&
               airton_check_in_range(item.duration1, airton_parser->payload_logic1_low_ticks, airton_parser->margin_ticks);
    return ret;
}

static esp_err_t airton_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    airton_parser_t *airton_parser = __containerof(parser, airton_parser_t, parent);
    if (airton_parse_logic0(airton_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (airton_parse_logic1(airton_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    airton_parser->cursor += 1;
    return ret;
}

static esp_err_t airton_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    airton_parser_t *airton_parser = __containerof(parser, airton_parser_t, parent);
    AIRTON_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    airton_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
    if (length == AIRTON_DATA_FRAME_RMT_WORDS) {
        airton_parser->repeat = false;
    } else if (length == AIRTON_REPEAT_FRAME_RMT_WORDS) {
        airton_parser->repeat = true;
    } else {
        ret = ESP_FAIL;
    }
    return ret;
err:
    return ret;
}

static esp_err_t airton_parser_get_scan_code(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    bool logic_value = false;
    airton_parser_t *airton_parser = __containerof(parser, airton_parser_t, parent);

    if (airton_parse_head(airton_parser)) {
        for (int i = 0; i < 28; i++) {
            if (airton_parse_logic(parser, &logic_value) == ESP_OK) {
                addr |= (logic_value << i);
            }
        }
        for (int i = 0; i < 28; i++) {
            if (airton_parse_logic(parser, &logic_value) == ESP_OK) {
                cmd |= (logic_value << i);
            }
        }
        *address = addr;
        *command = cmd;
        *repeat = false;
        // keep it as potential repeat code
        airton_parser->last_address = addr;
        airton_parser->last_command = cmd;
        ret = ESP_OK;
    }
    
    return ret;
}

static esp_err_t airton_parser_del(ir_parser_t *parser)
{
    airton_parser_t *airton_parser = __containerof(parser, airton_parser_t, parent);
    free(airton_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_airton(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    AIRTON_CHECK(config, "airton configuration can't be null", err, NULL);

    airton_parser_t *airton_parser = calloc(1, sizeof(airton_parser_t));
    AIRTON_CHECK(airton_parser, "request memory for airton_parser failed", err, NULL);

    airton_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        airton_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    AIRTON_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    airton_parser->leading_code_high_ticks = (uint32_t)(ratio * AIRTON_LEADING_CODE_HIGH_US);
    airton_parser->leading_code_low_ticks = (uint32_t)(ratio * AIRTON_LEADING_CODE_LOW_US);
    airton_parser->payload_logic0_high_ticks = (uint32_t)(ratio * AIRTON_PAYLOAD_ZERO_HIGH_US);
    airton_parser->payload_logic0_low_ticks = (uint32_t)(ratio * AIRTON_PAYLOAD_ZERO_LOW_US);
    airton_parser->payload_logic1_high_ticks = (uint32_t)(ratio * AIRTON_PAYLOAD_ONE_HIGH_US);
    airton_parser->payload_logic1_low_ticks = (uint32_t)(ratio * AIRTON_PAYLOAD_ONE_LOW_US);
    airton_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    airton_parser->parent.input = airton_parser_input;
    airton_parser->parent.get_scan_code = airton_parser_get_scan_code;
    airton_parser->parent.del = airton_parser_del;
    return &airton_parser->parent;
err:
    return ret;
}
