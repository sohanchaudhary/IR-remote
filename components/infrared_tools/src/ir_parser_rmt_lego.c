//==============================================================================
//         L       EEEEEE   EEEE    OOOO
//         L       E       E       O    O
//         L       EEEE    E  EEE  O    O
//         L       E       E    E  O    O
//         LLLLLL  EEEEEE   EEEE    OOOO
//==============================================================================
// from LEGO Power Functions RC Manual 26.02.2010 Version 1.20
// https://github.com/jurriaan/Arduino-PowerFunctions/raw/master/LEGO_Power_Functions_RC_v120.pdf
// https://oberguru.net/elektronik/ir/codes/lego_power_functions_train.lircd.conf
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "lego_parser";
#define LEGO_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define LEGO_DATA_FRAME_RMT_WORDS (18)
#define LEGO_REPEAT_FRAME_RMT_WORDS (2)

typedef struct {
    ir_parser_t parent;
    uint32_t flags;
    uint32_t leading_code_high_ticks;
    uint32_t leading_code_low_ticks;
    //uint32_t repeat_code_high_ticks;
   // uint32_t repeat_code_low_ticks;
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
} lego_parser_t;

static inline bool lego_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

static bool lego_parse_head(lego_parser_t *lego_parser)
{
    lego_parser->cursor = 0;
    rmt_item32_t item = lego_parser->buffer[lego_parser->cursor];
    bool ret = (item.level0 == lego_parser->inverse) && (item.level1 != lego_parser->inverse) &&
               lego_check_in_range(item.duration0, lego_parser->leading_code_high_ticks, lego_parser->margin_ticks) &&
               lego_check_in_range(item.duration1, lego_parser->leading_code_low_ticks, lego_parser->margin_ticks);
    lego_parser->cursor += 1;
    return ret;
}

static bool lego_parse_logic0(lego_parser_t *lego_parser)
{
    rmt_item32_t item = lego_parser->buffer[lego_parser->cursor];
    bool ret = (item.level0 == lego_parser->inverse) && (item.level1 != lego_parser->inverse) &&
               lego_check_in_range(item.duration0, lego_parser->payload_logic0_high_ticks, lego_parser->margin_ticks) &&
               lego_check_in_range(item.duration1, lego_parser->payload_logic0_low_ticks, lego_parser->margin_ticks);
    return ret;
}

static bool lego_parse_logic1(lego_parser_t *lego_parser)
{
    rmt_item32_t item = lego_parser->buffer[lego_parser->cursor];
    bool ret = (item.level0 == lego_parser->inverse) && (item.level1 != lego_parser->inverse) &&
               lego_check_in_range(item.duration0, lego_parser->payload_logic1_high_ticks, lego_parser->margin_ticks) &&
               lego_check_in_range(item.duration1, lego_parser->payload_logic1_low_ticks, lego_parser->margin_ticks);
    return ret;
}

static esp_err_t lego_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    lego_parser_t *lego_parser = __containerof(parser, lego_parser_t, parent);
    if (lego_parse_logic0(lego_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (lego_parse_logic1(lego_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    lego_parser->cursor += 1;
    return ret;
}
/*
static bool lego_parse_repeat_frame(lego_parser_t *lego_parser)
{
    lego_parser->cursor = 0;
    rmt_item32_t item = lego_parser->buffer[lego_parser->cursor];
    bool ret = (item.level0 == lego_parser->inverse) && (item.level1 != lego_parser->inverse) &&
               lego_check_in_range(item.duration0, lego_parser->repeat_code_high_ticks, lego_parser->margin_ticks) &&
               lego_check_in_range(item.duration1, lego_parser->repeat_code_low_ticks, lego_parser->margin_ticks);
    lego_parser->cursor += 1;
    return ret;
}
*/
static esp_err_t lego_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    lego_parser_t *lego_parser = __containerof(parser, lego_parser_t, parent);
    LEGO_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    lego_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
    if (length == LEGO_DATA_FRAME_RMT_WORDS) {
        lego_parser->repeat = false;
    } else if (length == LEGO_REPEAT_FRAME_RMT_WORDS) {
        lego_parser->repeat = true;
    } else {
        ret = ESP_FAIL;
    }
    return ret;
err:
    return ret;
}

static esp_err_t lego_parser_get_scan_code(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    bool logic_value = false;
    lego_parser_t *lego_parser = __containerof(parser, lego_parser_t, parent);
    
    if (lego_parse_head(lego_parser)) {
        for (int i = 11; i >= 0; i--) {
            if (lego_parse_logic(parser, &logic_value) == ESP_OK) {
                cmd |= (logic_value << i);
            }
        }
        for (int i = 3; i >= 0; i--) {
            if (lego_parse_logic(parser, &logic_value) == ESP_OK) {
                addr |= (logic_value << i);
            }
        }
        *address = addr;
        *command = cmd;
        *repeat = false;
        //ESP_LOGI(TAG, "Scan Code %s --- address: 0x%04x command: 0x%04x", repeat ? "(repeat)" : "", addr, cmd);
        // keep it as potential repeat code
        //lego_parser->last_address = addr;
        //lego_parser->last_command = cmd;
        ret = ESP_OK;
    }
    else {
        ESP_LOGI("INFO", "decodeing failed");
    }
   // }
//out:
    //ESP_LOGI("INFO", "Error due to repeat");
  return ret;
}

static esp_err_t lego_parser_del(ir_parser_t *parser)
{
    lego_parser_t *lego_parser = __containerof(parser, lego_parser_t, parent);
    free(lego_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_lego(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    LEGO_CHECK(config, "lego configuration can't be null", err, NULL);

    lego_parser_t *lego_parser = calloc(1, sizeof(lego_parser_t));
    LEGO_CHECK(lego_parser, "request memory for lego_parser failed", err, NULL);

    lego_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        lego_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    LEGO_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    lego_parser->leading_code_high_ticks = (uint32_t)(ratio * LEGO_LEADING_CODE_HIGH_US);
    lego_parser->leading_code_low_ticks = (uint32_t)(ratio * LEGO_LEADING_CODE_LOW_US);
    //lego_parser->repeat_code_high_ticks = (uint32_t)(ratio * LEGO_REPEAT_CODE_HIGH_US);
    //lego_parser->repeat_code_low_ticks = (uint32_t)(ratio * LEGO_REPEAT_CODE_LOW_US);
    lego_parser->payload_logic0_high_ticks = (uint32_t)(ratio * LEGO_PAYLOAD_ZERO_HIGH_US);
    lego_parser->payload_logic0_low_ticks = (uint32_t)(ratio * LEGO_PAYLOAD_ZERO_LOW_US);
    lego_parser->payload_logic1_high_ticks = (uint32_t)(ratio * LEGO_PAYLOAD_ONE_HIGH_US);
    lego_parser->payload_logic1_low_ticks = (uint32_t)(ratio * LEGO_PAYLOAD_ONE_LOW_US);
    lego_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    lego_parser->parent.input = lego_parser_input;
    lego_parser->parent.get_scan_code = lego_parser_get_scan_code;
    lego_parser->parent.del = lego_parser_del;
    return &lego_parser->parent;
err:
    return ret;
}
