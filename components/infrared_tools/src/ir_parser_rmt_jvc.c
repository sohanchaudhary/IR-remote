//==============================================================================
//                             JJJJJ  V   V   CCCC
//                               J    V   V  C
//                               J     V V   C
//                             J J     V V   C
//                              J       V     CCCC
//==============================================================================
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "jvc_parser";
#define JVC_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define JVC_DATA_FRAME_RMT_WORDS (18)
#define JVC_REPEAT_FRAME_RMT_WORDS (16)

typedef struct {
    ir_parser_t parent;
    uint32_t flags;
    uint32_t leading_code_high_ticks;
    uint32_t leading_code_low_ticks;
    //uint32_t repeat_code_high_ticks;
    //uint32_t repeat_code_low_ticks;
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
} jvc_parser_t;

static inline bool jvc_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

static bool jvc_parse_head(jvc_parser_t *jvc_parser)
{
    jvc_parser->cursor = 0;
    rmt_item32_t item = jvc_parser->buffer[jvc_parser->cursor];
    bool ret = (item.level0 == jvc_parser->inverse) && (item.level1 != jvc_parser->inverse) &&
               jvc_check_in_range(item.duration0, jvc_parser->leading_code_high_ticks, jvc_parser->margin_ticks) &&
               jvc_check_in_range(item.duration1, jvc_parser->leading_code_low_ticks, jvc_parser->margin_ticks);
    jvc_parser->cursor += 1;
    return ret;
}

static bool jvc_parse_logic0(jvc_parser_t *jvc_parser)
{
    rmt_item32_t item = jvc_parser->buffer[jvc_parser->cursor];
    bool ret = (item.level0 == jvc_parser->inverse) && (item.level1 != jvc_parser->inverse) &&
               jvc_check_in_range(item.duration0, jvc_parser->payload_logic0_high_ticks, jvc_parser->margin_ticks) &&
               jvc_check_in_range(item.duration1, jvc_parser->payload_logic0_low_ticks, jvc_parser->margin_ticks);
    return ret;
}

static bool jvc_parse_logic1(jvc_parser_t *jvc_parser)
{
    rmt_item32_t item = jvc_parser->buffer[jvc_parser->cursor];
    bool ret = (item.level0 == jvc_parser->inverse) && (item.level1 != jvc_parser->inverse) &&
               jvc_check_in_range(item.duration0, jvc_parser->payload_logic1_high_ticks, jvc_parser->margin_ticks) &&
               jvc_check_in_range(item.duration1, jvc_parser->payload_logic1_low_ticks, jvc_parser->margin_ticks);
    return ret;
}

static esp_err_t jvc_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    jvc_parser_t *jvc_parser = __containerof(parser, jvc_parser_t, parent);
    if (jvc_parse_logic0(jvc_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (jvc_parse_logic1(jvc_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    jvc_parser->cursor += 1;
    return ret;
}
/*
static bool jvc_parse_repeat_frame(jvc_parser_t *jvc_parser)
{
    jvc_parser->cursor = 0;
    rmt_item32_t item = jvc_parser->buffer[jvc_parser->cursor];
    bool ret = (item.level0 == jvc_parser->inverse) && (item.level1 != jvc_parser->inverse) &&
               jvc_check_in_range(item.duration0, jvc_parser->repeat_code_high_ticks, jvc_parser->margin_ticks) &&
               jvc_check_in_range(item.duration1, jvc_parser->repeat_code_low_ticks, jvc_parser->margin_ticks);
    jvc_parser->cursor += 1;
    return ret;
}*/

static esp_err_t jvc_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    jvc_parser_t *jvc_parser = __containerof(parser, jvc_parser_t, parent);
    JVC_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    jvc_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
    if (length == JVC_DATA_FRAME_RMT_WORDS) {
        jvc_parser->repeat = false;
    } else if (length == JVC_REPEAT_FRAME_RMT_WORDS) {
        jvc_parser->repeat = true;
    } else {
        ret = ESP_FAIL;
    }
    return ret;
err:
    return ret;
}

static esp_err_t jvc_parser_get_scan_code(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    bool logic_value = false;
    jvc_parser_t *jvc_parser = __containerof(parser, jvc_parser_t, parent);
    JVC_CHECK(address && command && repeat, "address, command and repeat can't be null", out, ESP_ERR_INVALID_ARG);
    /*if (jvc_parser->repeat) {
        if (jvc_parse_repeat_frame(jvc_parser)) {
            *address = jvc_parser->last_address;
            *command = jvc_parser->last_command;
            *repeat = true;
            ret = ESP_OK;
        }
    } else {*/
        if (jvc_parse_head(jvc_parser)) {
            for (int i = 0; i < 8; i++) {
                if (jvc_parse_logic(parser, &logic_value) == ESP_OK) {
                    addr |= (logic_value << i);
                }
            }
            for (int i = 0; i < 8; i++) {
                if (jvc_parse_logic(parser, &logic_value) == ESP_OK) {
                    cmd |= (logic_value << i);
                }
            }
            *address = addr;
            *command = cmd;
            *repeat = false;
            // keep it as potential repeat code
            jvc_parser->last_address = addr;
            jvc_parser->last_command = cmd;
            ret = ESP_OK;
        }
    //}
out:
    return ret;
}

static esp_err_t jvc_parser_del(ir_parser_t *parser)
{
    jvc_parser_t *jvc_parser = __containerof(parser, jvc_parser_t, parent);
    free(jvc_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_jvc(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    JVC_CHECK(config, "jvc configuration can't be null", err, NULL);

    jvc_parser_t *jvc_parser = calloc(1, sizeof(jvc_parser_t));
    JVC_CHECK(jvc_parser, "request memory for jvc_parser failed", err, NULL);

    jvc_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        jvc_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    JVC_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    jvc_parser->leading_code_high_ticks = (uint32_t)(ratio * JVC_LEADING_CODE_HIGH_US);
    jvc_parser->leading_code_low_ticks = (uint32_t)(ratio * JVC_LEADING_CODE_LOW_US);
   // jvc_parser->repeat_code_high_ticks = (uint32_t)(ratio * JVC_REPEAT_CODE_HIGH_US);
    //jvc_parser->repeat_code_low_ticks = (uint32_t)(ratio * JVC_REPEAT_CODE_LOW_US);
    jvc_parser->payload_logic0_high_ticks = (uint32_t)(ratio * JVC_PAYLOAD_ZERO_HIGH_US);
    jvc_parser->payload_logic0_low_ticks = (uint32_t)(ratio * JVC_PAYLOAD_ZERO_LOW_US);
    jvc_parser->payload_logic1_high_ticks = (uint32_t)(ratio * JVC_PAYLOAD_ONE_HIGH_US);
    jvc_parser->payload_logic1_low_ticks = (uint32_t)(ratio * JVC_PAYLOAD_ONE_LOW_US);
    jvc_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    jvc_parser->parent.input = jvc_parser_input;
    jvc_parser->parent.get_scan_code = jvc_parser_get_scan_code;
    jvc_parser->parent.del = jvc_parser_del;
    return &jvc_parser->parent;
err:
    return ret;
}
