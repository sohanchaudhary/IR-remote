//==============================================================================
//       PPPP    AAA   N   N   AAA    SSSS   OOO   N   N  IIIII   CCCC
//       P   P  A   A  NN  N  A   A  S      O   O  NN  N    I    C
//       PPPP   AAAAA  N N N  AAAAA   SSS   O   O  N N N    I    C
//       P      A   A  N  NN  A   A      S  O   O  N  NN    I    C
//       P      A   A  N   N  A   A  SSSS    OOO   N   N  IIIII   CCCC
//==============================================================================

#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "panasonic_parser";
#define PANASONIC_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define PANASONIC_DATA_FRAME_RMT_WORDS (50)
#define PANASONIC_REPEAT_FRAME_RMT_WORDS (2)

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
} panasonic_parser_t;

static inline bool panasonic_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

static bool panasonic_parse_head(panasonic_parser_t *panasonic_parser)
{
    panasonic_parser->cursor = 0;
    rmt_item32_t item = panasonic_parser->buffer[panasonic_parser->cursor];
    bool ret = (item.level0 == panasonic_parser->inverse) && (item.level1 != panasonic_parser->inverse) &&
               panasonic_check_in_range(item.duration0, panasonic_parser->leading_code_high_ticks, panasonic_parser->margin_ticks) &&
               panasonic_check_in_range(item.duration1, panasonic_parser->leading_code_low_ticks, panasonic_parser->margin_ticks);
    panasonic_parser->cursor += 1;
    return ret;
}

static bool panasonic_parse_logic0(panasonic_parser_t *panasonic_parser)
{
    rmt_item32_t item = panasonic_parser->buffer[panasonic_parser->cursor];
    bool ret = (item.level0 == panasonic_parser->inverse) && (item.level1 != panasonic_parser->inverse) &&
               panasonic_check_in_range(item.duration0, panasonic_parser->payload_logic0_high_ticks, panasonic_parser->margin_ticks) &&
               panasonic_check_in_range(item.duration1, panasonic_parser->payload_logic0_low_ticks, panasonic_parser->margin_ticks);
    return ret;
}

static bool panasonic_parse_logic1(panasonic_parser_t *panasonic_parser)
{
    rmt_item32_t item = panasonic_parser->buffer[panasonic_parser->cursor];
    bool ret = (item.level0 == panasonic_parser->inverse) && (item.level1 != panasonic_parser->inverse) &&
               panasonic_check_in_range(item.duration0, panasonic_parser->payload_logic1_high_ticks, panasonic_parser->margin_ticks) &&
               panasonic_check_in_range(item.duration1, panasonic_parser->payload_logic1_low_ticks, panasonic_parser->margin_ticks);
    return ret;
}

static esp_err_t panasonic_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    panasonic_parser_t *panasonic_parser = __containerof(parser, panasonic_parser_t, parent);
    if (panasonic_parse_logic0(panasonic_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (panasonic_parse_logic1(panasonic_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    panasonic_parser->cursor += 1;
    return ret;
}

static bool panasonic_parse_repeat_frame(panasonic_parser_t *panasonic_parser)
{
    panasonic_parser->cursor = 0;
    rmt_item32_t item = panasonic_parser->buffer[panasonic_parser->cursor];
    bool ret = (item.level0 == panasonic_parser->inverse) && (item.level1 != panasonic_parser->inverse) &&
               panasonic_check_in_range(item.duration0, panasonic_parser->repeat_code_high_ticks, panasonic_parser->margin_ticks) &&
               panasonic_check_in_range(item.duration1, panasonic_parser->repeat_code_low_ticks, panasonic_parser->margin_ticks);
    panasonic_parser->cursor += 1;
    return ret;
}

static esp_err_t panasonic_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    panasonic_parser_t *panasonic_parser = __containerof(parser, panasonic_parser_t, parent);
    PANASONIC_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    panasonic_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
    if (length == PANASONIC_DATA_FRAME_RMT_WORDS) {
        panasonic_parser->repeat = false;
    } else if (length == PANASONIC_REPEAT_FRAME_RMT_WORDS) {
        panasonic_parser->repeat = true;
    } else {
        ret = ESP_FAIL;
    }
    return ret;
err:
    return ret;
}

static esp_err_t panasonic_parser_get_scan_code(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    bool logic_value = false;
    panasonic_parser_t *panasonic_parser = __containerof(parser, panasonic_parser_t, parent);
    PANASONIC_CHECK(address && command && repeat, "address, command and repeat can't be null", out, ESP_ERR_INVALID_ARG);
    if (panasonic_parser->repeat) {
        if (panasonic_parse_repeat_frame(panasonic_parser)) {
            *address = panasonic_parser->last_address;
            *command = panasonic_parser->last_command;
            *repeat = true;
            ret = ESP_OK;
        }
    } else {
        if (panasonic_parse_head(panasonic_parser)) {
            for (int i = 0; i < 16; i++) {
                if (panasonic_parse_logic(parser, &logic_value) == ESP_OK) {
                    addr |= (logic_value << i);
                }
            }
            for (int i = 0; i < 32; i++) {
                if (panasonic_parse_logic(parser, &logic_value) == ESP_OK) {
                    cmd |= (logic_value << i);
                }
            }
            *address = addr;
            *command = cmd;
            *repeat = false;
            // keep it as potential repeat code
            panasonic_parser->last_address = addr;
            panasonic_parser->last_command = cmd;
            ret = ESP_OK;
        }
    }
out:
    return ret;
}

static esp_err_t panasonic_parser_del(ir_parser_t *parser)
{
    panasonic_parser_t *panasonic_parser = __containerof(parser, panasonic_parser_t, parent);
    free(panasonic_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_panasonic(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    PANASONIC_CHECK(config, "panasonic configuration can't be null", err, NULL);

    panasonic_parser_t *panasonic_parser = calloc(1, sizeof(panasonic_parser_t));
    PANASONIC_CHECK(panasonic_parser, "request memory for panasonic_parser failed", err, NULL);

    panasonic_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        panasonic_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    PANASONIC_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    panasonic_parser->leading_code_high_ticks = (uint32_t)(ratio * PANASONIC_LEADING_CODE_HIGH_US);
    panasonic_parser->leading_code_low_ticks = (uint32_t)(ratio * PANASONIC_LEADING_CODE_LOW_US);
    panasonic_parser->repeat_code_high_ticks = (uint32_t)(ratio * PANASONIC_REPEAT_CODE_HIGH_US);
    panasonic_parser->repeat_code_low_ticks = (uint32_t)(ratio * PANASONIC_REPEAT_CODE_LOW_US);
    panasonic_parser->payload_logic0_high_ticks = (uint32_t)(ratio * PANASONIC_PAYLOAD_ZERO_HIGH_US);
    panasonic_parser->payload_logic0_low_ticks = (uint32_t)(ratio * PANASONIC_PAYLOAD_ZERO_LOW_US);
    panasonic_parser->payload_logic1_high_ticks = (uint32_t)(ratio * PANASONIC_PAYLOAD_ONE_HIGH_US);
    panasonic_parser->payload_logic1_low_ticks = (uint32_t)(ratio * PANASONIC_PAYLOAD_ONE_LOW_US);
    panasonic_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    panasonic_parser->parent.input = panasonic_parser_input;
    panasonic_parser->parent.get_scan_code = panasonic_parser_get_scan_code;
    panasonic_parser->parent.del = panasonic_parser_del;
    return &panasonic_parser->parent;
err:
    return ret;
}
