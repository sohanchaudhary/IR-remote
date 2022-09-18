//==============================================================================
//              SSSS   AAA    MMM    SSSS  U   U  N   N   GGGG
//             S      A   A  M M M  S      U   U  NN  N  G
//              SSS   AAAAA  M M M   SSS   U   U  N N N  G  GG
//                 S  A   A  M   M      S  U   U  N  NN  G   G
//             SSSS   A   A  M   M  SSSS    UUU   N   N   GGG
//==============================================================================
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "samsung_parser";
#define SAMSUNG_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define SAMSUNG_DATA_FRAME_RMT_WORDS (34)
#define SAMSUNG_REPEAT_FRAME_RMT_WORDS (2)

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
    uint32_t cursor;
    uint32_t last_address;
    uint32_t last_command;
    bool repeat;
    bool inverse;
    rmt_item32_t *buffer;
} samsung_parser_t;

static inline bool samsung_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

static bool samsung_parse_head(samsung_parser_t *samsung_parser)
{
    samsung_parser->cursor = 0;
    rmt_item32_t item = samsung_parser->buffer[samsung_parser->cursor];
    bool ret = (item.level0 == samsung_parser->inverse) && (item.level1 != samsung_parser->inverse) &&
               samsung_check_in_range(item.duration0, samsung_parser->leading_code_high_ticks, samsung_parser->margin_ticks) &&
               samsung_check_in_range(item.duration1, samsung_parser->leading_code_low_ticks, samsung_parser->margin_ticks);
    samsung_parser->cursor += 1;
    return ret;
}

static bool samsung_parse_logic0(samsung_parser_t *samsung_parser)
{
    rmt_item32_t item = samsung_parser->buffer[samsung_parser->cursor];
    bool ret = (item.level0 == samsung_parser->inverse) && (item.level1 != samsung_parser->inverse) &&
               samsung_check_in_range(item.duration0, samsung_parser->payload_logic0_high_ticks, samsung_parser->margin_ticks) &&
               samsung_check_in_range(item.duration1, samsung_parser->payload_logic0_low_ticks, samsung_parser->margin_ticks);
    return ret;
}

static bool samsung_parse_logic1(samsung_parser_t *samsung_parser)
{
    rmt_item32_t item = samsung_parser->buffer[samsung_parser->cursor];
    bool ret = (item.level0 == samsung_parser->inverse) && (item.level1 != samsung_parser->inverse) &&
               samsung_check_in_range(item.duration0, samsung_parser->payload_logic1_high_ticks, samsung_parser->margin_ticks) &&
               samsung_check_in_range(item.duration1, samsung_parser->payload_logic1_low_ticks, samsung_parser->margin_ticks);
    return ret;
}

static esp_err_t samsung_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    samsung_parser_t *samsung_parser = __containerof(parser, samsung_parser_t, parent);
    if (samsung_parse_logic0(samsung_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (samsung_parse_logic1(samsung_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    samsung_parser->cursor += 1;
    return ret;
}


static bool samsung_parse_repeat_frame(samsung_parser_t *samsung_parser)
{
    samsung_parser->cursor = 0;
    rmt_item32_t item = samsung_parser->buffer[samsung_parser->cursor];
    bool ret = (item.level0 == samsung_parser->inverse) && (item.level1 != samsung_parser->inverse) &&
               samsung_check_in_range(item.duration0, samsung_parser->repeat_code_high_ticks, samsung_parser->margin_ticks) &&
               samsung_check_in_range(item.duration1, samsung_parser->repeat_code_low_ticks, samsung_parser->margin_ticks);
    samsung_parser->cursor += 1;
    return ret;
}


static esp_err_t samsung_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    samsung_parser_t *samsung_parser = __containerof(parser, samsung_parser_t, parent);
    SAMSUNG_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    samsung_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
    if (length == SAMSUNG_DATA_FRAME_RMT_WORDS) {
        samsung_parser->repeat = false;
    } else if (length == SAMSUNG_REPEAT_FRAME_RMT_WORDS) {
        samsung_parser->repeat = true;
    } else {
        ret = ESP_FAIL;
    }
    return ret;
err:
    return ret;
}

static esp_err_t samsung_parser_get_scan_code(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    bool logic_value = false;
    samsung_parser_t *samsung_parser = __containerof(parser, samsung_parser_t, parent);
    SAMSUNG_CHECK(address && command && repeat, "address, command and repeat can't be null", out, ESP_ERR_INVALID_ARG);
    if (samsung_parser->repeat) {
        if (samsung_parse_repeat_frame(samsung_parser)) {
            *address = samsung_parser->last_address;
            *command = samsung_parser->last_command;
            *repeat = true;
            ret = ESP_OK;
        }
    } else {
        if (samsung_parse_head(samsung_parser)) {
            for (int i = 0; i < 16; i++) {
                if (samsung_parse_logic(parser, &logic_value) == ESP_OK) {
                    addr |= (logic_value << i);
                }
            }
            for (int i = 0; i < 16; i++) {
                if (samsung_parse_logic(parser, &logic_value) == ESP_OK) {
                    cmd |= (logic_value << i);
                }
            }
            *address = addr;
            *command = cmd;
            *repeat = false;
            // keep it as potential repeat code
            samsung_parser->last_address = addr;
            samsung_parser->last_command = cmd;
            ret = ESP_OK;
        }
    }
out:
    return ret;
}

static esp_err_t samsung_parser_del(ir_parser_t *parser)
{
    samsung_parser_t *samsung_parser = __containerof(parser, samsung_parser_t, parent);
    free(samsung_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_samsung(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    SAMSUNG_CHECK(config, "samsung configuration can't be null", err, NULL);

    samsung_parser_t *samsung_parser = calloc(1, sizeof(samsung_parser_t));
    SAMSUNG_CHECK(samsung_parser, "request memory for samsung_parser failed", err, NULL);

    samsung_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        samsung_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    SAMSUNG_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    samsung_parser->leading_code_high_ticks = (uint32_t)(ratio * SAMSUNG_LEADING_CODE_HIGH_US);
    samsung_parser->leading_code_low_ticks = (uint32_t)(ratio * SAMSUNG_LEADING_CODE_LOW_US);
    samsung_parser->repeat_code_high_ticks = (uint32_t)(ratio * SAMSUNG_REPEAT_CODE_HIGH_US);
    samsung_parser->repeat_code_low_ticks = (uint32_t)(ratio * SAMSUNG_REPEAT_CODE_LOW_US);
    samsung_parser->payload_logic0_high_ticks = (uint32_t)(ratio * SAMSUNG_PAYLOAD_ZERO_HIGH_US);
    samsung_parser->payload_logic0_low_ticks = (uint32_t)(ratio * SAMSUNG_PAYLOAD_ZERO_LOW_US);
    samsung_parser->payload_logic1_high_ticks = (uint32_t)(ratio * SAMSUNG_PAYLOAD_ONE_HIGH_US);
    samsung_parser->payload_logic1_low_ticks = (uint32_t)(ratio * SAMSUNG_PAYLOAD_ONE_LOW_US);
    samsung_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    samsung_parser->parent.input = samsung_parser_input;
    samsung_parser->parent.get_scan_code = samsung_parser_get_scan_code;
    samsung_parser->parent.del = samsung_parser_del;
    return &samsung_parser->parent;
err:
    return ret;
}
