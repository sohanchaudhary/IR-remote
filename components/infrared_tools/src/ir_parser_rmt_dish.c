#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "dish_parser";
#define DISH_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define DISH_DATA_FRAME_RMT_WORDS (18)
#define DISH_REPEAT_FRAME_RMT_WORDS (2)

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
} dish_parser_t;

static inline bool dish_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

static bool dish_parse_head(dish_parser_t *dish_parser)
{
    dish_parser->cursor = 0;
    rmt_item32_t item = dish_parser->buffer[dish_parser->cursor];
    bool ret = (item.level0 == dish_parser->inverse) && (item.level1 != dish_parser->inverse) &&
               dish_check_in_range(item.duration0, dish_parser->leading_code_high_ticks, dish_parser->margin_ticks) &&
               dish_check_in_range(item.duration1, dish_parser->leading_code_low_ticks, dish_parser->margin_ticks);
    dish_parser->cursor += 1;
    return ret;
}

static bool dish_parse_logic0(dish_parser_t *dish_parser)
{
    rmt_item32_t item = dish_parser->buffer[dish_parser->cursor];
    bool ret = (item.level0 == dish_parser->inverse) && (item.level1 != dish_parser->inverse) &&
               dish_check_in_range(item.duration0, dish_parser->payload_logic0_high_ticks, dish_parser->margin_ticks) &&
               dish_check_in_range(item.duration1, dish_parser->payload_logic0_low_ticks, dish_parser->margin_ticks);
    return ret;
}

static bool dish_parse_logic1(dish_parser_t *dish_parser)
{
    rmt_item32_t item = dish_parser->buffer[dish_parser->cursor];
    bool ret = (item.level0 == dish_parser->inverse) && (item.level1 != dish_parser->inverse) &&
               dish_check_in_range(item.duration0, dish_parser->payload_logic1_high_ticks, dish_parser->margin_ticks) &&
               dish_check_in_range(item.duration1, dish_parser->payload_logic1_low_ticks, dish_parser->margin_ticks);
    return ret;
}

static esp_err_t dish_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    dish_parser_t *dish_parser = __containerof(parser, dish_parser_t, parent);
    if (dish_parse_logic0(dish_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (dish_parse_logic1(dish_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    dish_parser->cursor += 1;
    return ret;
}

static bool dish_parse_repeat_frame(dish_parser_t *dish_parser)
{
    dish_parser->cursor = 0;
    rmt_item32_t item = dish_parser->buffer[dish_parser->cursor];
    bool ret = (item.level0 == dish_parser->inverse) && (item.level1 != dish_parser->inverse) &&
               dish_check_in_range(item.duration0, dish_parser->repeat_code_high_ticks, dish_parser->margin_ticks) &&
               dish_check_in_range(item.duration1, dish_parser->repeat_code_low_ticks, dish_parser->margin_ticks);
    dish_parser->cursor += 1;
    return ret;
}

static esp_err_t dish_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    dish_parser_t *dish_parser = __containerof(parser, dish_parser_t, parent);
    DISH_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    dish_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
    if (length == DISH_DATA_FRAME_RMT_WORDS) {
        dish_parser->repeat = false;
    } else if (length == DISH_REPEAT_FRAME_RMT_WORDS) {
        dish_parser->repeat = true;
    } else {
        ret = ESP_FAIL;
    }
    return ret;
err:
    return ret;
}

static esp_err_t dish_parser_get_scan_code(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    bool logic_value = false;
    dish_parser_t *dish_parser = __containerof(parser, dish_parser_t, parent);
    DISH_CHECK(address && command && repeat, "address, command and repeat can't be null", out, ESP_ERR_INVALID_ARG);
   /* if (dish_parser->repeat) {
        if (dish_parse_repeat_frame(dish_parser)) {
            *address = dish_parser->last_address;
            *command = dish_parser->last_command;
            *repeat = true;
            ret = ESP_OK;
        }
    } else {
        */
    if (dish_parse_head(dish_parser)) {
        for (int i = 0; i < 8; i++) {
            if (dish_parse_logic(parser, &logic_value) == ESP_OK) {
                addr |= (logic_value << i);
            }
        }
        for (int i = 0; i < 8; i++) {
            if (dish_parse_logic(parser, &logic_value) == ESP_OK) {
                cmd |= (logic_value << i);
            }
        }
        *address = addr;
        *command = cmd;
        *repeat = false;
        // keep it as potential repeat code
        //dish_parser->last_address = addr;
        //dish_parser->last_command = cmd;
        ret = ESP_OK;
    }
    //}
out:
    return ret;
}

static esp_err_t dish_parser_del(ir_parser_t *parser)
{
    dish_parser_t *dish_parser = __containerof(parser, dish_parser_t, parent);
    free(dish_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_dish(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    DISH_CHECK(config, "dish configuration can't be null", err, NULL);

    dish_parser_t *dish_parser = calloc(1, sizeof(dish_parser_t));
    DISH_CHECK(dish_parser, "request memory for dish_parser failed", err, NULL);

    dish_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        dish_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    DISH_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    dish_parser->leading_code_high_ticks = (uint32_t)(ratio * DISH_LEADING_CODE_HIGH_US);
    dish_parser->leading_code_low_ticks = (uint32_t)(ratio * DISH_LEADING_CODE_LOW_US);
    dish_parser->repeat_code_high_ticks = (uint32_t)(ratio * DISH_REPEAT_CODE_HIGH_US);
    dish_parser->repeat_code_low_ticks = (uint32_t)(ratio * DISH_REPEAT_CODE_LOW_US);
    dish_parser->payload_logic0_high_ticks = (uint32_t)(ratio * DISH_PAYLOAD_ZERO_HIGH_US);
    dish_parser->payload_logic0_low_ticks = (uint32_t)(ratio * DISH_PAYLOAD_ZERO_LOW_US);
    dish_parser->payload_logic1_high_ticks = (uint32_t)(ratio * DISH_PAYLOAD_ONE_HIGH_US);
    dish_parser->payload_logic1_low_ticks = (uint32_t)(ratio * DISH_PAYLOAD_ONE_LOW_US);
    dish_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    dish_parser->parent.input = dish_parser_input;
    dish_parser->parent.get_scan_code = dish_parser_get_scan_code;
    dish_parser->parent.del = dish_parser_del;
    return &dish_parser->parent;
err:
    return ret;
}
