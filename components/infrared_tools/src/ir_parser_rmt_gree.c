#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "gree_parser";
#define GREE_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define GREE_DATA_FRAME_RMT_WORDS_data (70)
//#define GREE_DATA_FRAME_RMT_WORDS_data2 (33)
//#define GREE_REPEAT_FRAME_RMT_WORDS (2)

typedef struct {
    ir_parser_t parent;
    uint32_t flags;
    uint32_t leading_code_high_ticks;
    uint32_t leading_code_low_ticks;
    uint32_t message_high_ticks;
    uint32_t message_low_ticks;
    uint32_t payload_logic0_high_ticks;
    uint32_t payload_logic0_low_ticks;
    uint32_t payload_logic1_high_ticks;
    uint32_t payload_logic1_low_ticks;
    uint32_t margin_ticks;
    rmt_item32_t *buffer;
    uint32_t cursor;
    //uint32_t last_address;
   // uint32_t last_footer;
   // uint32_t last_command;
    bool repeat;
    bool inverse;
} gree_parser_t;

static inline bool gree_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

static bool gree_parse_head(gree_parser_t *gree_parser)
{
    gree_parser->cursor = 0;
    rmt_item32_t item = gree_parser->buffer[gree_parser->cursor];
    bool ret = (item.level0 == gree_parser->inverse) && (item.level1 != gree_parser->inverse) &&
               gree_check_in_range(item.duration0, gree_parser->leading_code_high_ticks, gree_parser->margin_ticks) &&
               gree_check_in_range(item.duration1, gree_parser->leading_code_low_ticks, gree_parser->margin_ticks);
    gree_parser->cursor += 1;
    return ret;
}

static bool gree_parse_logic0(gree_parser_t *gree_parser)
{
    rmt_item32_t item = gree_parser->buffer[gree_parser->cursor];
    bool ret = (item.level0 == gree_parser->inverse) && (item.level1 != gree_parser->inverse) &&
               gree_check_in_range(item.duration0, gree_parser->payload_logic0_high_ticks, gree_parser->margin_ticks) &&
               gree_check_in_range(item.duration1, gree_parser->payload_logic0_low_ticks, gree_parser->margin_ticks);
    return ret;
}

static bool gree_parse_logic1(gree_parser_t *gree_parser)
{
    rmt_item32_t item = gree_parser->buffer[gree_parser->cursor];
    bool ret = (item.level0 == gree_parser->inverse) && (item.level1 != gree_parser->inverse) &&
               gree_check_in_range(item.duration0, gree_parser->payload_logic1_high_ticks, gree_parser->margin_ticks) &&
               gree_check_in_range(item.duration1, gree_parser->payload_logic1_low_ticks, gree_parser->margin_ticks);
    return ret;
}

static bool gree_parse_message(gree_parser_t *gree_parser)
{
    rmt_item32_t item = gree_parser->buffer[gree_parser->cursor];
    bool ret = (item.level0 == gree_parser->inverse) && (item.level1 != gree_parser->inverse) &&
               gree_check_in_range(item.duration0, gree_parser->message_high_ticks, gree_parser->margin_ticks) &&
               gree_check_in_range(item.duration1, gree_parser->message_low_ticks, gree_parser->margin_ticks);
    gree_parser->cursor += 1;
   // ESP_LOGI("INFO", "Message function called");
    return ret;
}

static esp_err_t gree_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    gree_parser_t *gree_parser = __containerof(parser, gree_parser_t, parent);
    if (gree_parse_logic0(gree_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (gree_parse_logic1(gree_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    gree_parser->cursor += 1;
    return ret;
}

/*
static bool gree_parse_repeat_frame(gree_parser_t *gree_parser)
{
    gree_parser->cursor = 0;
    rmt_item32_t item = gree_parser->buffer[gree_parser->cursor];
    bool ret = (item.level0 == gree_parser->inverse) && (item.level1 != gree_parser->inverse) &&
               gree_check_in_range(item.duration0, gree_parser->repeat_code_high_ticks, gree_parser->margin_ticks) &&
               gree_check_in_range(item.duration1, gree_parser->repeat_code_low_ticks, gree_parser->margin_ticks);
    gree_parser->cursor += 1;
    return ret;
}
*/

static esp_err_t gree_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    gree_parser_t *gree_parser = __containerof(parser, gree_parser_t, parent);
    GREE_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    gree_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
    if (length == GREE_DATA_FRAME_RMT_WORDS_data) {
        gree_parser->repeat = false;
    } //else if (length == GREE_REPEAT_FRAME_RMT_WORDS) {
       // gree_parser->repeat = true;
    //} 
    else {
        ret = ESP_FAIL;
    }
    //ESP_LOGI("INFO"," Reveiver input function Called");
    return ret;
err:
    return ret;
}

static esp_err_t gree_parser_get_scan_code_gree(ir_parser_t *parser, uint32_t *address, uint32_t *footer, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    uint32_t ftr = 0;
    bool logic_value = false;
    //ESP_LOGI("INFO"," Reveiver scan function Called");
    gree_parser_t *gree_parser = __containerof(parser, gree_parser_t, parent);
    GREE_CHECK(address && command && repeat, "address, command and repeat can't be null", out, ESP_ERR_INVALID_ARG);
    //ESP_LOGI("INFO"," Reveiver scan function Called");
    if (gree_parse_head(gree_parser)) {
        for (int i = 0; i < 32; i++) {
            if (gree_parse_logic(parser, &logic_value) == ESP_OK) {
                    addr |= (logic_value << i);
            }
        }
        for (int i = 0; i < 3; i++) {
            if (gree_parse_logic(parser, &logic_value) == ESP_OK) {
                ftr |= (logic_value << i);
            }
        }
        if (gree_parse_message(gree_parser)){
            ESP_LOGI("INFO", "Message space good");
        }
        //ESP_LOGI("INFO", "Message out");
        for (int i = 0; i < 32; i++) {
            if (gree_parse_logic(parser, &logic_value) == ESP_OK) {
                cmd |= (logic_value << i);
               // ESP_LOGI("INFO", "data1 decoding %d", i);
            }
        }
        //}
            
        *address = addr;
        *command = cmd;
        *repeat = false;
        //ESP_LOGI("INFO"," Reveiver scan function Called");
        // keep it as potential repeat code
        //gree_parser->last_address = addr;
        //gree_parser->last_footer = ftr;
        //gree_parser->last_command = cmd;
        ret = ESP_OK;
    }
    
out:
    return ret;
}

static esp_err_t gree_parser_del(ir_parser_t *parser)
{
    gree_parser_t *gree_parser = __containerof(parser, gree_parser_t, parent);
    free(gree_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_gree(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    GREE_CHECK(config, "gree configuration can't be null", err, NULL);

    gree_parser_t *gree_parser = calloc(1, sizeof(gree_parser_t));
    GREE_CHECK(gree_parser, "request memory for gree_parser failed", err, NULL);

    gree_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        gree_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    GREE_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    gree_parser->leading_code_high_ticks = (uint32_t)(ratio * GREE_LEADING_CODE_HIGH_US);
    gree_parser->leading_code_low_ticks = (uint32_t)(ratio * GREE_LEADING_CODE_LOW_US);
    gree_parser->message_high_ticks = (uint32_t)(ratio * GREE_MESSAGE_SPACE_HIGH_US);
    gree_parser->message_low_ticks = (uint32_t)(ratio * GREE_MESSAGE_SPACE_LOW_US);
    gree_parser->payload_logic0_high_ticks = (uint32_t)(ratio * GREE_PAYLOAD_ZERO_HIGH_US);
    gree_parser->payload_logic0_low_ticks = (uint32_t)(ratio * GREE_PAYLOAD_ZERO_LOW_US);
    gree_parser->payload_logic1_high_ticks = (uint32_t)(ratio * GREE_PAYLOAD_ONE_HIGH_US);
    gree_parser->payload_logic1_low_ticks = (uint32_t)(ratio * GREE_PAYLOAD_ONE_LOW_US);
    gree_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    gree_parser->parent.input = gree_parser_input;
    gree_parser->parent.get_scan_code_gree = gree_parser_get_scan_code_gree;
    gree_parser->parent.del = gree_parser_del;
    return &gree_parser->parent;
err:
    return ret;
}