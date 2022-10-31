//==============================================================================
//                    DDDD   EEEEE  N   N   OOO   N   N
//                     D  D  E      NN  N  O   O  NN  N
//                     D  D  EEE    N N N  O   O  N N N
//                     D  D  E      N  NN  O   O  N  NN
//                    DDDD   EEEEE  N   N   OOO   N   N
//==============================================================================
//                       SSSS  H   H   AAA   RRRR   PPPP
//                      S      H   H  A   A  R   R  P   P
//                       SSS   HHHHH  AAAAA  RRRR   PPPP
//                          S  H   H  A   A  R  R   P
//                      SSSS   H   H  A   A  R   R  P
//==============================================================================
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "sharp_parser";
#define SHARP_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#define SHARP_DATA_FRAME_RMT_WORDS (16)
#define SHARP_REPEAT_FRAME_RMT_WORDS (2)

typedef struct {
    ir_parser_t parent;
    uint32_t flags;
    uint32_t msggap_high_ticks;
    uint32_t msggap_low_ticks;
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
} sharp_parser_t;

static inline bool sharp_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

 
static bool sharp_parse_spacegap(sharp_parser_t *sharp_parser)
{
    //sharp_parser->cursor = 0;
    //ESP_LOGI("INFO SPACEGAP", "spacegap called");
  //  ESP_LOGI("INFO", "SpaceGap Parser Cursor = %d", sharp_parser->cursor);
    rmt_item32_t item = sharp_parser->buffer[sharp_parser->cursor];
    bool ret = (item.level0 == sharp_parser->inverse) && (item.level1 != sharp_parser->inverse) &&
               sharp_check_in_range(item.duration0, sharp_parser->msggap_high_ticks, sharp_parser->margin_ticks) &&
               sharp_check_in_range(item.duration1, sharp_parser->msggap_low_ticks, sharp_parser->margin_ticks);
    sharp_parser->cursor += 1;
    return ret;
}

static bool sharp_parse_logic0(sharp_parser_t *sharp_parser)
{
    rmt_item32_t item = sharp_parser->buffer[sharp_parser->cursor];
    bool ret = (item.level0 == sharp_parser->inverse) && (item.level1 != sharp_parser->inverse) &&
               sharp_check_in_range(item.duration0, sharp_parser->payload_logic0_high_ticks, sharp_parser->margin_ticks) &&
               sharp_check_in_range(item.duration1, sharp_parser->payload_logic0_low_ticks, sharp_parser->margin_ticks);
    return ret;
}

static bool sharp_parse_logic1(sharp_parser_t *sharp_parser)
{
    rmt_item32_t item = sharp_parser->buffer[sharp_parser->cursor];
    bool ret = (item.level0 == sharp_parser->inverse) && (item.level1 != sharp_parser->inverse) &&
               sharp_check_in_range(item.duration0, sharp_parser->payload_logic1_high_ticks, sharp_parser->margin_ticks) &&
               sharp_check_in_range(item.duration1, sharp_parser->payload_logic1_low_ticks, sharp_parser->margin_ticks);
    return ret;
}

static esp_err_t sharp_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    sharp_parser_t *sharp_parser = __containerof(parser, sharp_parser_t, parent);
    if (sharp_parse_logic0(sharp_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (sharp_parse_logic1(sharp_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    sharp_parser->cursor += 1;
    return ret;
}

static bool sharp_parse_repeat_frame(sharp_parser_t *sharp_parser)
{
    sharp_parser->cursor = 0;
    rmt_item32_t item = sharp_parser->buffer[sharp_parser->cursor];
    bool ret = (item.level0 == sharp_parser->inverse) && (item.level1 != sharp_parser->inverse) &&
               sharp_check_in_range(item.duration0, sharp_parser->repeat_code_high_ticks, sharp_parser->margin_ticks) &&
               sharp_check_in_range(item.duration1, sharp_parser->repeat_code_low_ticks, sharp_parser->margin_ticks);
    sharp_parser->cursor += 1;
    return ret;
}

static esp_err_t sharp_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    sharp_parser_t *sharp_parser = __containerof(parser, sharp_parser_t, parent);
    SHARP_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    sharp_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
   // ESP_LOGI("INFO", "Length = %d", length);
    if (length == SHARP_DATA_FRAME_RMT_WORDS) {
        sharp_parser->repeat = false;
    } else if (length == SHARP_REPEAT_FRAME_RMT_WORDS) {
        sharp_parser->repeat = true;
    } else {
        ret = ESP_FAIL;
    }
    return ret;
err:
    return ret;
}

static esp_err_t sharp_parser_get_scan_code(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    uint32_t addrI = 0;
    uint32_t cmdI = 0;
    rmt_item32_t *items = NULL;
    size_t length = 0;
   // bool repeat = false;
    RingbufHandle_t rb = NULL;
    bool logic_value = false;
    bool check = false;
    sharp_parser_t *sharp_parser = __containerof(parser, sharp_parser_t, parent);
    sharp_parser->cursor = 0;
    SHARP_CHECK(address && command && repeat, "address, command and repeat can't be null", out, ESP_ERR_INVALID_ARG);
   
        for (int i = 0; i < 5; i++) {
            if (sharp_parse_logic(parser, &logic_value) == ESP_OK) {
                addr |= (logic_value << i);
            }
        }
        for (int i = 0; i < 10; i++) {
            if (sharp_parse_logic(parser, &logic_value) == ESP_OK) {
                cmd |= (logic_value << i);
            }
        }
        /*
        items = (rmt_item32_t *) xRingbufferReceive(rb, &length, portMAX_DELAY);
        ESP_LOGI("INFO"," Received raw length = %d", length);
        if (items) {
            length /= 4; // one RMT = 4 Bytes
            ESP_LOGI("INFO"," Received RMT BYTE length = %d", length);
            if (sharp_parser_input(parser, items, length) == ESP_OK) {

                for (int i = 0; i < 5; i++) {
                    if (sharp_parse_logic(parser, &logic_value) == ESP_OK) {
                        addrI |= (logic_value << i);
                    }
                }
                for (int i = 0; i < 10; i++) {
                    if (sharp_parse_logic(parser, &logic_value) == ESP_OK) {
                        cmdI |= (logic_value << i);
                    }
                }
            }
        }
        */
        ESP_LOGI(TAG, "Scan Code --- addr: 0x%04x cmd: 0x%04x", addr, cmd);
        ESP_LOGI(TAG, "Scan Code --- addr: 0x%04x cmd: 0x%04x", addrI, cmdI);
        if (cmd == (cmdI ^ 0x3FF)) {
            check = true;
        }
        SHARP_CHECK(check, "command, inverted command can't be equal", out, ESP_ERR_INVALID_ARG);

        *address = addr;
        *command = cmd;
        *repeat = false;
        // keep it as potential repeat code
        sharp_parser->last_address = addr;
        sharp_parser->last_command = cmd;
        ret = ESP_OK;
out:
    return ret;
    ESP_LOGI("INFO", "ERROR in inverted data");
}

static esp_err_t sharp_parser_del(ir_parser_t *parser)
{
    sharp_parser_t *sharp_parser = __containerof(parser, sharp_parser_t, parent);
    free(sharp_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_sharp(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    SHARP_CHECK(config, "sharp configuration can't be null", err, NULL);

    sharp_parser_t *sharp_parser = calloc(1, sizeof(sharp_parser_t));
    SHARP_CHECK(sharp_parser, "request memory for sharp_parser failed", err, NULL);

    sharp_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        sharp_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    SHARP_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    sharp_parser->msggap_high_ticks = (uint32_t)(ratio * SHARP_ENDING_CODE_HIGH_US);
    sharp_parser->msggap_low_ticks = (uint32_t)(ratio * SHARP_DELAY_LOW_US);
    sharp_parser->repeat_code_high_ticks = (uint32_t)(ratio * SHARP_REPEAT_CODE_HIGH_US);
    sharp_parser->repeat_code_low_ticks = (uint32_t)(ratio * SHARP_REPEAT_CODE_LOW_US);
    sharp_parser->payload_logic0_high_ticks = (uint32_t)(ratio * SHARP_PAYLOAD_ZERO_HIGH_US);
    sharp_parser->payload_logic0_low_ticks = (uint32_t)(ratio * SHARP_PAYLOAD_ZERO_LOW_US);
    sharp_parser->payload_logic1_high_ticks = (uint32_t)(ratio * SHARP_PAYLOAD_ONE_HIGH_US);
    sharp_parser->payload_logic1_low_ticks = (uint32_t)(ratio * SHARP_PAYLOAD_ONE_LOW_US);
    sharp_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    sharp_parser->parent.input = sharp_parser_input;
    sharp_parser->parent.get_scan_code = sharp_parser_get_scan_code;
    sharp_parser->parent.del = sharp_parser_del;
    return &sharp_parser->parent;
err:
    return ret;
}
