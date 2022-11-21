//=============================================================================
// TTTTTTTTTTT   OOO     SSSS  H  H  IIIII  BBB     AAA        AAA     CCCCC
//     TT      OO   OO  S      H  H    I    B  B   A   A      A   A    C   
//     TT     OO     OO  SSS   HHHH    I    BBB    AAAAA      AAAAA    C
//     TT      OO   OO      S  H  H    I    B  B   A   A      A   A    C   
//     TT        OOO    SSSS   H  H  IIIII  BBB    A   A      A   A    CCCCC
//============================================================================

#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "toshibaAC_parser";
#define TOSHIBAAC_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

#if CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBAAC72
#define TOSHIBAAC_DATA_FRAME_RMT_WORDS (148)
#else
#define TOSHIBAAC_DATA_FRAME_RMT_WORDS (100)
#endif
#define TOSHIBAAC_REPEAT_FRAME_RMT_WORDS (2)

typedef struct {
    ir_parser_t parent;
    uint32_t flags;
    uint32_t leading_code_high_ticks;
    uint32_t leading_code_low_ticks;
    uint32_t message_high_ticks;
    uint32_t message_low_ticks;
    uint32_t new_message_low_ticks;
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
} toshibaAC_parser_t;

static inline bool toshibaAC_check_in_range(uint32_t raw_ticks, uint32_t target_ticks, uint32_t margin_ticks)
{
    return (raw_ticks < (target_ticks + margin_ticks)) && (raw_ticks > (target_ticks - margin_ticks));
}

static bool toshibaAC_parse_head(toshibaAC_parser_t *toshibaAC_parser)
{
    toshibaAC_parser->cursor = 0;
    rmt_item32_t item = toshibaAC_parser->buffer[toshibaAC_parser->cursor];
    bool ret = (item.level0 == toshibaAC_parser->inverse) && (item.level1 != toshibaAC_parser->inverse) &&
               toshibaAC_check_in_range(item.duration0, toshibaAC_parser->leading_code_high_ticks, toshibaAC_parser->margin_ticks) &&
               toshibaAC_check_in_range(item.duration1, toshibaAC_parser->leading_code_low_ticks, toshibaAC_parser->margin_ticks);
    toshibaAC_parser->cursor += 1;
    return ret;
}

static bool toshibaAC_parse_logic0(toshibaAC_parser_t *toshibaAC_parser)
{
    rmt_item32_t item = toshibaAC_parser->buffer[toshibaAC_parser->cursor];
    bool ret = (item.level0 == toshibaAC_parser->inverse) && (item.level1 != toshibaAC_parser->inverse) &&
               toshibaAC_check_in_range(item.duration0, toshibaAC_parser->payload_logic0_high_ticks, toshibaAC_parser->margin_ticks) &&
               toshibaAC_check_in_range(item.duration1, toshibaAC_parser->payload_logic0_low_ticks, toshibaAC_parser->margin_ticks);
    return ret;
}

static bool toshibaAC_parse_logic1(toshibaAC_parser_t *toshibaAC_parser)
{
    rmt_item32_t item = toshibaAC_parser->buffer[toshibaAC_parser->cursor];
    bool ret = (item.level0 == toshibaAC_parser->inverse) && (item.level1 != toshibaAC_parser->inverse) &&
               toshibaAC_check_in_range(item.duration0, toshibaAC_parser->payload_logic1_high_ticks, toshibaAC_parser->margin_ticks) &&
               toshibaAC_check_in_range(item.duration1, toshibaAC_parser->payload_logic1_low_ticks, toshibaAC_parser->margin_ticks);
    return ret;
}

static bool toshibaAC_parse_message_space(toshibaAC_parser_t *toshibaAC_parser)
{
    //toshibaAC_parser->cursor = 0;
    rmt_item32_t item = toshibaAC_parser->buffer[toshibaAC_parser->cursor];
    bool ret = (item.level0 == toshibaAC_parser->inverse) && (item.level1 != toshibaAC_parser->inverse) &&
               toshibaAC_check_in_range(item.duration0, toshibaAC_parser->message_high_ticks, toshibaAC_parser->margin_ticks) &&
               (toshibaAC_check_in_range(item.duration1, toshibaAC_parser->message_low_ticks, toshibaAC_parser->margin_ticks) | toshibaAC_check_in_range(item.duration1, toshibaAC_parser->new_message_low_ticks, toshibaAC_parser->margin_ticks));
    toshibaAC_parser->cursor += 1;
    return ret;
}

static esp_err_t toshibaAC_parse_logic(ir_parser_t *parser, bool *logic)
{
    esp_err_t ret = ESP_FAIL;
    bool logic_value = false;
    toshibaAC_parser_t *toshibaAC_parser = __containerof(parser, toshibaAC_parser_t, parent);
    if (toshibaAC_parse_logic0(toshibaAC_parser)) {
        logic_value = false;
        ret = ESP_OK;
    } else if (toshibaAC_parse_logic1(toshibaAC_parser)) {
        logic_value = true;
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        *logic = logic_value;
    }
    toshibaAC_parser->cursor += 1;
    return ret;
}

static esp_err_t toshibaAC_parser_input(ir_parser_t *parser, void *raw_data, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    toshibaAC_parser_t *toshibaAC_parser = __containerof(parser, toshibaAC_parser_t, parent);
    TOSHIBAAC_CHECK(raw_data, "input data can't be null", err, ESP_ERR_INVALID_ARG);
    toshibaAC_parser->buffer = raw_data;
    // Data Frame costs 34 items and Repeat Frame costs 2 items
    if (length == TOSHIBAAC_DATA_FRAME_RMT_WORDS) {
        toshibaAC_parser->repeat = false;
    } else if (length == TOSHIBAAC_REPEAT_FRAME_RMT_WORDS) {
        toshibaAC_parser->repeat = true;
    } else {
        ret = ESP_FAIL;
    }
    return ret;
err:
    return ret;
}

static esp_err_t toshibaAC_parser_get_scan_code(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    uint32_t addrR = 0;
    uint32_t cmdR = 0;
    bool logic_value = false;
    toshibaAC_parser_t *toshibaAC_parser = __containerof(parser, toshibaAC_parser_t, parent);
    if (toshibaAC_parse_head(toshibaAC_parser)) {
        for (int i = 23; i >= 0; i--) {
            if (toshibaAC_parse_logic(parser, &logic_value) == ESP_OK) {
                addr |= (logic_value << i);
            }
        }
        for (int i = 23; i >= 0; i--) {
            if (toshibaAC_parse_logic(parser, &logic_value) == ESP_OK) {
                cmd |= (logic_value << i);
            }
        }
        //ESP_LOGI(TAG, "Scan Code --- addr: 0x%04x cmd: 0x%04x", addr, cmd);
        if(toshibaAC_parse_message_space(toshibaAC_parser)){
            if (toshibaAC_parse_head(toshibaAC_parser)) {
                for (int i = 23; i >= 0; i--) {
                    if (toshibaAC_parse_logic(parser, &logic_value) == ESP_OK) {
                        addrR |= (logic_value << i);
                    }
                }
                for (int i = 23; i >= 0; i--) {
                    if (toshibaAC_parse_logic(parser, &logic_value) == ESP_OK) {
                        cmdR |= (logic_value << i);
                    }
                }
            }
        }
        if ((addr == addrR) && (cmd == cmdR)) {
            *address = addr;
            *command = cmd;
            *repeat = false;
            // keep it as potential repeat code
            //toshibaAC_parser->last_address = addr;
            //toshibaAC_parser->last_command = cmd;
            ret = ESP_OK;
        }
       
    }
    return ret;
}

static esp_err_t toshibaAC_parser_get_scan_code_toshibaAC(ir_parser_t *parser, uint32_t *address, uint32_t *command, uint32_t *checksum, bool *repeat)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t addr = 0;
    uint32_t cmd = 0;
    uint32_t cksum = 0;
    uint32_t addrR = 0;
    uint32_t cmdR = 0;
    uint32_t cksumR = 0;
    bool logic_value = false;
    toshibaAC_parser_t *toshibaAC_parser = __containerof(parser, toshibaAC_parser_t, parent);
    //TOSHIBAAC_CHECK(address && command && repeat, "address, command and repeat can't be null", out, ESP_ERR_INVALID_ARG);
    //ESP_LOGI("INFO", "Scan Code function called");
    if (toshibaAC_parse_head(toshibaAC_parser)) {
        for (int i = 31; i >= 0; i--) {
            if (toshibaAC_parse_logic(parser, &logic_value) == ESP_OK) {
                addr |= (logic_value << i);
            }
        }
        for (int i = 31; i >= 0; i--) {
            if (toshibaAC_parse_logic(parser, &logic_value) == ESP_OK) {
                cmd |= (logic_value << i);
            }
        }
        for (int i = 7; i >= 0; i--) {
            if (toshibaAC_parse_logic(parser, &logic_value) == ESP_OK) {
                cksum |= (logic_value << i);
            }
        }
        if(toshibaAC_parse_message_space(toshibaAC_parser)){
            if (toshibaAC_parse_head(toshibaAC_parser)) {
                for (int i = 31; i >= 0; i--) {
                    if (toshibaAC_parse_logic(parser, &logic_value) == ESP_OK) {
                        addrR |= (logic_value << i);
                    }
                }
                for (int i = 31; i >= 0; i--) {
                    if (toshibaAC_parse_logic(parser, &logic_value) == ESP_OK) {
                        cmdR |= (logic_value << i);
                    }
                }
                for (int i = 7; i >= 0; i--) {
                    if (toshibaAC_parse_logic(parser, &logic_value) == ESP_OK) {
                        cksumR |= (logic_value << i);
                    }
                }
            }
        }
        if((addr == addrR) && (cmd == cmdR) && (cksum == cksumR)){
            *address = addr;
            *command = cmd;
            *checksum = cksum;
            *repeat = false;
            // keep it as potential repeat code
            toshibaAC_parser->last_address = addr;
            toshibaAC_parser->last_command = cmd;
            ret = ESP_OK;
        }
    }
//out:
    return ret;
}

static esp_err_t toshibaAC_parser_del(ir_parser_t *parser)
{
    toshibaAC_parser_t *toshibaAC_parser = __containerof(parser, toshibaAC_parser_t, parent);
    free(toshibaAC_parser);
    return ESP_OK;
}

ir_parser_t *ir_parser_rmt_new_toshibaAC(const ir_parser_config_t *config)
{
    ir_parser_t *ret = NULL;
    TOSHIBAAC_CHECK(config, "toshibaAC configuration can't be null", err, NULL);

    toshibaAC_parser_t *toshibaAC_parser = calloc(1, sizeof(toshibaAC_parser_t));
    TOSHIBAAC_CHECK(toshibaAC_parser, "request memory for toshibaAC_parser failed", err, NULL);

    toshibaAC_parser->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        toshibaAC_parser->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    TOSHIBAAC_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    #if CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBAAC72
    toshibaAC_parser->leading_code_high_ticks = (uint32_t)(ratio * TOSHIBAAC_LEADING_CODE_HIGH_US);
    toshibaAC_parser->leading_code_low_ticks = (uint32_t)(ratio * TOSHIBAAC_LEADING_CODE_LOW_US);
    toshibaAC_parser->message_high_ticks = (uint32_t)(ratio * TOSHIBAAC_MESSAGE_SPACE_HIGH_US);
    toshibaAC_parser->message_low_ticks = (uint32_t)(ratio * TOSHIBAAC_MESSAGE_SPACE_LOW_US);
    toshibaAC_parser->payload_logic0_high_ticks = (uint32_t)(ratio * TOSHIBAAC_PAYLOAD_ZERO_HIGH_US);
    toshibaAC_parser->payload_logic0_low_ticks = (uint32_t)(ratio * TOSHIBAAC_PAYLOAD_ZERO_LOW_US);
    toshibaAC_parser->payload_logic1_high_ticks = (uint32_t)(ratio * TOSHIBAAC_PAYLOAD_ONE_HIGH_US);
    toshibaAC_parser->payload_logic1_low_ticks = (uint32_t)(ratio * TOSHIBAAC_PAYLOAD_ONE_LOW_US);
    #else
    toshibaAC_parser->leading_code_high_ticks = (uint32_t)(ratio * TOSHIBAAC50_LEADING_CODE_HIGH_US);
    toshibaAC_parser->leading_code_low_ticks = (uint32_t)(ratio * TOSHIBAAC50_LEADING_CODE_LOW_US);
    toshibaAC_parser->message_high_ticks = (uint32_t)(ratio * TOSHIBAAC50_MESSAGE_SPACE_HIGH_US);
    toshibaAC_parser->message_low_ticks = (uint32_t)(ratio * TOSHIBAAC50_MESSAGE_SPACE_LOW_US);
    toshibaAC_parser->new_message_low_ticks = (uint32_t)(ratio * TOSHIBAAC50_NEW_MESSAGE_SPACE_LOW_US);
    toshibaAC_parser->payload_logic0_high_ticks = (uint32_t)(ratio * TOSHIBAAC50_PAYLOAD_ZERO_HIGH_US);
    toshibaAC_parser->payload_logic0_low_ticks = (uint32_t)(ratio * TOSHIBAAC50_PAYLOAD_ZERO_LOW_US);
    toshibaAC_parser->payload_logic1_high_ticks = (uint32_t)(ratio * TOSHIBAAC50_PAYLOAD_ONE_HIGH_US);
    toshibaAC_parser->payload_logic1_low_ticks = (uint32_t)(ratio * TOSHIBAAC50_PAYLOAD_ONE_LOW_US);
    #endif
    toshibaAC_parser->margin_ticks = (uint32_t)(ratio * config->margin_us);
    toshibaAC_parser->parent.input = toshibaAC_parser_input;
    toshibaAC_parser->parent.get_scan_code_toshibaAC = toshibaAC_parser_get_scan_code_toshibaAC;
    toshibaAC_parser->parent.get_scan_code = toshibaAC_parser_get_scan_code;
    toshibaAC_parser->parent.del = toshibaAC_parser_del;
    return &toshibaAC_parser->parent;
err:
    return ret;
}