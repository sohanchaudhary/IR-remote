//==============================================================================
//                           SSSS   OOO   N   N  Y   Y
//                          S      O   O  NN  N   Y Y
//                           SSS   O   O  N N N    Y
//                              S  O   O  N  NN    Y
//                          SSSS    OOO   N   N    Y
//==============================================================================
/*
 * Protocol=Sony Address=0x4B9 Command=0x7 Raw-Data=0x25C87 20 bits LSB first
 * Curently supportive for 12-Bits ; 7 Command Bits , 5 Address Bits
 * The Header is 2.4ms in length, logic 1 is 1.8ms ( 1.2ms high + 0.6ms low), logic 0 is 1.2ms ( 0.6ms high +
0.6ms low). The packet consists of Header, Command code (7-bit) which presents the actual button pressed
on the remote control, and Device code (5-bit) which presents a TV, VCR, CD player and etc. Those signal is
inverted to Rout of the RPM7140. When the data is sent, 45ms delay time before the next packet is sent and
it is repeated for as long as the key is pressed. Table shown below is some list of key Command code for TV
(Device code = 1).
*/

#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "sony_builder";
#define SONY_CHECK(a, str, goto_tag, ret_value, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_value;                                                      \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

typedef struct {
    ir_builder_t parent;
    uint32_t buffer_size;
    uint32_t cursor;
    uint32_t flags;
    uint32_t leading_code_high_ticks;
    uint32_t leading_code_low_ticks;
    uint32_t repeat_code_high_ticks;
    uint32_t repeat_code_low_ticks;
    uint32_t payload_logic0_high_ticks;
    uint32_t payload_logic0_low_ticks;
    uint32_t payload_logic1_high_ticks;
    uint32_t payload_logic1_low_ticks;
    bool inverse;
    rmt_item32_t buffer[0];
} sony_builder_t;

static esp_err_t sony_builder_make_head(ir_builder_t *builder)
{
    sony_builder_t *sony_builder = __containerof(builder, sony_builder_t, parent);
    sony_builder->cursor = 0;
    sony_builder->buffer[sony_builder->cursor].level0 = !sony_builder->inverse;
    sony_builder->buffer[sony_builder->cursor].duration0 = sony_builder->leading_code_high_ticks;
    sony_builder->buffer[sony_builder->cursor].level1 = sony_builder->inverse;
    sony_builder->buffer[sony_builder->cursor].duration1 = sony_builder->leading_code_low_ticks;
    sony_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t sony_builder_make_logic0(ir_builder_t *builder)
{
    sony_builder_t *sony_builder = __containerof(builder, sony_builder_t, parent);
    sony_builder->buffer[sony_builder->cursor].level0 = !sony_builder->inverse;
    sony_builder->buffer[sony_builder->cursor].duration0 = sony_builder->payload_logic0_high_ticks;
    sony_builder->buffer[sony_builder->cursor].level1 = sony_builder->inverse;
    sony_builder->buffer[sony_builder->cursor].duration1 = sony_builder->payload_logic0_low_ticks;
    sony_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t sony_builder_make_logic1(ir_builder_t *builder)
{
    sony_builder_t *sony_builder = __containerof(builder, sony_builder_t, parent);
    sony_builder->buffer[sony_builder->cursor].level0 = !sony_builder->inverse;
    sony_builder->buffer[sony_builder->cursor].duration0 = sony_builder->payload_logic1_high_ticks;
    sony_builder->buffer[sony_builder->cursor].level1 = sony_builder->inverse;
    sony_builder->buffer[sony_builder->cursor].duration1 = sony_builder->payload_logic1_low_ticks;
    sony_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t sony_build_frame(ir_builder_t *builder, uint32_t address, uint32_t command)
{
    esp_err_t ret = ESP_OK;
    sony_builder_t *sony_builder = __containerof(builder, sony_builder_t, parent);
   
   /*******************  NOT USED IN SONY    ***************************
    * 
    if (!sony_builder->flags & IR_TOOLS_FLAGS_PROTO_EXT) {
        uint8_t low_byte = address & 0xFF;
        uint8_t high_byte = (address >> 8) & 0xFF;
        SONY_CHECK(low_byte == (~high_byte & 0xFF), "address not match standard SONY protocol", err, ESP_ERR_INVALID_ARG);
        low_byte = command & 0xFF;
        high_byte = (command >> 8) & 0xFF;
        SONY_CHECK(low_byte == (~high_byte & 0xFF), "command not match standard SONY protocol", err, ESP_ERR_INVALID_ARG);
    }
    *
    */

    builder->make_head(builder);
    #if CONFIG_EXAMPLE_IR_PROTOCOL_SONY_12
        // LSB -> MSB
        for (int i = 0; i < 7; i++) {
            if (command & (1 << i)) {
                builder->make_logic1(builder);
            } else {
                builder->make_logic0(builder);
            }
        }
        for (int i = 0; i < 5; i++) {
            if (address & (1 << i)) {
                builder->make_logic1(builder);
            } else {
                builder->make_logic0(builder);
            }
        }

    #elif CONFIG_EXAMPLE_IR_PROTOCOL_SONY_15
        // LSB -> MSB
        for (int i = 0; i < 7; i++) {
            if (command & (1 << i)) {
                builder->make_logic1(builder);
            } else {
                builder->make_logic0(builder);
            }
        }
        for (int i = 0; i < 8; i++) {
            if (address & (1 << i)) {
                builder->make_logic1(builder);
            } else {
                builder->make_logic0(builder);
            }
        }

    #elif CONFIG_EXAMPLE_IR_PROTOCOL_SONY_20
        // LSB -> MSB
        for (int i = 0; i < 7; i++) {
            if (command & (1 << i)) {
                builder->make_logic1(builder);
            } else {
                builder->make_logic0(builder);
            }
        }
        for (int i = 0; i < 13; i++) {
            if (address & (1 << i)) {
                builder->make_logic1(builder);
            } else {
                builder->make_logic0(builder);
            }
        }
    #endif    
    //builder->make_end(builder);
    return ESP_OK;
//err:
  //  return ret;
}

static esp_err_t sony_build_repeat_frame(ir_builder_t *builder)
{
    sony_builder_t *sony_builder = __containerof(builder, sony_builder_t, parent);
    sony_builder->cursor = 0;
    sony_builder->buffer[sony_builder->cursor].level0 = !sony_builder->inverse;
    sony_builder->buffer[sony_builder->cursor].duration0 = sony_builder->repeat_code_high_ticks;
    sony_builder->buffer[sony_builder->cursor].level1 = sony_builder->inverse;
    sony_builder->buffer[sony_builder->cursor].duration1 = sony_builder->repeat_code_low_ticks;
    sony_builder->cursor += 1;
  //  sony_builder_make_end(builder);
    return ESP_OK;
}

static esp_err_t sony_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    sony_builder_t *sony_builder = __containerof(builder, sony_builder_t, parent);
    SONY_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = sony_builder->buffer;
    *length = sony_builder->cursor;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t sony_builder_del(ir_builder_t *builder)
{
    sony_builder_t *sony_builder = __containerof(builder, sony_builder_t, parent);
    free(sony_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_sony(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    SONY_CHECK(config, "sony configuration can't be null", err, NULL);
    SONY_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(sony_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    sony_builder_t *sony_builder = calloc(1, builder_size);
    SONY_CHECK(sony_builder, "request memory for sony_builder failed", err, NULL);

    sony_builder->buffer_size = config->buffer_size;
    sony_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        sony_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    SONY_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    sony_builder->leading_code_high_ticks = (uint32_t)(ratio * SONY_LEADING_CODE_HIGH_US);
    sony_builder->leading_code_low_ticks = (uint32_t)(ratio * SONY_LEADING_CODE_LOW_US);
    sony_builder->repeat_code_high_ticks = (uint32_t)(ratio * SONY_REPEAT_CODE_HIGH_US);
    sony_builder->repeat_code_low_ticks = (uint32_t)(ratio * SONY_REPEAT_CODE_LOW_US);
    sony_builder->payload_logic0_high_ticks = (uint32_t)(ratio * SONY_PAYLOAD_ZERO_HIGH_US);
    sony_builder->payload_logic0_low_ticks = (uint32_t)(ratio * SONY_PAYLOAD_ZERO_LOW_US);
    sony_builder->payload_logic1_high_ticks = (uint32_t)(ratio * SONY_PAYLOAD_ONE_HIGH_US);
    sony_builder->payload_logic1_low_ticks = (uint32_t)(ratio * SONY_PAYLOAD_ONE_LOW_US);
  //  sony_builder->ending_code_high_ticks = (uint32_t)(ratio * SONY_ENDING_CODE_HIGH_US);
  //  sony_builder->ending_code_low_ticks = 0x7FFF;
    sony_builder->parent.make_head = sony_builder_make_head;
    sony_builder->parent.make_logic0 = sony_builder_make_logic0;
    sony_builder->parent.make_logic1 = sony_builder_make_logic1;
    //sony_builder->parent.make_end = sony_builder_make_end;
    sony_builder->parent.build_frame = sony_build_frame;
    sony_builder->parent.build_repeat_frame = sony_build_repeat_frame;
    sony_builder->parent.get_result = sony_builder_get_result;
    sony_builder->parent.del = sony_builder_del;
    sony_builder->parent.repeat_period_ms = 45;
    return &sony_builder->parent;
err:
    return ret;
}
