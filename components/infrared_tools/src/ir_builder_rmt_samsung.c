/*******************
SAMSUNG Protocol
http://techiesms.blogspot.com/2016/01/ir-protocol-decoding-and-transmiting.html
1 bit start bit
8 bit Address
8 bit address
8 bit data
inverse of 8 bit
stop bit

start bit - Address - Address - Data - Inverse Data - stop bit
********************/
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "samsung_builder";
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
    uint32_t ending_code_high_ticks;
    uint32_t ending_code_low_ticks;
    bool inverse;
    rmt_item32_t buffer[0];
} samsung_builder_t;

//code to edit
static esp_err_t samsung_builder_make_head(ir_builder_t *builder)
{
    samsung_builder_t *samsung_builder = __containerof(builder, samsung_builder_t, parent);
    samsung_builder->cursor = 0;
    samsung_builder->buffer[samsung_builder->cursor].level0 = !samsung_builder->inverse;
    samsung_builder->buffer[samsung_builder->cursor].duration0 = samsung_builder->leading_code_high_ticks;
    samsung_builder->buffer[samsung_builder->cursor].level1 = samsung_builder->inverse;
    samsung_builder->buffer[samsung_builder->cursor].duration1 = samsung_builder->leading_code_low_ticks;
    samsung_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t samsung_builder_make_logic0(ir_builder_t *builder)
{
    samsung_builder_t *samsung_builder = __containerof(builder, samsung_builder_t, parent);
    samsung_builder->buffer[samsung_builder->cursor].level0 = !samsung_builder->inverse;
    samsung_builder->buffer[samsung_builder->cursor].duration0 = samsung_builder->payload_logic0_high_ticks;
    samsung_builder->buffer[samsung_builder->cursor].level1 = samsung_builder->inverse;
    samsung_builder->buffer[samsung_builder->cursor].duration1 = samsung_builder->payload_logic0_low_ticks;
    samsung_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t samsung_builder_make_logic1(ir_builder_t *builder)
{
    samsung_builder_t *samsung_builder = __containerof(builder, samsung_builder_t, parent);
    samsung_builder->buffer[samsung_builder->cursor].level0 = !samsung_builder->inverse;
    samsung_builder->buffer[samsung_builder->cursor].duration0 = samsung_builder->payload_logic1_high_ticks;
    samsung_builder->buffer[samsung_builder->cursor].level1 = samsung_builder->inverse;
    samsung_builder->buffer[samsung_builder->cursor].duration1 = samsung_builder->payload_logic1_low_ticks;
    samsung_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t samsung_builder_make_end(ir_builder_t *builder)
{
    samsung_builder_t *samsung_builder = __containerof(builder, samsung_builder_t, parent);
    samsung_builder->buffer[samsung_builder->cursor].level0 = !samsung_builder->inverse;
    samsung_builder->buffer[samsung_builder->cursor].duration0 = samsung_builder->ending_code_high_ticks;
    samsung_builder->buffer[samsung_builder->cursor].level1 = samsung_builder->inverse;
    samsung_builder->buffer[samsung_builder->cursor].duration1 = samsung_builder->ending_code_low_ticks;
    samsung_builder->cursor += 1;
    samsung_builder->buffer[samsung_builder->cursor].val = 0;
    samsung_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t samsung_build_frame(ir_builder_t *builder, uint32_t address, uint32_t command)
{
    esp_err_t ret = ESP_OK;
    samsung_builder_t *samsung_builder = __containerof(builder, samsung_builder_t, parent);
    if (!samsung_builder->flags & IR_TOOLS_FLAGS_PROTO_EXT) {
        uint8_t low_byte = address & 0xFF;
        uint8_t high_byte = (address >> 8) & 0xFF;
        SAMSUNG_CHECK(low_byte == (high_byte & 0xFF), "address not match standard SAMSUNG protocol", err, ESP_ERR_INVALID_ARG);
        low_byte = command & 0xFF;
        high_byte = (command >> 8) & 0xFF;
        SAMSUNG_CHECK(low_byte == (~high_byte & 0xFF), "command not match standard SAMSUNG protocol", err, ESP_ERR_INVALID_ARG);
    }
    builder->make_head(builder);
    // LSB -> MSB
    for (int i = 0; i < 16; i++) {
        if (address & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    for (int i = 0; i < 16; i++) {
        if (command & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    builder->make_end(builder);
    return ESP_OK;
err:
    return ret;
}

static esp_err_t samsung_build_repeat_frame(ir_builder_t *builder)
{
    samsung_builder_t *samsung_builder = __containerof(builder, samsung_builder_t, parent);
    samsung_builder->cursor = 0;
    samsung_builder->buffer[samsung_builder->cursor].level0 = !samsung_builder->inverse;
    samsung_builder->buffer[samsung_builder->cursor].duration0 = samsung_builder->repeat_code_high_ticks;
    samsung_builder->buffer[samsung_builder->cursor].level1 = samsung_builder->inverse;
    samsung_builder->buffer[samsung_builder->cursor].duration1 = samsung_builder->repeat_code_low_ticks;
    samsung_builder->cursor += 1;
    samsung_builder_make_end(builder);
    return ESP_OK;
}

static esp_err_t samsung_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    samsung_builder_t *samsung_builder = __containerof(builder, samsung_builder_t, parent);
    SAMSUNG_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = samsung_builder->buffer;
    *length = samsung_builder->cursor;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t samsung_builder_del(ir_builder_t *builder)
{
    samsung_builder_t *samsung_builder = __containerof(builder, samsung_builder_t, parent);
    free(samsung_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_samsung(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    SAMSUNG_CHECK(config, "samsung configuration can't be null", err, NULL);
    SAMSUNG_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(samsung_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    samsung_builder_t *samsung_builder = calloc(1, builder_size);
    SAMSUNG_CHECK(samsung_builder, "request memory for samsung_builder failed", err, NULL);

    samsung_builder->buffer_size = config->buffer_size;
    samsung_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        samsung_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    SAMSUNG_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    samsung_builder->leading_code_high_ticks = (uint32_t)(ratio * SAMSUNG_LEADING_CODE_HIGH_US);
    samsung_builder->leading_code_low_ticks = (uint32_t)(ratio * SAMSUNG_LEADING_CODE_LOW_US);
    samsung_builder->repeat_code_high_ticks = (uint32_t)(ratio * SAMSUNG_REPEAT_CODE_HIGH_US);
    samsung_builder->repeat_code_low_ticks = (uint32_t)(ratio * SAMSUNG_REPEAT_CODE_LOW_US);
    samsung_builder->payload_logic0_high_ticks = (uint32_t)(ratio * SAMSUNG_PAYLOAD_ZERO_HIGH_US);
    samsung_builder->payload_logic0_low_ticks = (uint32_t)(ratio * SAMSUNG_PAYLOAD_ZERO_LOW_US);
    samsung_builder->payload_logic1_high_ticks = (uint32_t)(ratio * SAMSUNG_PAYLOAD_ONE_HIGH_US);
    samsung_builder->payload_logic1_low_ticks = (uint32_t)(ratio * SAMSUNG_PAYLOAD_ONE_LOW_US);
    samsung_builder->ending_code_high_ticks = (uint32_t)(ratio * SAMSUNG_ENDING_CODE_HIGH_US);
    samsung_builder->ending_code_low_ticks = 0x7FFF;
    samsung_builder->parent.make_head = samsung_builder_make_head;
    samsung_builder->parent.make_logic0 = samsung_builder_make_logic0;
    samsung_builder->parent.make_logic1 = samsung_builder_make_logic1;
    samsung_builder->parent.make_end = samsung_builder_make_end;
    samsung_builder->parent.build_frame = samsung_build_frame;
    samsung_builder->parent.build_repeat_frame = samsung_build_repeat_frame;
    samsung_builder->parent.get_result = samsung_builder_get_result;
    samsung_builder->parent.del = samsung_builder_del;
    samsung_builder->parent.repeat_period_ms = 110;
    return &samsung_builder->parent;
err:
    return ret;
}