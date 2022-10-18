//==============================================================================
//       PPPP    AAA   N   N   AAA    SSSS   OOO   N   N  IIIII   CCCC
//       P   P  A   A  NN  N  A   A  S      O   O  NN  N    I    C
//       PPPP   AAAAA  N N N  AAAAA   SSS   O   O  N N N    I    C
//       P      A   A  N  NN  A   A      S  O   O  N  NN    I    C
//       P      A   A  N   N  A   A  SSSS    OOO   N   N  IIIII   CCCC
//==============================================================================

#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "panasonic_builder";
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
} panasonic_builder_t;

static esp_err_t panasonic_builder_make_head(ir_builder_t *builder)
{
    panasonic_builder_t *panasonic_builder = __containerof(builder, panasonic_builder_t, parent);
    panasonic_builder->cursor = 0;
    panasonic_builder->buffer[panasonic_builder->cursor].level0 = !panasonic_builder->inverse;
    panasonic_builder->buffer[panasonic_builder->cursor].duration0 = panasonic_builder->leading_code_high_ticks;
    panasonic_builder->buffer[panasonic_builder->cursor].level1 = panasonic_builder->inverse;
    panasonic_builder->buffer[panasonic_builder->cursor].duration1 = panasonic_builder->leading_code_low_ticks;
    panasonic_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t panasonic_builder_make_logic0(ir_builder_t *builder)
{
    panasonic_builder_t *panasonic_builder = __containerof(builder, panasonic_builder_t, parent);
    panasonic_builder->buffer[panasonic_builder->cursor].level0 = !panasonic_builder->inverse;
    panasonic_builder->buffer[panasonic_builder->cursor].duration0 = panasonic_builder->payload_logic0_high_ticks;
    panasonic_builder->buffer[panasonic_builder->cursor].level1 = panasonic_builder->inverse;
    panasonic_builder->buffer[panasonic_builder->cursor].duration1 = panasonic_builder->payload_logic0_low_ticks;
    panasonic_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t panasonic_builder_make_logic1(ir_builder_t *builder)
{
    panasonic_builder_t *panasonic_builder = __containerof(builder, panasonic_builder_t, parent);
    panasonic_builder->buffer[panasonic_builder->cursor].level0 = !panasonic_builder->inverse;
    panasonic_builder->buffer[panasonic_builder->cursor].duration0 = panasonic_builder->payload_logic1_high_ticks;
    panasonic_builder->buffer[panasonic_builder->cursor].level1 = panasonic_builder->inverse;
    panasonic_builder->buffer[panasonic_builder->cursor].duration1 = panasonic_builder->payload_logic1_low_ticks;
    panasonic_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t panasonic_builder_make_end(ir_builder_t *builder)
{
    panasonic_builder_t *panasonic_builder = __containerof(builder, panasonic_builder_t, parent);
    panasonic_builder->buffer[panasonic_builder->cursor].level0 = !panasonic_builder->inverse;
    panasonic_builder->buffer[panasonic_builder->cursor].duration0 = panasonic_builder->ending_code_high_ticks;
    panasonic_builder->buffer[panasonic_builder->cursor].level1 = panasonic_builder->inverse;
    panasonic_builder->buffer[panasonic_builder->cursor].duration1 = panasonic_builder->ending_code_low_ticks;
    panasonic_builder->cursor += 1;
    panasonic_builder->buffer[panasonic_builder->cursor].val = 0;
    panasonic_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t panasonic_build_frame(ir_builder_t *builder, uint32_t address, uint32_t command)
{
    esp_err_t ret = ESP_OK;
    panasonic_builder_t *panasonic_builder = __containerof(builder, panasonic_builder_t, parent);
    if (!panasonic_builder->flags & IR_TOOLS_FLAGS_PROTO_EXT) {
        uint8_t low_byte = address & 0xFF;
        uint8_t high_byte = (address >> 8) & 0xFF;
        PANASONIC_CHECK(low_byte == (~high_byte & 0xFF), "address not match standard PANASONIC protocol", err, ESP_ERR_INVALID_ARG);
        low_byte = command & 0xFF;
        high_byte = (command >> 8) & 0xFF;
        PANASONIC_CHECK(low_byte == (~high_byte & 0xFF), "command not match standard PANASONIC protocol", err, ESP_ERR_INVALID_ARG);
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
    for (int i = 0; i < 32; i++) {
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

static esp_err_t panasonic_build_repeat_frame(ir_builder_t *builder)
{
    panasonic_builder_t *panasonic_builder = __containerof(builder, panasonic_builder_t, parent);
    panasonic_builder->cursor = 0;
    panasonic_builder->buffer[panasonic_builder->cursor].level0 = !panasonic_builder->inverse;
    panasonic_builder->buffer[panasonic_builder->cursor].duration0 = panasonic_builder->repeat_code_high_ticks;
    panasonic_builder->buffer[panasonic_builder->cursor].level1 = panasonic_builder->inverse;
    panasonic_builder->buffer[panasonic_builder->cursor].duration1 = panasonic_builder->repeat_code_low_ticks;
    panasonic_builder->cursor += 1;
    panasonic_builder_make_end(builder);
    return ESP_OK;
}

static esp_err_t panasonic_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    panasonic_builder_t *panasonic_builder = __containerof(builder, panasonic_builder_t, parent);
    PANASONIC_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = panasonic_builder->buffer;
    *length = panasonic_builder->cursor;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t panasonic_builder_del(ir_builder_t *builder)
{
    panasonic_builder_t *panasonic_builder = __containerof(builder, panasonic_builder_t, parent);
    free(panasonic_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_panasonic(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    PANASONIC_CHECK(config, "panasonic configuration can't be null", err, NULL);
    PANASONIC_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(panasonic_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    panasonic_builder_t *panasonic_builder = calloc(1, builder_size);
    PANASONIC_CHECK(panasonic_builder, "request memory for panasonic_builder failed", err, NULL);

    panasonic_builder->buffer_size = config->buffer_size;
    panasonic_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        panasonic_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    PANASONIC_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    panasonic_builder->leading_code_high_ticks = (uint32_t)(ratio * PANASONIC_LEADING_CODE_HIGH_US);
    panasonic_builder->leading_code_low_ticks = (uint32_t)(ratio * PANASONIC_LEADING_CODE_LOW_US);
    panasonic_builder->repeat_code_high_ticks = (uint32_t)(ratio * PANASONIC_REPEAT_CODE_HIGH_US);
    panasonic_builder->repeat_code_low_ticks = (uint32_t)(ratio * PANASONIC_REPEAT_CODE_LOW_US);
    panasonic_builder->payload_logic0_high_ticks = (uint32_t)(ratio * PANASONIC_PAYLOAD_ZERO_HIGH_US);
    panasonic_builder->payload_logic0_low_ticks = (uint32_t)(ratio * PANASONIC_PAYLOAD_ZERO_LOW_US);
    panasonic_builder->payload_logic1_high_ticks = (uint32_t)(ratio * PANASONIC_PAYLOAD_ONE_HIGH_US);
    panasonic_builder->payload_logic1_low_ticks = (uint32_t)(ratio * PANASONIC_PAYLOAD_ONE_LOW_US);
    panasonic_builder->ending_code_high_ticks = (uint32_t)(ratio * PANASONIC_ENDING_CODE_HIGH_US);
    panasonic_builder->ending_code_low_ticks = 0x7FFF;
    panasonic_builder->parent.make_head = panasonic_builder_make_head;
    panasonic_builder->parent.make_logic0 = panasonic_builder_make_logic0;
    panasonic_builder->parent.make_logic1 = panasonic_builder_make_logic1;
    panasonic_builder->parent.make_end = panasonic_builder_make_end;
    panasonic_builder->parent.build_frame = panasonic_build_frame;
    panasonic_builder->parent.build_repeat_frame = panasonic_build_repeat_frame;
    panasonic_builder->parent.get_result = panasonic_builder_get_result;
    panasonic_builder->parent.del = panasonic_builder_del;
    panasonic_builder->parent.repeat_period_ms = 110;
    return &panasonic_builder->parent;
err:
    return ret;
}
