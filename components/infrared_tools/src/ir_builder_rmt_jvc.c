//==============================================================================
//                             JJJJJ  V   V   CCCC
//                               J    V   V  C
//                               J     V V   C
//                             J J     V V   C
//                              J       V     CCCC
//==============================================================================
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "jvc_builder";
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

typedef struct {
    ir_builder_t parent;
    uint32_t buffer_size;
    uint32_t cursor;
    uint32_t flags;
    uint32_t leading_code_high_ticks;
    uint32_t leading_code_low_ticks;
   // uint32_t repeat_code_high_ticks;
    //uint32_t repeat_code_low_ticks;
    uint32_t payload_logic0_high_ticks;
    uint32_t payload_logic0_low_ticks;
    uint32_t payload_logic1_high_ticks;
    uint32_t payload_logic1_low_ticks;
    uint32_t ending_code_high_ticks;
    uint32_t ending_code_low_ticks;
    bool inverse;
    rmt_item32_t buffer[0];
} jvc_builder_t;

static esp_err_t jvc_builder_make_head(ir_builder_t *builder)
{
    jvc_builder_t *jvc_builder = __containerof(builder, jvc_builder_t, parent);
    jvc_builder->cursor = 0;
    jvc_builder->buffer[jvc_builder->cursor].level0 = !jvc_builder->inverse;
    jvc_builder->buffer[jvc_builder->cursor].duration0 = jvc_builder->leading_code_high_ticks;
    jvc_builder->buffer[jvc_builder->cursor].level1 = jvc_builder->inverse;
    jvc_builder->buffer[jvc_builder->cursor].duration1 = jvc_builder->leading_code_low_ticks;
    jvc_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t jvc_builder_make_logic0(ir_builder_t *builder)
{
    jvc_builder_t *jvc_builder = __containerof(builder, jvc_builder_t, parent);
    jvc_builder->buffer[jvc_builder->cursor].level0 = !jvc_builder->inverse;
    jvc_builder->buffer[jvc_builder->cursor].duration0 = jvc_builder->payload_logic0_high_ticks;
    jvc_builder->buffer[jvc_builder->cursor].level1 = jvc_builder->inverse;
    jvc_builder->buffer[jvc_builder->cursor].duration1 = jvc_builder->payload_logic0_low_ticks;
    jvc_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t jvc_builder_make_logic1(ir_builder_t *builder)
{
    jvc_builder_t *jvc_builder = __containerof(builder, jvc_builder_t, parent);
    jvc_builder->buffer[jvc_builder->cursor].level0 = !jvc_builder->inverse;
    jvc_builder->buffer[jvc_builder->cursor].duration0 = jvc_builder->payload_logic1_high_ticks;
    jvc_builder->buffer[jvc_builder->cursor].level1 = jvc_builder->inverse;
    jvc_builder->buffer[jvc_builder->cursor].duration1 = jvc_builder->payload_logic1_low_ticks;
    jvc_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t jvc_builder_make_end(ir_builder_t *builder)
{
    jvc_builder_t *jvc_builder = __containerof(builder, jvc_builder_t, parent);
    jvc_builder->buffer[jvc_builder->cursor].level0 = !jvc_builder->inverse;
    jvc_builder->buffer[jvc_builder->cursor].duration0 = jvc_builder->ending_code_high_ticks;
    jvc_builder->buffer[jvc_builder->cursor].level1 = jvc_builder->inverse;
    jvc_builder->buffer[jvc_builder->cursor].duration1 = jvc_builder->ending_code_low_ticks;
    jvc_builder->cursor += 1;
    jvc_builder->buffer[jvc_builder->cursor].val = 0;
    jvc_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t jvc_build_frame(ir_builder_t *builder, uint32_t address, uint32_t command)
{
    esp_err_t ret = ESP_OK;
    jvc_builder_t *jvc_builder = __containerof(builder, jvc_builder_t, parent);
    
    builder->make_head(builder);
    // LSB -> MSB
    for (int i = 0; i < 8; i++) {
        if (address & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    
    for (int i = 0; i < 8; i++) {
        if (command & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    builder->make_end(builder);
    return ESP_OK;

}
/*
static esp_err_t jvc_build_repeat_frame(ir_builder_t *builder)
{
    jvc_builder_t *jvc_builder = __containerof(builder, jvc_builder_t, parent);
    jvc_builder->cursor = 0;
    jvc_builder->buffer[jvc_builder->cursor].level0 = !jvc_builder->inverse;
    jvc_builder->buffer[jvc_builder->cursor].duration0 = jvc_builder->repeat_code_high_ticks;
    jvc_builder->buffer[jvc_builder->cursor].level1 = jvc_builder->inverse;
    jvc_builder->buffer[jvc_builder->cursor].duration1 = jvc_builder->repeat_code_low_ticks;
    jvc_builder->cursor += 1;
    jvc_builder_make_end(builder);
    return ESP_OK;
}
*/

static esp_err_t jvc_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    jvc_builder_t *jvc_builder = __containerof(builder, jvc_builder_t, parent);
    JVC_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = jvc_builder->buffer;
    *length = jvc_builder->cursor;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t jvc_builder_del(ir_builder_t *builder)
{
    jvc_builder_t *jvc_builder = __containerof(builder, jvc_builder_t, parent);
    free(jvc_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_jvc(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    JVC_CHECK(config, "jvc configuration can't be null", err, NULL);
    JVC_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(jvc_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    jvc_builder_t *jvc_builder = calloc(1, builder_size);
    JVC_CHECK(jvc_builder, "request memory for jvc_builder failed", err, NULL);

    jvc_builder->buffer_size = config->buffer_size;
    jvc_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        jvc_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    JVC_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    jvc_builder->leading_code_high_ticks = (uint32_t)(ratio * JVC_LEADING_CODE_HIGH_US);
    jvc_builder->leading_code_low_ticks = (uint32_t)(ratio * JVC_LEADING_CODE_LOW_US);
    //jvc_builder->repeat_code_high_ticks = (uint32_t)(ratio * JVC_REPEAT_CODE_HIGH_US);
    //jvc_builder->repeat_code_low_ticks = (uint32_t)(ratio * JVC_REPEAT_CODE_LOW_US);
    jvc_builder->payload_logic0_high_ticks = (uint32_t)(ratio * JVC_PAYLOAD_ZERO_HIGH_US);
    jvc_builder->payload_logic0_low_ticks = (uint32_t)(ratio * JVC_PAYLOAD_ZERO_LOW_US);
    jvc_builder->payload_logic1_high_ticks = (uint32_t)(ratio * JVC_PAYLOAD_ONE_HIGH_US);
    jvc_builder->payload_logic1_low_ticks = (uint32_t)(ratio * JVC_PAYLOAD_ONE_LOW_US);
    jvc_builder->ending_code_high_ticks = (uint32_t)(ratio * JVC_ENDING_CODE_HIGH_US);
    jvc_builder->ending_code_low_ticks = 0x7FFF;
    jvc_builder->parent.make_head = jvc_builder_make_head;
    jvc_builder->parent.make_logic0 = jvc_builder_make_logic0;
    jvc_builder->parent.make_logic1 = jvc_builder_make_logic1;
    jvc_builder->parent.make_end = jvc_builder_make_end;
    jvc_builder->parent.build_frame = jvc_build_frame;
    //jvc_builder->parent.build_repeat_frame = jvc_build_repeat_frame;
    jvc_builder->parent.get_result = jvc_builder_get_result;
    jvc_builder->parent.del = jvc_builder_del;
    jvc_builder->parent.repeat_period_ms = 60;
    return &jvc_builder->parent;
err:
    return ret;
}
