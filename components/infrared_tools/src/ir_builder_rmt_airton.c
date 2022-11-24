#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "airton_builder";
#define AIRTON_CHECK(a, str, goto_tag, ret_value, ...)                               \
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
    uint32_t payload_logic0_high_ticks;
    uint32_t payload_logic0_low_ticks;
    uint32_t payload_logic1_high_ticks;
    uint32_t payload_logic1_low_ticks;
    uint32_t ending_code_high_ticks;
    uint32_t ending_code_low_ticks;
    bool inverse;
    rmt_item32_t buffer[0];
} airton_builder_t;

static esp_err_t airton_builder_make_head(ir_builder_t *builder)
{
    airton_builder_t *airton_builder = __containerof(builder, airton_builder_t, parent);
    airton_builder->cursor = 0;
    airton_builder->buffer[airton_builder->cursor].level0 = !airton_builder->inverse;
    airton_builder->buffer[airton_builder->cursor].duration0 = airton_builder->leading_code_high_ticks;
    airton_builder->buffer[airton_builder->cursor].level1 = airton_builder->inverse;
    airton_builder->buffer[airton_builder->cursor].duration1 = airton_builder->leading_code_low_ticks;
    airton_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t airton_builder_make_logic0(ir_builder_t *builder)
{
    airton_builder_t *airton_builder = __containerof(builder, airton_builder_t, parent);
    airton_builder->buffer[airton_builder->cursor].level0 = !airton_builder->inverse;
    airton_builder->buffer[airton_builder->cursor].duration0 = airton_builder->payload_logic0_high_ticks;
    airton_builder->buffer[airton_builder->cursor].level1 = airton_builder->inverse;
    airton_builder->buffer[airton_builder->cursor].duration1 = airton_builder->payload_logic0_low_ticks;
    airton_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t airton_builder_make_logic1(ir_builder_t *builder)
{
    airton_builder_t *airton_builder = __containerof(builder, airton_builder_t, parent);
    airton_builder->buffer[airton_builder->cursor].level0 = !airton_builder->inverse;
    airton_builder->buffer[airton_builder->cursor].duration0 = airton_builder->payload_logic1_high_ticks;
    airton_builder->buffer[airton_builder->cursor].level1 = airton_builder->inverse;
    airton_builder->buffer[airton_builder->cursor].duration1 = airton_builder->payload_logic1_low_ticks;
    airton_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t airton_builder_make_end(ir_builder_t *builder)
{
    airton_builder_t *airton_builder = __containerof(builder, airton_builder_t, parent);
    airton_builder->buffer[airton_builder->cursor].level0 = !airton_builder->inverse;
    airton_builder->buffer[airton_builder->cursor].duration0 = airton_builder->ending_code_high_ticks;
    airton_builder->buffer[airton_builder->cursor].level1 = airton_builder->inverse;
    airton_builder->buffer[airton_builder->cursor].duration1 = airton_builder->ending_code_low_ticks;
    airton_builder->cursor += 1;
    airton_builder->buffer[airton_builder->cursor].val = 0;
    airton_builder->cursor += 1;
    return ESP_OK;
}


static esp_err_t airton_build_frame(ir_builder_t *builder, uint32_t address, uint32_t command)
{
    esp_err_t ret = ESP_OK;
    airton_builder_t *airton_builder = __containerof(builder, airton_builder_t, parent);
    builder->make_head(builder);
    // LSB -> MSB
    for (int i = 0; i < 28; i++) {
        if (address & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    for (int i = 0; i < 28; i++) {
        if (command & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    builder->make_end(builder);
   // return ESP_OK;
    return ret;
}

static esp_err_t airton_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    airton_builder_t *airton_builder = __containerof(builder, airton_builder_t, parent);
    AIRTON_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = airton_builder->buffer;
    *length = airton_builder->cursor;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t airton_builder_del(ir_builder_t *builder)
{
    airton_builder_t *airton_builder = __containerof(builder, airton_builder_t, parent);
    free(airton_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_airton(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    AIRTON_CHECK(config, "airton configuration can't be null", err, NULL);
    AIRTON_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(airton_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    airton_builder_t *airton_builder = calloc(1, builder_size);
    AIRTON_CHECK(airton_builder, "request memory for airton_builder failed", err, NULL);

    airton_builder->buffer_size = config->buffer_size;
    airton_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        airton_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    AIRTON_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    airton_builder->leading_code_high_ticks = (uint32_t)(ratio * AIRTON_LEADING_CODE_HIGH_US);
    airton_builder->leading_code_low_ticks = (uint32_t)(ratio * AIRTON_LEADING_CODE_LOW_US);
    airton_builder->payload_logic0_high_ticks = (uint32_t)(ratio * AIRTON_PAYLOAD_ZERO_HIGH_US);
    airton_builder->payload_logic0_low_ticks = (uint32_t)(ratio * AIRTON_PAYLOAD_ZERO_LOW_US);
    airton_builder->payload_logic1_high_ticks = (uint32_t)(ratio * AIRTON_PAYLOAD_ONE_HIGH_US);
    airton_builder->payload_logic1_low_ticks = (uint32_t)(ratio * AIRTON_PAYLOAD_ONE_LOW_US);
    airton_builder->ending_code_high_ticks = (uint32_t)(ratio * AIRTON_ENDING_CODE_HIGH_US);
    airton_builder->ending_code_low_ticks = 0x7FFF;
    airton_builder->parent.make_head = airton_builder_make_head;
    airton_builder->parent.make_logic0 = airton_builder_make_logic0;
    airton_builder->parent.make_logic1 = airton_builder_make_logic1;
    airton_builder->parent.make_end = airton_builder_make_end;
    airton_builder->parent.build_frame = airton_build_frame;
    airton_builder->parent.get_result = airton_builder_get_result;
    airton_builder->parent.del = airton_builder_del;
    return &airton_builder->parent;
err:
    return ret;
}
