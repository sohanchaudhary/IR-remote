#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "dish_builder";
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
} dish_builder_t;

static esp_err_t dish_builder_make_head(ir_builder_t *builder)
{
    dish_builder_t *dish_builder = __containerof(builder, dish_builder_t, parent);
    dish_builder->cursor = 0;
    dish_builder->buffer[dish_builder->cursor].level0 = !dish_builder->inverse;
    dish_builder->buffer[dish_builder->cursor].duration0 = dish_builder->leading_code_high_ticks;
    dish_builder->buffer[dish_builder->cursor].level1 = dish_builder->inverse;
    dish_builder->buffer[dish_builder->cursor].duration1 = dish_builder->leading_code_low_ticks;
    dish_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t dish_builder_make_logic0(ir_builder_t *builder)
{
    dish_builder_t *dish_builder = __containerof(builder, dish_builder_t, parent);
    dish_builder->buffer[dish_builder->cursor].level0 = !dish_builder->inverse;
    dish_builder->buffer[dish_builder->cursor].duration0 = dish_builder->payload_logic0_high_ticks;
    dish_builder->buffer[dish_builder->cursor].level1 = dish_builder->inverse;
    dish_builder->buffer[dish_builder->cursor].duration1 = dish_builder->payload_logic0_low_ticks;
    dish_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t dish_builder_make_logic1(ir_builder_t *builder)
{
    dish_builder_t *dish_builder = __containerof(builder, dish_builder_t, parent);
    dish_builder->buffer[dish_builder->cursor].level0 = !dish_builder->inverse;
    dish_builder->buffer[dish_builder->cursor].duration0 = dish_builder->payload_logic1_high_ticks;
    dish_builder->buffer[dish_builder->cursor].level1 = dish_builder->inverse;
    dish_builder->buffer[dish_builder->cursor].duration1 = dish_builder->payload_logic1_low_ticks;
    dish_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t dish_builder_make_end(ir_builder_t *builder)
{
    dish_builder_t *dish_builder = __containerof(builder, dish_builder_t, parent);
    dish_builder->buffer[dish_builder->cursor].level0 = !dish_builder->inverse;
    dish_builder->buffer[dish_builder->cursor].duration0 = dish_builder->ending_code_high_ticks;
    dish_builder->buffer[dish_builder->cursor].level1 = dish_builder->inverse;
    dish_builder->buffer[dish_builder->cursor].duration1 = dish_builder->ending_code_low_ticks;
    dish_builder->cursor += 1;
    dish_builder->buffer[dish_builder->cursor].val = 0;
    dish_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t dish_build_frame(ir_builder_t *builder, uint32_t address, uint32_t command)
{
    esp_err_t ret = ESP_OK;
    dish_builder_t *dish_builder = __containerof(builder, dish_builder_t, parent);
    /*if (!dish_builder->flags & IR_TOOLS_FLAGS_PROTO_EXT) {
        uint8_t low_byte = address & 0xFF;
        uint8_t high_byte = (address >> 8) & 0xFF;
        DISH_CHECK(low_byte == (~high_byte & 0xFF), "address not match standard DISH protocol", err, ESP_ERR_INVALID_ARG);
        low_byte = command & 0xFF;
        high_byte = (command >> 8) & 0xFF;
        DISH_CHECK(low_byte == (~high_byte & 0xFF), "command not match standard DISH protocol", err, ESP_ERR_INVALID_ARG);
    }*/
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

static esp_err_t dish_build_repeat_frame(ir_builder_t *builder)
{
    dish_builder_t *dish_builder = __containerof(builder, dish_builder_t, parent);
    dish_builder->cursor = 0;
    dish_builder->buffer[dish_builder->cursor].level0 = !dish_builder->inverse;
    dish_builder->buffer[dish_builder->cursor].duration0 = dish_builder->repeat_code_high_ticks;
    dish_builder->buffer[dish_builder->cursor].level1 = dish_builder->inverse;
    dish_builder->buffer[dish_builder->cursor].duration1 = dish_builder->repeat_code_low_ticks;
    dish_builder->cursor += 1;
    dish_builder_make_end(builder);
    return ESP_OK;
}

static esp_err_t dish_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    dish_builder_t *dish_builder = __containerof(builder, dish_builder_t, parent);
    DISH_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = dish_builder->buffer;
    *length = dish_builder->cursor;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t dish_builder_del(ir_builder_t *builder)
{
    dish_builder_t *dish_builder = __containerof(builder, dish_builder_t, parent);
    free(dish_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_dish(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    DISH_CHECK(config, "dish configuration can't be null", err, NULL);
    DISH_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(dish_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    dish_builder_t *dish_builder = calloc(1, builder_size);
    DISH_CHECK(dish_builder, "request memory for dish_builder failed", err, NULL);

    dish_builder->buffer_size = config->buffer_size;
    dish_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        dish_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    DISH_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    dish_builder->leading_code_high_ticks = (uint32_t)(ratio * DISH_LEADING_CODE_HIGH_US);
    dish_builder->leading_code_low_ticks = (uint32_t)(ratio * DISH_LEADING_CODE_LOW_US);
    dish_builder->repeat_code_high_ticks = (uint32_t)(ratio * DISH_REPEAT_CODE_HIGH_US);
    dish_builder->repeat_code_low_ticks = (uint32_t)(ratio * DISH_REPEAT_CODE_LOW_US);
    dish_builder->payload_logic0_high_ticks = (uint32_t)(ratio * DISH_PAYLOAD_ZERO_HIGH_US);
    dish_builder->payload_logic0_low_ticks = (uint32_t)(ratio * DISH_PAYLOAD_ZERO_LOW_US);
    dish_builder->payload_logic1_high_ticks = (uint32_t)(ratio * DISH_PAYLOAD_ONE_HIGH_US);
    dish_builder->payload_logic1_low_ticks = (uint32_t)(ratio * DISH_PAYLOAD_ONE_LOW_US);
    dish_builder->ending_code_high_ticks = (uint32_t)(ratio * DISH_ENDING_CODE_HIGH_US);
    dish_builder->ending_code_low_ticks = 0x7FFF;
    dish_builder->parent.make_head = dish_builder_make_head;
    dish_builder->parent.make_logic0 = dish_builder_make_logic0;
    dish_builder->parent.make_logic1 = dish_builder_make_logic1;
    dish_builder->parent.make_end = dish_builder_make_end;
    dish_builder->parent.build_frame = dish_build_frame;
    dish_builder->parent.build_repeat_frame = dish_build_repeat_frame;
    dish_builder->parent.get_result = dish_builder_get_result;
    dish_builder->parent.del = dish_builder_del;
    dish_builder->parent.repeat_period_ms = 110;
    return &dish_builder->parent;
err:
    return ret;
}
