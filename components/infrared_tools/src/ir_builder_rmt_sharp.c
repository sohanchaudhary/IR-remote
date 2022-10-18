#include <sys/cdefs.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "sharp_builder";
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

typedef struct {
    ir_builder_t parent;
    uint32_t buffer_size;
    uint32_t cursor;
    uint32_t flags;
    //uint32_t msggap_high_ticks;
    uint32_t msggap_low_ticks;
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
} sharp_builder_t;


static esp_err_t sharp_builder_make_spacegap(ir_builder_t *builder)
{
    sharp_builder_t *sharp_builder = __containerof(builder, sharp_builder_t, parent);
    //sharp_builder->cursor = 0;
    //sharp_builder->buffer[sharp_builder->cursor].level0 = !sharp_builder->inverse;
    //sharp_builder->buffer[sharp_builder->cursor].duration0 = sharp_builder->leading_code_high_ticks;
    sharp_builder->buffer[sharp_builder->cursor].level1 = sharp_builder->inverse;
    sharp_builder->buffer[sharp_builder->cursor].duration1 = sharp_builder->msggap_low_ticks;
    sharp_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t sharp_builder_make_logic0(ir_builder_t *builder)
{
    sharp_builder_t *sharp_builder = __containerof(builder, sharp_builder_t, parent);
    sharp_builder->buffer[sharp_builder->cursor].level0 = !sharp_builder->inverse;
    sharp_builder->buffer[sharp_builder->cursor].duration0 = sharp_builder->payload_logic0_high_ticks;
    sharp_builder->buffer[sharp_builder->cursor].level1 = sharp_builder->inverse;
    sharp_builder->buffer[sharp_builder->cursor].duration1 = sharp_builder->payload_logic0_low_ticks;
    sharp_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t sharp_builder_make_logic1(ir_builder_t *builder)
{
    sharp_builder_t *sharp_builder = __containerof(builder, sharp_builder_t, parent);
    sharp_builder->buffer[sharp_builder->cursor].level0 = !sharp_builder->inverse;
    sharp_builder->buffer[sharp_builder->cursor].duration0 = sharp_builder->payload_logic1_high_ticks;
    sharp_builder->buffer[sharp_builder->cursor].level1 = sharp_builder->inverse;
    sharp_builder->buffer[sharp_builder->cursor].duration1 = sharp_builder->payload_logic1_low_ticks;
    sharp_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t sharp_builder_make_end(ir_builder_t *builder)
{
    sharp_builder_t *sharp_builder = __containerof(builder, sharp_builder_t, parent);
    sharp_builder->buffer[sharp_builder->cursor].level0 = !sharp_builder->inverse;
    sharp_builder->buffer[sharp_builder->cursor].duration0 = sharp_builder->ending_code_high_ticks;
    sharp_builder->buffer[sharp_builder->cursor].level1 = sharp_builder->inverse;
    sharp_builder->buffer[sharp_builder->cursor].duration1 = sharp_builder->ending_code_low_ticks;
    sharp_builder->cursor += 1;
    sharp_builder->buffer[sharp_builder->cursor].val = 0;
    sharp_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t sharp_build_frame(ir_builder_t *builder, uint32_t address, uint32_t command)
{
    //esp_err_t ret = ESP_OK;
    sharp_builder_t *sharp_builder = __containerof(builder, sharp_builder_t, parent);
    sharp_builder->cursor = 0;
   // builder->make_head(builder);
    // LSB -> MSB
    for (int j = 0; j<2; j++) {
        for (int i = 0; i < 5; i++) {
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
        if(j==0){
            builder->make_logic1(builder);  //expansion bit
            builder->make_logic0(builder);  //check bit 
        }
        else {
            builder->make_logic0(builder);  //expansion bit
            builder->make_logic1(builder);  //check bit       
        }
        
        builder->make_end(builder);
        ESP_LOGI("INFO", "This is %d time called", j);
        sharp_builder_make_spacegap(builder);
        command ^= 0xFF;
        ESP_LOGI("INFO", "This is %d time called", sharp_builder->cursor);
    }

    ESP_LOGI("INFO", "FRAME BUILDED");
    return ESP_OK;
}

static esp_err_t sharp_build_repeat_frame(ir_builder_t *builder)
{
    sharp_builder_t *sharp_builder = __containerof(builder, sharp_builder_t, parent);
    sharp_builder->cursor = 0;
    sharp_builder->buffer[sharp_builder->cursor].level0 = !sharp_builder->inverse;
    sharp_builder->buffer[sharp_builder->cursor].duration0 = sharp_builder->repeat_code_high_ticks;
    sharp_builder->buffer[sharp_builder->cursor].level1 = sharp_builder->inverse;
    sharp_builder->buffer[sharp_builder->cursor].duration1 = sharp_builder->repeat_code_low_ticks;
    sharp_builder->cursor += 1;
    sharp_builder_make_end(builder);
    return ESP_OK;
}

static esp_err_t sharp_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    sharp_builder_t *sharp_builder = __containerof(builder, sharp_builder_t, parent);
    SHARP_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = sharp_builder->buffer;
    *length = sharp_builder->cursor;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t sharp_builder_del(ir_builder_t *builder)
{
    sharp_builder_t *sharp_builder = __containerof(builder, sharp_builder_t, parent);
    free(sharp_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_sharp(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    SHARP_CHECK(config, "sharp configuration can't be null", err, NULL);
    SHARP_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(sharp_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    sharp_builder_t *sharp_builder = calloc(1, builder_size);
    SHARP_CHECK(sharp_builder, "request memory for sharp_builder failed", err, NULL);

    sharp_builder->buffer_size = config->buffer_size;
    sharp_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        sharp_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    SHARP_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    //sharp_builder->leading_code_high_ticks = (uint32_t)(ratio * SHARP_LEADING_CODE_HIGH_US);
    sharp_builder->msggap_low_ticks = (uint32_t)(ratio * SHARP_DELAY_LOW_US);
    sharp_builder->repeat_code_high_ticks = (uint32_t)(ratio * SHARP_REPEAT_CODE_HIGH_US);
    sharp_builder->repeat_code_low_ticks = (uint32_t)(ratio * SHARP_REPEAT_CODE_LOW_US);
    sharp_builder->payload_logic0_high_ticks = (uint32_t)(ratio * SHARP_PAYLOAD_ZERO_HIGH_US);
    sharp_builder->payload_logic0_low_ticks = (uint32_t)(ratio * SHARP_PAYLOAD_ZERO_LOW_US);
    sharp_builder->payload_logic1_high_ticks = (uint32_t)(ratio * SHARP_PAYLOAD_ONE_HIGH_US);
    sharp_builder->payload_logic1_low_ticks = (uint32_t)(ratio * SHARP_PAYLOAD_ONE_LOW_US);
    sharp_builder->ending_code_high_ticks = (uint32_t)(ratio * SHARP_ENDING_CODE_HIGH_US);
    sharp_builder->ending_code_low_ticks = 0x7FFF;
    //sharp_builder->parent.make_head = sharp_builder_make_head;
    sharp_builder->parent.make_logic0 = sharp_builder_make_logic0;
    sharp_builder->parent.make_logic1 = sharp_builder_make_logic1;
    sharp_builder->parent.make_end = sharp_builder_make_end;
    sharp_builder->parent.build_frame = sharp_build_frame;
    sharp_builder->parent.build_repeat_frame = sharp_build_repeat_frame;
    sharp_builder->parent.get_result = sharp_builder_get_result;
    sharp_builder->parent.del = sharp_builder_del;
    sharp_builder->parent.repeat_period_ms = 110;
    return &sharp_builder->parent;
err:
    return ret;
}
