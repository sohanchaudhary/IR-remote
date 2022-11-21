//==============================================================================
// TTTTTTTTTTT   OOO     SSSS  H  H  IIIII  BBB     AAA         AAA    CCCCC
//     TT      OO   OO  S      H  H    I    B  B   A   A       A   A   C
//     TT     OO     OO  SSS   HHHH    I    BBB    AAAAA       AAAAA   C
//     TT      OO   OO      S  H  H    I    B  B   A   A       A   A   C
//     TT        OOO    SSSS   H  H  IIIII  BBB    A   A       A   A   CCCCC
//===============================================================================
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "toshibaAC_builder";
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

typedef struct {
    ir_builder_t parent;
    uint32_t buffer_size;
    uint32_t cursor;
    uint32_t flags;
    uint32_t leading_code_high_ticks;
    uint32_t leading_code_low_ticks;
    uint32_t message_high_ticks;
    uint32_t message_low_ticks;
    uint32_t payload_logic0_high_ticks;
    uint32_t payload_logic0_low_ticks;
    uint32_t payload_logic1_high_ticks;
    uint32_t payload_logic1_low_ticks;
    uint32_t ending_code_high_ticks;
    uint32_t ending_code_low_ticks;
    bool inverse;
    rmt_item32_t buffer[0];
} toshibaAC_builder_t;

static esp_err_t toshibaAC_builder_make_head(ir_builder_t *builder)
{  
    toshibaAC_builder_t *toshibaAC_builder = __containerof(builder, toshibaAC_builder_t, parent);
    
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].level0 = !toshibaAC_builder->inverse;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].duration0 = toshibaAC_builder->leading_code_high_ticks;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].level1 = toshibaAC_builder->inverse;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].duration1 = toshibaAC_builder->leading_code_low_ticks;
    toshibaAC_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t toshibaAC_builder_make_logic0(ir_builder_t *builder)
{
    toshibaAC_builder_t *toshibaAC_builder = __containerof(builder, toshibaAC_builder_t, parent);
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].level0 = !toshibaAC_builder->inverse;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].duration0 = toshibaAC_builder->payload_logic0_high_ticks;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].level1 = toshibaAC_builder->inverse;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].duration1 = toshibaAC_builder->payload_logic0_low_ticks;
    toshibaAC_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t toshibaAC_builder_make_logic1(ir_builder_t *builder)
{
    toshibaAC_builder_t *toshibaAC_builder = __containerof(builder, toshibaAC_builder_t, parent);
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].level0 = !toshibaAC_builder->inverse;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].duration0 = toshibaAC_builder->payload_logic1_high_ticks;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].level1 = toshibaAC_builder->inverse;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].duration1 = toshibaAC_builder->payload_logic1_low_ticks;
    toshibaAC_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t toshibaAC_build_make_message_space(ir_builder_t *builder)
{
    toshibaAC_builder_t *toshibaAC_builder = __containerof(builder, toshibaAC_builder_t, parent);
    //toshibaAC_builder->cursor = 0;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].level0 = !toshibaAC_builder->inverse;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].duration0 = toshibaAC_builder->message_high_ticks;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].level1 = toshibaAC_builder->inverse;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].duration1 = toshibaAC_builder->message_low_ticks;
    toshibaAC_builder->cursor += 1;
    //ESP_LOGI("INFO", "MESSAGE GAP Called");
    //toshibaAC_builder_make_end(builder);
    return ESP_OK;
}

static esp_err_t toshibaAC_builder_make_end(ir_builder_t *builder)
{
    toshibaAC_builder_t *toshibaAC_builder = __containerof(builder, toshibaAC_builder_t, parent);
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].level0 = !toshibaAC_builder->inverse;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].duration0 = toshibaAC_builder->ending_code_high_ticks;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].level1 = toshibaAC_builder->inverse;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].duration1 = toshibaAC_builder->ending_code_low_ticks;
    toshibaAC_builder->cursor += 1;
    toshibaAC_builder->buffer[toshibaAC_builder->cursor].val = 0;
    toshibaAC_builder->cursor += 1;
    //ESP_LOGI("INFO", "END Called");
    return ESP_OK;
}

static esp_err_t toshibaAC_build_frame(ir_builder_t *builder, uint32_t address, uint32_t command)
{
    toshibaAC_builder_t *toshibaAC_builder = __containerof(builder, toshibaAC_builder_t, parent);
    toshibaAC_builder->cursor = 0;
    builder->make_head(builder);
    // LSB -> MSB
    for (int i = 23; i >= 0; i--) {
        if (address & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    for (int i = 23; i >= 0; i--) {
        if (command & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    builder->make_message_space(builder);
    builder->make_head(builder);
    for (int i = 23; i >= 0; i--) {
        if (address & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    for (int i = 23; i >= 0; i--) {
        if (command & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    builder->make_end(builder);
    return ESP_OK;
}

static esp_err_t toshibaAC_build_frame_toshibaAC(ir_builder_t *builder, uint32_t address, uint32_t command, uint32_t checksum, uint32_t addr, uint32_t cmd, uint32_t cksum)
{
    //esp_err_t ret = ESP_OK;
    //uint32_t address1 = address;
   // uint32_t command1 = command;
    toshibaAC_builder_t *toshibaAC_builder = __containerof(builder, toshibaAC_builder_t, parent);
    toshibaAC_builder->cursor = 0;
    builder->make_head(builder);
    // LSB -> MSB
    for (int i = 31; i >= 0; i--) {
        if (address & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
   // builder->make_message_space(builder);
    for (int i = 31; i >= 0; i--) {
        if (command & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    for (int i = 7; i >= 0; i--) {
        if (checksum & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    builder->make_message_space(builder);
    builder->make_head(builder);
    for (int i = 31; i >= 0; i--) {
        if (addr & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    for (int i = 31; i >= 0; i--) {
        if (cmd & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    for (int i = 7; i >= 0; i--) {
        if (cksum & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    builder->make_end(builder);
    return ESP_OK;
//err:
    //return ret;
}

static esp_err_t toshibaAC_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    toshibaAC_builder_t *toshibaAC_builder = __containerof(builder, toshibaAC_builder_t, parent);
    TOSHIBAAC_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = toshibaAC_builder->buffer;
    *length = toshibaAC_builder->cursor;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t toshibaAC_builder_del(ir_builder_t *builder)
{
    toshibaAC_builder_t *toshibaAC_builder = __containerof(builder, toshibaAC_builder_t, parent);
    free(toshibaAC_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_toshibaAC(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    TOSHIBAAC_CHECK(config, "toshibaAC configuration can't be null", err, NULL);
    TOSHIBAAC_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(toshibaAC_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    toshibaAC_builder_t *toshibaAC_builder = calloc(1, builder_size);
    TOSHIBAAC_CHECK(toshibaAC_builder, "request memory for toshibaAC_builder failed", err, NULL);

    toshibaAC_builder->buffer_size = config->buffer_size;
    toshibaAC_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        toshibaAC_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    TOSHIBAAC_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    #if CONFIG_EXAMPLE_IR_PROTOCOL_TOSHIBAAC72
    toshibaAC_builder->leading_code_high_ticks = (uint32_t)(ratio * TOSHIBAAC_LEADING_CODE_HIGH_US);
    toshibaAC_builder->leading_code_low_ticks = (uint32_t)(ratio * TOSHIBAAC_LEADING_CODE_LOW_US);
    toshibaAC_builder->message_high_ticks = (uint32_t)(ratio * TOSHIBAAC_MESSAGE_SPACE_HIGH_US);
    toshibaAC_builder->message_low_ticks = (uint32_t)(ratio * TOSHIBAAC_MESSAGE_SPACE_LOW_US);
    toshibaAC_builder->payload_logic0_high_ticks = (uint32_t)(ratio * TOSHIBAAC_PAYLOAD_ZERO_HIGH_US);
    toshibaAC_builder->payload_logic0_low_ticks = (uint32_t)(ratio * TOSHIBAAC_PAYLOAD_ZERO_LOW_US);
    toshibaAC_builder->payload_logic1_high_ticks = (uint32_t)(ratio * TOSHIBAAC_PAYLOAD_ONE_HIGH_US);
    toshibaAC_builder->payload_logic1_low_ticks = (uint32_t)(ratio * TOSHIBAAC_PAYLOAD_ONE_LOW_US);
    toshibaAC_builder->ending_code_high_ticks = (uint32_t)(ratio * TOSHIBAAC_ENDING_CODE_HIGH_US);
    #else
    toshibaAC_builder->leading_code_high_ticks = (uint32_t)(ratio * TOSHIBAAC50_LEADING_CODE_HIGH_US);
    toshibaAC_builder->leading_code_low_ticks = (uint32_t)(ratio * TOSHIBAAC50_LEADING_CODE_LOW_US);
    toshibaAC_builder->message_high_ticks = (uint32_t)(ratio * TOSHIBAAC50_MESSAGE_SPACE_HIGH_US);
    toshibaAC_builder->message_low_ticks = (uint32_t)(ratio * TOSHIBAAC50_MESSAGE_SPACE_LOW_US);
    toshibaAC_builder->payload_logic0_high_ticks = (uint32_t)(ratio * TOSHIBAAC50_PAYLOAD_ZERO_HIGH_US);
    toshibaAC_builder->payload_logic0_low_ticks = (uint32_t)(ratio * TOSHIBAAC50_PAYLOAD_ZERO_LOW_US);
    toshibaAC_builder->payload_logic1_high_ticks = (uint32_t)(ratio * TOSHIBAAC50_PAYLOAD_ONE_HIGH_US);
    toshibaAC_builder->payload_logic1_low_ticks = (uint32_t)(ratio * TOSHIBAAC50_PAYLOAD_ONE_LOW_US);
    toshibaAC_builder->ending_code_high_ticks = (uint32_t)(ratio * TOSHIBAAC50_ENDING_CODE_HIGH_US);
    #endif
    toshibaAC_builder->ending_code_low_ticks = 0x7FFF;
    toshibaAC_builder->parent.make_head = toshibaAC_builder_make_head;
    toshibaAC_builder->parent.make_logic0 = toshibaAC_builder_make_logic0;
    toshibaAC_builder->parent.make_logic1 = toshibaAC_builder_make_logic1;
    toshibaAC_builder->parent.make_end = toshibaAC_builder_make_end;
    toshibaAC_builder->parent.build_frame_toshibaAC = toshibaAC_build_frame_toshibaAC;
    toshibaAC_builder->parent.build_frame = toshibaAC_build_frame;
    toshibaAC_builder->parent.make_message_space = toshibaAC_build_make_message_space;
    toshibaAC_builder->parent.get_result = toshibaAC_builder_get_result;
    toshibaAC_builder->parent.del = toshibaAC_builder_del;
    //toshibaAC_builder->parent.repeat_period_ms = 110;
    return &toshibaAC_builder->parent;
err:
    return ret;
}
