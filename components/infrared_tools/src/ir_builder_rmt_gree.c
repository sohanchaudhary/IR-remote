/// @file
/// @brief Support for Gree A/C protocols.
/// @see https://github.com/ToniA/arduino-heatpumpir/blob/master/GreeHeatpumpIR.h
/// @see https://github.com/crankyoldgit/IRremoteESP8266/issues/1508
/// @see https://github.com/crankyoldgit/IRremoteESP8266/issues/1821

// Supports:
//   Brand: Ultimate,  Model: Heat Pump
//   Brand: EKOKAI,  Model: A/C
//   Brand: RusClimate,  Model: EACS/I-09HAR_X/N3 A/C
//   Brand: RusClimate,  Model: YAW1F remote
//   Brand: Green,  Model: YBOFB remote
//   Brand: Green,  Model: YBOFB2 remote
//   Brand: Gree,  Model: YAA1FBF remote
//   Brand: Gree,  Model: YB1F2F remote
//   Brand: Gree,  Model: YAN1F1 remote
//   Brand: Gree,  Model: YX1F2F remote (YX1FSF)
//   Brand: Gree,  Model: VIR09HP115V1AH A/C
//   Brand: Gree,  Model: VIR12HP230V1AH A/C
//   Brand: Amana,  Model: PBC093G00CC A/C
//   Brand: Amana,  Model: YX1FF remote
//   Brand: Cooper & Hunter,  Model: YB1F2 remote
//   Brand: Cooper & Hunter,  Model: CH-S09FTXG A/C
//   Brand: Vailland,  Model: YACIFB remote
//   Brand: Vailland,  Model: VAI5-035WNI A/C
//   Brand: Soleus Air,  Model: window A/C (YX1FSF)

#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "gree_builder";
#define GREE_CHECK(a, str, goto_tag, ret_value, ...)                               \
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
    //uint32_t repeat_code_high_ticks;
    //uint32_t repeat_code_low_ticks;
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
} gree_builder_t;

static esp_err_t gree_builder_make_head(ir_builder_t *builder)
{
    gree_builder_t *gree_builder = __containerof(builder, gree_builder_t, parent);
    gree_builder->cursor = 0;
    gree_builder->buffer[gree_builder->cursor].level0 = !gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration0 = gree_builder->leading_code_high_ticks;
    gree_builder->buffer[gree_builder->cursor].level1 = gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration1 = gree_builder->leading_code_low_ticks;
    gree_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t gree_builder_make_logic0(ir_builder_t *builder)
{
    gree_builder_t *gree_builder = __containerof(builder, gree_builder_t, parent);
    gree_builder->buffer[gree_builder->cursor].level0 = !gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration0 = gree_builder->payload_logic0_high_ticks;
    gree_builder->buffer[gree_builder->cursor].level1 = gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration1 = gree_builder->payload_logic0_low_ticks;
    gree_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t gree_builder_make_logic1(ir_builder_t *builder)
{
    gree_builder_t *gree_builder = __containerof(builder, gree_builder_t, parent);
    gree_builder->buffer[gree_builder->cursor].level0 = !gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration0 = gree_builder->payload_logic1_high_ticks;
    gree_builder->buffer[gree_builder->cursor].level1 = gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration1 = gree_builder->payload_logic1_low_ticks;
    gree_builder->cursor += 1;
    return ESP_OK;
}

/******** To Make Footer BITS 010 *****************/
static esp_err_t gree_builder_make_footer(ir_builder_t *builder)
{
    //ESP_LOGI("INFO", "Footer function called");
    //gree_builder_t *gree_builder = __containerof(builder, gree_builder_t, parent);
    builder->make_logic0(builder);
    builder->make_logic1(builder);
    builder->make_logic0(builder);
    //gree_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t gree_builder_make_message_space(ir_builder_t *builder)
{
    //ESP_LOGI("INFO", "Message SPACE function called");
    gree_builder_t *gree_builder = __containerof(builder, gree_builder_t, parent);
    gree_builder->buffer[gree_builder->cursor].level0 = !gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration0 = gree_builder->message_high_ticks;
    gree_builder->buffer[gree_builder->cursor].level1 = gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration1 = gree_builder->message_low_ticks;
    gree_builder->cursor += 1;
    //ESP_LOGI("INFO", "Message SPACE2 function called");
    return ESP_OK;
}

static esp_err_t gree_builder_make_end(ir_builder_t *builder)
{
    //ESP_LOGI("INFO", "END function called");
    gree_builder_t *gree_builder = __containerof(builder, gree_builder_t, parent);
    gree_builder->buffer[gree_builder->cursor].level0 = !gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration0 = gree_builder->ending_code_high_ticks;
    gree_builder->buffer[gree_builder->cursor].level1 = gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration1 = gree_builder->ending_code_low_ticks;
    gree_builder->cursor += 1;
    gree_builder->buffer[gree_builder->cursor].val = 0;
    gree_builder->cursor += 1;
   // ESP_LOGI("INFO", "END function called");
    return ESP_OK;
}


static esp_err_t gree_build_frame(ir_builder_t *builder, uint32_t addr, uint32_t cmd)
{
    //esp_err_t ret = ESP_OK;
    gree_builder_t *gree_builder = __containerof(builder, gree_builder_t, parent);
    
     /*
   if (!gree_builder->flags & IR_TOOLS_FLAGS_PROTO_EXT) {
        uint8_t low_byte = data0 & 0xFF;
        uint8_t high_byte = (data0 >> 8) & 0xFF;
        GREE_CHECK(low_byte == (~high_byte & 0xFF), "data0 not match standard GREE protocol", err, ESP_ERR_INVALID_ARG);
        low_byte = data1 & 0xFF;
        high_byte = (data1 >> 8) & 0xFF;
        GREE_CHECK(low_byte == (~high_byte & 0xFF), "data1 not match standard GREE protocol", err, ESP_ERR_INVALID_ARG);
    }
*/

    builder->make_head(builder);
    // LSB -> MSB  data0
    for (int i = 0; i < 32; i++) {
        if (addr & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
       // ESP_LOGI("BUFFER INFO", "%d", i);
    }
    
    builder->make_footer(builder);
    builder->make_message_space(builder);
    //ESP_LOGI("INFO", "Data1 left to encode");
    //data1
    for (int i = 0; i < 32; i++) {
        if (cmd & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    builder->make_end(builder);
    return ESP_OK;
// err:
//    return ret;
}

/*
static esp_err_t gree_build_repeat_frame(ir_builder_t *builder)
{
    gree_builder_t *gree_builder = __containerof(builder, gree_builder_t, parent);
    gree_builder->cursor = 0;
    gree_builder->buffer[gree_builder->cursor].level0 = !gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration0 = gree_builder->repeat_code_high_ticks;
    gree_builder->buffer[gree_builder->cursor].level1 = gree_builder->inverse;
    gree_builder->buffer[gree_builder->cursor].duration1 = gree_builder->repeat_code_low_ticks;
    gree_builder->cursor += 1;
    gree_builder_make_end(builder);
    return ESP_OK;
}
*/

static esp_err_t gree_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    gree_builder_t *gree_builder = __containerof(builder, gree_builder_t, parent);
    GREE_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = gree_builder->buffer;
    *length = gree_builder->cursor;
    return ESP_OK;
err:
    return ret;
    ESP_LOGE("ERROR", "There is error while checking result");
}

static esp_err_t gree_builder_del(ir_builder_t *builder)
{
    gree_builder_t *gree_builder = __containerof(builder, gree_builder_t, parent);
    free(gree_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_gree(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    GREE_CHECK(config, "gree configuration can't be null", err, NULL);
    GREE_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(gree_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    gree_builder_t *gree_builder = calloc(1, builder_size);
    GREE_CHECK(gree_builder, "request memory for gree_builder failed", err, NULL);

    gree_builder->buffer_size = config->buffer_size;
    gree_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        gree_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    GREE_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    gree_builder->leading_code_high_ticks = (uint32_t)(ratio * GREE_LEADING_CODE_HIGH_US);
    gree_builder->leading_code_low_ticks = (uint32_t)(ratio * GREE_LEADING_CODE_LOW_US);
    gree_builder->message_high_ticks = (uint32_t)(ratio * GREE_MESSAGE_SPACE_HIGH_US);
    gree_builder->message_low_ticks = (uint32_t)(ratio * GREE_MESSAGE_SPACE_LOW_US);
    gree_builder->payload_logic0_high_ticks = (uint32_t)(ratio * GREE_PAYLOAD_ZERO_HIGH_US);
    gree_builder->payload_logic0_low_ticks = (uint32_t)(ratio * GREE_PAYLOAD_ZERO_LOW_US);
    gree_builder->payload_logic1_high_ticks = (uint32_t)(ratio * GREE_PAYLOAD_ONE_HIGH_US);
    gree_builder->payload_logic1_low_ticks = (uint32_t)(ratio * GREE_PAYLOAD_ONE_LOW_US);
    gree_builder->ending_code_high_ticks = (uint32_t)(ratio * GREE_ENDING_CODE_HIGH_US);
    gree_builder->ending_code_low_ticks = 0x7FFF;
    gree_builder->parent.make_head = gree_builder_make_head;
    gree_builder->parent.make_logic0 = gree_builder_make_logic0;
    gree_builder->parent.make_logic1 = gree_builder_make_logic1;
    gree_builder->parent.make_end = gree_builder_make_end;
    gree_builder->parent.build_frame = gree_build_frame;
    gree_builder->parent.make_footer = gree_builder_make_footer;
    gree_builder->parent.make_message_space = gree_builder_make_message_space;
   // gree_builder->parent.build_repeat_frame = gree_build_repeat_frame;
    gree_builder->parent.get_result = gree_builder_get_result;
    gree_builder->parent.del = gree_builder_del;
    //gree_builder->parent.repeat_period_ms = 110;
    return &gree_builder->parent;
err:
    return ret;
}
