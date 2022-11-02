/** \addtogroup Decoder Decoders and encoders for different protocols
 * @{
 */
//==============================================================================
//         L       EEEEEE   EEEE    OOOO
//         L       E       E       O    O
//         L       EEEE    E  EEE  O    O
//         L       E       E    E  O    O
//         LLLLLL  EEEEEE   EEEE    OOOO
//==============================================================================
// from LEGO Power Functions RC Manual 26.02.2010 Version 1.20
// https://github.com/jurriaan/Arduino-PowerFunctions/raw/master/LEGO_Power_Functions_RC_v120.pdf
// https://oberguru.net/elektronik/ir/codes/lego_power_functions_train.lircd.conf
//
// To ensure correct detection of IR messages six 38 kHz cycles are transmitted as mark.
// Low bit consists of 6 cycles of IR and 10 “cycles” of pause,
// high bit of 6 cycles IR and 21 “cycles” of pause and start bit of 6 cycles IR and 39 “cycles” of pause.
// Low bit range 316 - 526 us
// High bit range 526 – 947 us
// Start/stop bit range 947 – 1579 us
// If tm is the maximum message length (16ms) and Ch is the channel number, then
// The delay before transmitting the first message is: (4 – Ch)*tm
// The time from start to start for the next 2 messages is: 5*tm
// The time from start to start for the following messages is: (6 + 2*Ch)*tm
// Supported Devices
// LEGO Power Functions IR Receiver 8884
// MSB first, 1 start bit + 4 bit channel, 4 bit mode + 4 bit command + 4 bit parity + 1 stop bit.
#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "lego_builder";
#define LEGO_CHECK(a, str, goto_tag, ret_value, ...)                               \
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
} lego_builder_t;

static esp_err_t lego_builder_make_head(ir_builder_t *builder)
{
    lego_builder_t *lego_builder = __containerof(builder, lego_builder_t, parent);
    lego_builder->cursor = 0;
    lego_builder->buffer[lego_builder->cursor].level0 = !lego_builder->inverse;
    lego_builder->buffer[lego_builder->cursor].duration0 = lego_builder->leading_code_high_ticks;
    lego_builder->buffer[lego_builder->cursor].level1 = lego_builder->inverse;
    lego_builder->buffer[lego_builder->cursor].duration1 = lego_builder->leading_code_low_ticks;
    lego_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t lego_builder_make_logic0(ir_builder_t *builder)
{
    lego_builder_t *lego_builder = __containerof(builder, lego_builder_t, parent);
    lego_builder->buffer[lego_builder->cursor].level0 = !lego_builder->inverse;
    lego_builder->buffer[lego_builder->cursor].duration0 = lego_builder->payload_logic0_high_ticks;
    lego_builder->buffer[lego_builder->cursor].level1 = lego_builder->inverse;
    lego_builder->buffer[lego_builder->cursor].duration1 = lego_builder->payload_logic0_low_ticks;
    lego_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t lego_builder_make_logic1(ir_builder_t *builder)
{
    lego_builder_t *lego_builder = __containerof(builder, lego_builder_t, parent);
    lego_builder->buffer[lego_builder->cursor].level0 = !lego_builder->inverse;
    lego_builder->buffer[lego_builder->cursor].duration0 = lego_builder->payload_logic1_high_ticks;
    lego_builder->buffer[lego_builder->cursor].level1 = lego_builder->inverse;
    lego_builder->buffer[lego_builder->cursor].duration1 = lego_builder->payload_logic1_low_ticks;
    lego_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t lego_builder_make_end(ir_builder_t *builder)
{
    lego_builder_t *lego_builder = __containerof(builder, lego_builder_t, parent);
    lego_builder->buffer[lego_builder->cursor].level0 = !lego_builder->inverse;
    lego_builder->buffer[lego_builder->cursor].duration0 = lego_builder->ending_code_high_ticks;
    lego_builder->buffer[lego_builder->cursor].level1 = lego_builder->inverse;
    lego_builder->buffer[lego_builder->cursor].duration1 = lego_builder->ending_code_low_ticks;
    lego_builder->cursor += 1;
    lego_builder->buffer[lego_builder->cursor].val = 0;
    lego_builder->cursor += 1;
    return ESP_OK;
}


static esp_err_t lego_build_frame(ir_builder_t *builder, uint32_t address, uint32_t command)
{
    esp_err_t ret = ESP_OK;
    lego_builder_t *lego_builder = __containerof(builder, lego_builder_t, parent);
    
    builder->make_head(builder);
    // MSB -> LSB
    for (int i = 11; i >= 0; i--) {
        if (command & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    for (int i = 3; i >= 0; i--) {
        if (address & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    builder->make_end(builder);
    return ESP_OK;
//err:
    return ret;
}
/*
static esp_err_t lego_build_repeat_frame(ir_builder_t *builder)
{
    lego_builder_t *lego_builder = __containerof(builder, lego_builder_t, parent);
    lego_builder->cursor = 0;
    lego_builder->buffer[lego_builder->cursor].level0 = !lego_builder->inverse;
    lego_builder->buffer[lego_builder->cursor].duration0 = lego_builder->repeat_code_high_ticks;
    lego_builder->buffer[lego_builder->cursor].level1 = lego_builder->inverse;
    lego_builder->buffer[lego_builder->cursor].duration1 = lego_builder->repeat_code_low_ticks;
    lego_builder->cursor += 1;
    lego_builder_make_end(builder);
    return ESP_OK;
}
*/
static esp_err_t lego_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    lego_builder_t *lego_builder = __containerof(builder, lego_builder_t, parent);
    LEGO_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = lego_builder->buffer;
    *length = lego_builder->cursor;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t lego_builder_del(ir_builder_t *builder)
{
    lego_builder_t *lego_builder = __containerof(builder, lego_builder_t, parent);
    free(lego_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_lego(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    LEGO_CHECK(config, "lego configuration can't be null", err, NULL);
    LEGO_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(lego_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    lego_builder_t *lego_builder = calloc(1, builder_size);
    LEGO_CHECK(lego_builder, "request memory for lego_builder failed", err, NULL);

    lego_builder->buffer_size = config->buffer_size;
    lego_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        lego_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    LEGO_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    lego_builder->leading_code_high_ticks = (uint32_t)(ratio * LEGO_LEADING_CODE_HIGH_US);
    lego_builder->leading_code_low_ticks = (uint32_t)(ratio * LEGO_LEADING_CODE_LOW_US);
    //lego_builder->repeat_code_high_ticks = (uint32_t)(ratio * LEGO_REPEAT_CODE_HIGH_US);
    //lego_builder->repeat_code_low_ticks = (uint32_t)(ratio * LEGO_REPEAT_CODE_LOW_US);
    lego_builder->payload_logic0_high_ticks = (uint32_t)(ratio * LEGO_PAYLOAD_ZERO_HIGH_US);
    lego_builder->payload_logic0_low_ticks = (uint32_t)(ratio * LEGO_PAYLOAD_ZERO_LOW_US);
    lego_builder->payload_logic1_high_ticks = (uint32_t)(ratio * LEGO_PAYLOAD_ONE_HIGH_US);
    lego_builder->payload_logic1_low_ticks = (uint32_t)(ratio * LEGO_PAYLOAD_ONE_LOW_US);
    lego_builder->ending_code_high_ticks = (uint32_t)(ratio * LEGO_ENDING_CODE_HIGH_US);
    lego_builder->ending_code_low_ticks = 0x7FFF;
    lego_builder->parent.make_head = lego_builder_make_head;
    lego_builder->parent.make_logic0 = lego_builder_make_logic0;
    lego_builder->parent.make_logic1 = lego_builder_make_logic1;
    lego_builder->parent.make_end = lego_builder_make_end;
    lego_builder->parent.build_frame = lego_build_frame;
    //lego_builder->parent.build_repeat_frame = lego_build_repeat_frame;
    lego_builder->parent.get_result = lego_builder_get_result;
    lego_builder->parent.del = lego_builder_del;
    //lego_builder->parent.repeat_period_ms = 110;
    return &lego_builder->parent;
err:
    return ret;
}
