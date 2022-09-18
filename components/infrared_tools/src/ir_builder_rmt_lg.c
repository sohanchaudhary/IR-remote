//==============================================================================
//                               L       GGGG
//                               L      G
//                               L      G  GG
//                               L      G   G
//                               LLLLL   GGG
//==============================================================================
// LG originally added by Darryl Smith (based on the JVC protocol)
// see: https://github.com/Arduino-IRremote/Arduino-IRremote/tree/master/examples/LGAirConditionerSendDemo
// see: https://www.mikrocontroller.net/articles/IRMP_-_english#LGAIR
// MSB first, 1 start bit + 8 bit address + 16 bit command + 4 bit checksum + 1 stop bit (28 data bits).
// Bit and repeat timing is like NEC
// LG2 has different header timing and a shorter bit time
/*
 * LG remote IR-LED measurements: Type AKB 73315611 for air conditioner, Ver1.1 from 2011.03.01
 * Protocol: LG2
 * Internal crystal: 4 MHz
 * Header:  8.9 ms mark 4.15 ms space
 * Data:    500 / 540 and 500 / 1580;
 * Clock is not synchronized with gate so you have 19 and sometimes 19 and a spike pulses for mark
 * Duty:    9 us on 17 us off => around 33 % duty
 * NO REPEAT: If value like temperature has changed during long press, the last value is send at button release.
 * If you do a double press, the next value can be sent after around 118 ms. Tested with the fan button.

 * LG remote IR-LED measurements: Type AKB 75095308 for LG TV
 * Protocol: NEC!!!
 * Frequency 37.88 kHz
 * Header:  9.0 ms mark 4.5 ms space
 * Data:    560 / 560 and 560 / 1680;
 * Clock is synchronized with gate, mark always starts with a full period
 * Duty:    13 us on 13 us off => 50 % duty
 * Repeat:  110 ms 9.0 ms mark, 2250 us space, 560 stop
 * LSB first!
 *
 * 
 */

#include <sys/cdefs.h>
#include "esp_log.h"
#include "ir_tools.h"
#include "ir_timings.h"
#include "driver/rmt.h"

static const char *TAG = "lgac_builder";
#define LGAC_CHECK(a, str, goto_tag, ret_value, ...)                               \
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
} lgac_builder_t;

static esp_err_t lgac_builder_make_head(ir_builder_t *builder)
{
    lgac_builder_t *lgac_builder = __containerof(builder, lgac_builder_t, parent);
    lgac_builder->cursor = 0;
    lgac_builder->buffer[lgac_builder->cursor].level0 = !lgac_builder->inverse;
    lgac_builder->buffer[lgac_builder->cursor].duration0 = lgac_builder->leading_code_high_ticks;
    lgac_builder->buffer[lgac_builder->cursor].level1 = lgac_builder->inverse;
    lgac_builder->buffer[lgac_builder->cursor].duration1 = lgac_builder->leading_code_low_ticks;
    lgac_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t lgac_builder_make_logic0(ir_builder_t *builder)
{
    lgac_builder_t *lgac_builder = __containerof(builder, lgac_builder_t, parent);
    lgac_builder->buffer[lgac_builder->cursor].level0 = !lgac_builder->inverse;
    lgac_builder->buffer[lgac_builder->cursor].duration0 = lgac_builder->payload_logic0_high_ticks;
    lgac_builder->buffer[lgac_builder->cursor].level1 = lgac_builder->inverse;
    lgac_builder->buffer[lgac_builder->cursor].duration1 = lgac_builder->payload_logic0_low_ticks;
    lgac_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t lgac_builder_make_logic1(ir_builder_t *builder)
{
    lgac_builder_t *lgac_builder = __containerof(builder, lgac_builder_t, parent);
    lgac_builder->buffer[lgac_builder->cursor].level0 = !lgac_builder->inverse;
    lgac_builder->buffer[lgac_builder->cursor].duration0 = lgac_builder->payload_logic1_high_ticks;
    lgac_builder->buffer[lgac_builder->cursor].level1 = lgac_builder->inverse;
    lgac_builder->buffer[lgac_builder->cursor].duration1 = lgac_builder->payload_logic1_low_ticks;
    lgac_builder->cursor += 1;
    return ESP_OK;
}

static esp_err_t lgac_builder_make_end(ir_builder_t *builder)
{
    lgac_builder_t *lgac_builder = __containerof(builder, lgac_builder_t, parent);
    lgac_builder->buffer[lgac_builder->cursor].level0 = !lgac_builder->inverse;
    lgac_builder->buffer[lgac_builder->cursor].duration0 = lgac_builder->ending_code_high_ticks;
    lgac_builder->buffer[lgac_builder->cursor].level1 = lgac_builder->inverse;
    lgac_builder->buffer[lgac_builder->cursor].duration1 = lgac_builder->ending_code_low_ticks;
    lgac_builder->cursor += 1;
    lgac_builder->buffer[lgac_builder->cursor].val = 0;
    lgac_builder->cursor += 1;
    return ESP_OK;
}

//Funcion for building frame of LGAC protocol

static esp_err_t lgac_build_frame(ir_builder_t *builder, uint32_t address, uint32_t command) //, uint8_t checksum
{
    esp_err_t ret = ESP_OK;
    lgac_builder_t *lgac_builder = __containerof(builder, lgac_builder_t, parent);
    if (!lgac_builder->flags & IR_TOOLS_FLAGS_PROTO_EXT) {
        uint8_t low_byte = address & 0xFF;
        //uint8_t high_byte = (address >> 8) & 0xFF;
        //uint8_t high_byte = command & 0xFF;
        //LGAC_CHECK(low_byte == (~high_byte & 0xFF), "address not match standard LGAC protocol", err, ESP_ERR_INVALID_ARG);
        LGAC_CHECK(low_byte == (low_byte & 0xFF), "address not match standard LGAC protocol", err, ESP_ERR_INVALID_ARG);
        low_byte = command & 0xFF;
        //LGAC_CHECK(low_byte == (high_byte & 0xFF), "command not match standard LGAC protocol", err, ESP_ERR_INVALID_ARG);
        uint8_t high_byte = (command >> 8) & 0xFF;;
        LGAC_CHECK(low_byte == (high_byte & 0xFF), "command not match standard LGAC protocol", err, ESP_ERR_INVALID_ARG);
    }
    builder->make_head(builder);
    // LSB -> MSB
    // MSB -> LSB
    for (int i = 15; i > 0; i--) {
        if (address & (1 << i)) {
            builder->make_logic1(builder);
        } else {
            builder->make_logic0(builder);
        }
    }
    for (int i = 15; i > 0; i--) {
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

static esp_err_t lgac_build_repeat_frame(ir_builder_t *builder)
{
    lgac_builder_t *lgac_builder = __containerof(builder, lgac_builder_t, parent);
    lgac_builder->cursor = 0;
    lgac_builder->buffer[lgac_builder->cursor].level0 = !lgac_builder->inverse;
    lgac_builder->buffer[lgac_builder->cursor].duration0 = lgac_builder->repeat_code_high_ticks;
    lgac_builder->buffer[lgac_builder->cursor].level1 = lgac_builder->inverse;
    lgac_builder->buffer[lgac_builder->cursor].duration1 = lgac_builder->repeat_code_low_ticks;
    lgac_builder->cursor += 1;
    lgac_builder_make_end(builder);
    return ESP_OK;
}

static esp_err_t lgac_builder_get_result(ir_builder_t *builder, void *result, size_t *length)
{
    esp_err_t ret = ESP_OK;
    lgac_builder_t *lgac_builder = __containerof(builder, lgac_builder_t, parent);
    LGAC_CHECK(result && length, "result and length can't be null", err, ESP_ERR_INVALID_ARG);
    *(rmt_item32_t **)result = lgac_builder->buffer;
    *length = lgac_builder->cursor;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t lgac_builder_del(ir_builder_t *builder)
{
    lgac_builder_t *lgac_builder = __containerof(builder, lgac_builder_t, parent);
    free(lgac_builder);
    return ESP_OK;
}

ir_builder_t *ir_builder_rmt_new_lgac(const ir_builder_config_t *config)
{
    ir_builder_t *ret = NULL;
    LGAC_CHECK(config, "lgac configuration can't be null", err, NULL);
    LGAC_CHECK(config->buffer_size, "buffer size can't be zero", err, NULL);

    uint32_t builder_size = sizeof(lgac_builder_t) + config->buffer_size * sizeof(rmt_item32_t);
    lgac_builder_t *lgac_builder = calloc(1, builder_size);
    LGAC_CHECK(lgac_builder, "request memory for lgac_builder failed", err, NULL);

    lgac_builder->buffer_size = config->buffer_size;
    lgac_builder->flags = config->flags;
    if (config->flags & IR_TOOLS_FLAGS_INVERSE) {
        lgac_builder->inverse = true;
    }

    uint32_t counter_clk_hz = 0;
    LGAC_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev_hdl, &counter_clk_hz) == ESP_OK,
              "get rmt counter clock failed", err, NULL);
    float ratio = (float)counter_clk_hz / 1e6;
    lgac_builder->leading_code_high_ticks = (uint32_t)(ratio * LGAC_LEADING_CODE_HIGH_US);
    lgac_builder->leading_code_low_ticks = (uint32_t)(ratio * LGAC_LEADING_CODE_LOW_US);
    lgac_builder->repeat_code_high_ticks = (uint32_t)(ratio * LGAC_REPEAT_CODE_HIGH_US);
    lgac_builder->repeat_code_low_ticks = (uint32_t)(ratio * LGAC_REPEAT_CODE_LOW_US);
    lgac_builder->payload_logic0_high_ticks = (uint32_t)(ratio * LGAC_PAYLOAD_ZERO_HIGH_US);
    lgac_builder->payload_logic0_low_ticks = (uint32_t)(ratio * LGAC_PAYLOAD_ZERO_LOW_US);
    lgac_builder->payload_logic1_high_ticks = (uint32_t)(ratio * LGAC_PAYLOAD_ONE_HIGH_US);
    lgac_builder->payload_logic1_low_ticks = (uint32_t)(ratio * LGAC_PAYLOAD_ONE_LOW_US);
    lgac_builder->ending_code_high_ticks = (uint32_t)(ratio * LGAC_ENDING_CODE_HIGH_US);
    lgac_builder->ending_code_low_ticks = 0x7FFF;
    lgac_builder->parent.make_head = lgac_builder_make_head;
    lgac_builder->parent.make_logic0 = lgac_builder_make_logic0;
    lgac_builder->parent.make_logic1 = lgac_builder_make_logic1;
    lgac_builder->parent.make_end = lgac_builder_make_end;
    lgac_builder->parent.build_frame = lgac_build_frame;
    lgac_builder->parent.build_repeat_frame = lgac_build_repeat_frame;
    lgac_builder->parent.get_result = lgac_builder_get_result;
    lgac_builder->parent.del = lgac_builder_del;
    lgac_builder->parent.repeat_period_ms = 110;
    return &lgac_builder->parent;
err:
    return ret;
}
