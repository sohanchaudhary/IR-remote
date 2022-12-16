#include <stdio.h>

// STRUCTURE Containing IR PROTOCOLS 

/**
 *  @brief IR PROTOCOLS CONTAINER TYPE
 * 
 */ 
typedef struct ir_remote_protocol_s ir_remote_protocols_t;

/**
 *  @brief IR PROTOCOLS CONTAINER TYPE
 * 
 */ 
typedef struct ir_remote_details_s ir_remote_details_t;


/** 
 * @brief   /// Enumerator for defining and numbering of supported IR protocol.
* @note     Always add to the end of the list and should never remove entries
*           or change order. Projects may save the type number for later usage
*           so numbering should always stay the same.
 * 
 */ 
enum decode_protocol_type_t {
    INDEX_REMOTE_NEC = 0,
    INDEX_REMOTE_RC5,
    INDEX_REMOTE_SAMSUNGTV,
    INDEX_REMOTE_LG,
    INDEX_REMOTE_SONY, 
    INDEX_REMOTE_GREE,  //5
    INDEX_REMOTE_PANASONIC,
    INDEX_REMOTE_DISH,
    INDEX_REMOTE_JVC,
    INDEX_REMOTE_LEGO,
    INDEX_REMOTE_SAMSUNGAC,  //10
    INDEX_REMOTE_TOSHIBAAC50,
    INDEX_REMOTE_TOSHIBAAC100,
    INDEX_REMOTE_TOSHIBATV,
    INDEX_REMOTE_EPSON,
    INDEX_REMOTE_AIRTON,  //15
    INDEX_REMOTE_AIWA
};

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
} protocol_timings_t;

// Message lengths & required repeat values 
const uint16_t NoRepeat = 0;
const uint16_t SingleRepeat = 1;
/***     AIRTON  ********/
const uint16_t AirtonBits = 58;  // Start and Stop Bit both included
const uint16_t AirtonDefaultRepeat = NoRepeat;
const char AirtonBitOrder = "LSB";
/***     AIWA    ********/
const uint16_t AiwaBits = 44;
const uint16_t AiwaDefaultRepeat = NoRepeat;
const char AiwaBitOrder = "LSB";
/***     TOSHIBATV    ********/
const uint16_t ToshibaTVBits = 34;
const uint16_t ToshibaTVDefaultRepeat = NoRepeat;
const char ToshibaTVBitOrder = "LSB";
/***     TOSHIBA AC 50 Bits    ********/
const uint16_t ToshibaAC50Bits = 100;
const uint16_t ToshibaAC50DefaultRepeat = SingleRepeat;
const char ToshibaAC50BitOrder = "MSB";
/***     TOSHIBA AC 72 Bits    ********/
const uint16_t ToshibaAC72Bits = 148;
const uint16_t ToshibaAC72DefaultRepeat = SingleRepeat;
const char ToshibaAC72BitOrder = "MSB";
/***     NEC     ********/
const uint16_t NecBits = 34;
const uint16_t NecDefaultRepeat = NoRepeat;
const char NecBitOrder = "LSB";
/***     EPSON    ********/
const uint16_t EpsonBits = 34;
const uint16_t EpsonDefaultRepeat = NoRepeat;
const char EpsonBitOrder = "LSB";
/***     LG     ********/
const uint16_t LgBits = 34;
const uint16_t LgDefaultRepeat = NoRepeat;
const char LgACBitOrder = "MSB";
const char LgTVBitOrder = "LSB";
/***     SONY / SIRCS    *******/
const uint16_t Sony12Bits = 13;
const uint16_t Sony15Bits = 16;
const uint16_t Sony20Bits = 21;
const uint16_t SonyDefaultRepeat = NoRepeat;
const char SonyBitOrder = "LSB";
/***     DISH      *******/
const uint16_t DishBits = 18;
const uint16_t DishDefaultRepeat = NoRepeat;
const char DishBitOrder = "LSB";
/****    PANASONIC     *******/
const uint16_t PanasonicBits = 58;
const uint16_t PanasonicDefaultRepeat = NoRepeat;
const char PanasonicBitOrder = "LSB";
/****    Lego     *******/
const uint16_t LegoBits = 18;
const uint16_t LegoDefaultRepeat = NoRepeat;
const char LegoBitOrder = "MSB";
/****    JVC     *******/
const uint16_t JvcBits = 18;
const uint16_t jvcDefaultRepeat = NoRepeat;
const char jvcBitOrder = "LSB";
/****    GREE     *******/
const uint16_t GreeBits = 70;
const uint16_t GreeDefaultRepeat = NoRepeat;
const char GreeBitOrder = "LSB";
/****    SAMSUNG     *******/
const uint16_t SamsungTVBits = 34;
const uint16_t SamsungACBits = 22;
const uint16_t SamsungDefaultRepeat = NoRepeat;
const char SamsungBitOrder = "LSB";