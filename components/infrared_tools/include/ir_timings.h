// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Timings for NEC protocol
 *
 */
#define NEC_LEADING_CODE_HIGH_US (9000)
#define NEC_LEADING_CODE_LOW_US (4500)
#define NEC_PAYLOAD_ONE_HIGH_US (560)
#define NEC_PAYLOAD_ONE_LOW_US (1690)
#define NEC_PAYLOAD_ZERO_HIGH_US (560)
#define NEC_PAYLOAD_ZERO_LOW_US (560)
#define NEC_REPEAT_CODE_HIGH_US (9000)
#define NEC_REPEAT_CODE_LOW_US (2250)
#define NEC_ENDING_CODE_HIGH_US (560)

/**
 * @brief Timings for RC5 protocol
 *
 */
#define RC5_PULSE_DURATION_US (889)

/**
 * @brief Timings for SAMSUNG protocol
 *
 */
#define SAMSUNG_LEADING_CODE_HIGH_US (4500)
#define SAMSUNG_LEADING_CODE_LOW_US (4500)
#define SAMSUNG_PAYLOAD_ONE_HIGH_US (560)
#define SAMSUNG_PAYLOAD_ONE_LOW_US (1690)
#define SAMSUNG_PAYLOAD_ZERO_HIGH_US (560)  //590
#define SAMSUNG_PAYLOAD_ZERO_LOW_US (560)
#define SAMSUNG_REPEAT_CODE_HIGH_US (4500)
#define SAMSUNG_REPEAT_CODE_LOW_US (2250)
#define SAMSUNG_ENDING_CODE_HIGH_US (560)
//#define SAMSUNG_ENDING_CODE_LOW_US (560)

/**
 * @brief Timings for LGAC protocol
 *
 */
#define LGAC_LEADING_CODE_HIGH_US (9000)
#define LGAC_LEADING_CODE_LOW_US (4500)
#define LGAC_PAYLOAD_ONE_HIGH_US (560)
#define LGAC_PAYLOAD_ONE_LOW_US (1690)
#define LGAC_PAYLOAD_ZERO_HIGH_US (560)
#define LGAC_PAYLOAD_ZERO_LOW_US (560)
#define LGAC_REPEAT_CODE_HIGH_US (9000)
#define LGAC_REPEAT_CODE_LOW_US (2250)
#define LGAC_ENDING_CODE_HIGH_US (560)

/**
 * @brief Timings for SONY (SIRCS) protocol
 *
 */
#define SONY_LEADING_CODE_HIGH_US (2400)
#define SONY_LEADING_CODE_LOW_US (600)
#define SONY_PAYLOAD_ONE_HIGH_US (1200)
#define SONY_PAYLOAD_ONE_LOW_US (600)
#define SONY_PAYLOAD_ZERO_HIGH_US (600)
#define SONY_PAYLOAD_ZERO_LOW_US (600)
#define SONY_REPEAT_CODE_HIGH_US (2400)
#define SONY_REPEAT_CODE_LOW_US (600)
//#define SONY_ENDING_CODE_HIGH_US (560)

/**
 * @brief Timings for GREE protocol
 *
 */
/*
#define GREE_LEADING_CODE_HIGH_US (9000)
#define GREE_LEADING_CODE_LOW_US (4500)
#define GREE_PAYLOAD_ONE_HIGH_US (620)
#define GREE_PAYLOAD_ONE_LOW_US (1600)
#define GREE_PAYLOAD_ZERO_HIGH_US (620)
#define GREE_PAYLOAD_ZERO_LOW_US (540)
//#define GREE_REPEAT_CODE_HIGH_US (2400)
//#define GREE_REPEAT_CODE_LOW_US (600)
//#define GREE_ENDING_CODE_HIGH_US (560)
*/

#ifdef __cplusplus
}
#endif
