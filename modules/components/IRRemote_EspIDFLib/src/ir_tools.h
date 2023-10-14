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

#include "esp_err.h"

#define IR_TOOLS_FLAGS_PROTO_EXT (1 << 0) /*!< Enable Extended IR protocol */
#define IR_TOOLS_FLAGS_INVERSE (1 << 1)   /*!< Inverse the IR signal, i.e. take high level as low, and vice versa */

/**
* @brief IR device type
*
*/
typedef void *ir_dev_t;

/**
* @brief IR parser type
*
*/
typedef struct ir_parser_s ir_parser_t;

/**
* @brief Type definition of IR parser
*
*/
struct ir_parser_s {
    /**
    * @brief Input raw data to IR parser
    *
    * @param[in] parser: Handle of IR parser
    * @param[in] raw_data: Raw data which need decoding by IR parser
    * @param[in] length: Length of raw data
    *
    * @return
    *      - ESP_OK: Input raw data successfully
    *      - ESP_ERR_INVALID_ARG: Input raw data failed because of invalid argument
    *      - ESP_FAIL: Input raw data failed because some other error occurred
    */
    esp_err_t (*input)(ir_parser_t *parser, void *raw_data, uint32_t length);

    /**
    * @brief Get the scan code after decoding of raw data
    *
    * @param[in] parser: Handle of IR parser
    * @param[out] address: Address of the scan code
    * @param[out] command: Command of the scan code
    * @param[out] repeat: Indicate if it's a repeat code
    *
    * @return
    *      - ESP_OK: Get scan code successfully
    *      - ESP_ERR_INVALID_ARG: Get scan code failed because of invalid arguments
    *      - ESP_FAIL: Get scan code failed because some error occurred
    */
    esp_err_t (*get_scan_code)(ir_parser_t *parser, uint32_t *address, uint32_t *command, bool *repeat);

    /**
    * @brief Free resources used by IR parser
    *
    * @param[in] parser: Handle of IR parser
    *
    * @return
    *      - ESP_OK: Free resource successfully
    *      - ESP_FAIL: Free resources fail failed because some error occurred
    */
    esp_err_t (*del)(ir_parser_t *parser);
};

/**
* @brief Configuration type of IR parser
*
*/
typedef struct {
    ir_dev_t dev_hdl;   /*!< IR device handle */
    uint32_t flags;     /*!< Flags for IR parser, different flags will enable different features */
    uint32_t margin_us; /*!< Timing parameter, indicating the tolerance to environment noise */
} ir_parser_config_t;

/**
 * @brief Default configuration for IR parser
 *
 */
#define IR_PARSER_DEFAULT_CONFIG(dev) \
    {                                 \
        .dev_hdl = dev,               \
        .flags = 0,                   \
        .margin_us = 200,             \
    }



/**
* @brief Creat a NEC protocol parser
*
* @param config: configuration of NEC parser
* @return
*      Handle of NEC parser or NULL
*/
ir_parser_t *ir_parser_rmt_new_nec(const ir_parser_config_t *config);

#ifdef __cplusplus
}
#endif
