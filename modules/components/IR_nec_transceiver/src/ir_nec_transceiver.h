#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "esp_attr.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "ir_nec_encoder.h"

#define EXAMPLE_IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_TX_GPIO_NUM       18
#define EXAMPLE_IR_RX_GPIO_NUM       19
#define EXAMPLE_IR_NEC_DECODE_MARGIN 200     // Tolerance for parsing RMT symbols into bit stream

/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0  9000
#define NEC_LEADING_CODE_DURATION_1  4500
#define NEC_PAYLOAD_ZERO_DURATION_0  560
#define NEC_PAYLOAD_ZERO_DURATION_1  560
#define NEC_PAYLOAD_ONE_DURATION_0   560
#define NEC_PAYLOAD_ONE_DURATION_1   1690
#define NEC_REPEAT_CODE_DURATION_0   9000
#define NEC_REPEAT_CODE_DURATION_1   2250

class ir_nec_transceiver{
public:
    /**
    * @brief Decode RMT symbols into NEC scan code and print the result
    */
    uint16_t parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num);

private:

    /**
    * @brief Saving NEC decode results
    */
    uint16_t s_nec_code_address;
    uint16_t s_nec_code_command;

    /**
    * @brief Check whether the RMT symbols represent NEC repeat code
    */
    bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols);

    /**
    * @brief Decode RMT symbols into NEC address and command
    */
    bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols);

    /**
    * @brief Check whether a RMT symbol represents NEC logic one
    */
    bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols);

    /**
    * @brief Check whether a RMT symbol represents NEC logic zero
    */
    bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols);

    /**
    * @brief Check whether a duration is within expected range
    */
    bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration);
};