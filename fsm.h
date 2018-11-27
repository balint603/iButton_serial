/*
 * fsm.h
 *
 *  Created on: 2018. okt. 27.
 *      Author: MAJXAAPPTE
 */


#ifndef FSM_H_
#define FSM_H_

#include <inttypes.h>


#define REL_PORT_DIR P2DIR
#define REL_BIT BIT3
#define REL_ON REL_PORT_DIR |= REL_BIT
#define REL_OFF REL_PORT_DIR &= ~REL_BIT

#define READ_DISABLE_TIME 500 /* ms */
#define READ_POLLING_TIME 10 /* ms */
#define OPENING_TIME_BASIC 5000 /* ms */

typedef enum inputs {
    key_touched,
    master_key_touched,
    button_pressed,
    key_away,
    timeout
} inputs_t;

typedef void ( *p_state_handler )(inputs_t input);

typedef struct fsm{
    inputs_t input;
    p_state_handler current_state;
    uint8_t input_to_serve;
} ibutton_fsm_t;

typedef struct iButton_key_data{
    uint16_t const *master_key_code_ptr;
    uint16_t const *opening_time_ptr;
    uint16_t const *mode_ptr;
    uint16_t const *button_enable_ptr;
    uint16_t key_code[3];
    const uint16_t super_master_key_code[3];
    volatile uint8_t reader_enable_flag;
} iButton_key_data_t;

void ibutton_fsm_init();
void ibutton_fsm_change_state();
void ibutton_read();
void put_input(inputs_t input);
int compare_key(uint16_t *key1, uint16_t *key2);

ibutton_fsm_t ibutton_fsm;
uint_fast16_t reader_polling_ms;
volatile uint8_t reader_polling_flag;
uint_fast16_t reader_disable_ms;

/** Input to serve */
volatile extern uint8_t fsm_input_flag;
extern iButton_key_data_t iButton_data;

extern uint8_t led_blink_enable;

volatile extern uint_fast16_t led_blink_ms;
volatile extern uint_fast16_t timeout_ms;

#endif /* FSM_H_ */
