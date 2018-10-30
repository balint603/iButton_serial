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
/** Need to be declared */
typedef struct iButton_key_data{
    uint8_t key_code[8];
    uint8_t master_key_code[8];
    const uint8_t super_master_key_code[8];
    volatile uint8_t reader_enable_flag;
    uint8_t buffer_rx[64];
    uint16_t buffer_cnt;
    uint16_t first_free_address;
} iButton_key_data_t;

void ibutton_fsm_init();
void ibutton_fsm_change_state();
void ibutton_fsm_switch_state_to(p_state_handler state);
void put_input(inputs_t input);
int compare_key(uint8_t *key1, uint8_t *key2);

/** States: */
void access_allow(inputs_t input);
void access_denied(inputs_t input);
void master_mode(inputs_t input);
void fast_add_mode(inputs_t input);
void add_master_key(inputs_t input);
void check_touch(inputs_t input);

ibutton_fsm_t ibutton_fsm;
uint_fast16_t reader_polling_ms;
volatile uint8_t reader_polling_flag;
uint_fast16_t reader_disable_ms;

/** Input to serve */
volatile extern uint8_t fsm_input_flag;
extern iButton_key_data_t iButton_data;
volatile extern uint_fast16_t timeout_ms;
extern uint8_t led_blink_enable;

#endif /* FSM_H_ */
