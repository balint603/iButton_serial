/*
 * fsm.h
 *
 *  Created on: 2018. okt. 27.
 *      Author: MAJXAAPPTE
 */


#ifndef FSM_H_
#define FSM_H_

#include <inttypes.h>
/** PORT SETTINGS START______________________________________________________________________ */
#define JUMPER_A_PORT   1
#define JUMPER_B_PORT   1
#define JUMPER_M_PORT   1
#define JUMPER_A_PIN    BIT5
#define JUMPER_B_PIN    BIT4
#define JUMPER_M_PIN    BIT3

#define PUSHBUTTON_PORT 2
#define PUSHBUTTON_PIN BIT2

#define GPIO_DIR(port)  P ## port ## DIR
#define GPIO_OUT(port)  P ## port ## OUT
#define GPIO_IN(port)   P ## port ## IN
#define GPIO_REN(port)  P ## port ## REN

#define GPIO_SET_INPUT_PULL_UP_VCC(port,pin) { GPIO_DIR(port) &= ~pin; GPIO_REN(port) |= pin; GPIO_OUT(port) |= pin;}
#define GPIO_GET_INPUT(port,pin)  (GPIO_IN(port) & (pin))

#define REL_PORT_DIR    P2DIR
#define REL_PORT_OUT    P2OUT
#define REL_BIT         BIT1
#define REL_OFF         (REL_PORT_OUT |= REL_BIT)
#define REL_ON          (REL_PORT_OUT &= ~REL_BIT)

#define PIEZO_PORT_DIR  P2DIR
#define PIEZO_PORT_SEL  P2SEL
#define PIEZO_BIT       BIT6

/** PORT SETTINGS START______________________________________________________________________ */


#define READ_DISABLE_TIME 600 /* ms/2 */
#define READ_POLLING_TIME 10 /* ms/2 */

#define OPENING_TIME_BASIC 2 /* ms/2 */
#define OPENING_TIME_J_A 20   /* ms/2 */
#define OPENING_TIME_J_B 10   /* ms/2 */

#define MODE_NORMAL 0xFFFF
#define MODE_BISTABLE 0xAAAA
#define MODE_BISTABLE_SAME_K 0x5555

typedef enum inputs {
    key_touched,
    master_key_touched,
    button_pressed,
    key_away,
    timeout,
    relay_timeout
} inputs_t;

typedef void ( *p_state_handler )(inputs_t input);

typedef struct fsm{
    inputs_t input;
    p_state_handler current_state;
    uint8_t input_to_serve;
} ibutton_fsm_t;



/**
 *
 * \var mode_ptr: If this equals to 0xFFFF normal mode is active, 0xAAAA - Bistable, 0x5555 - Bistable same key.
 * */
typedef struct iButton_key_data{
    uint16_t const *master_key_code_ptr;
    uint16_t const *opening_time_ptr;
    uint16_t const *mode_ptr;
    uint16_t const *button_enable_ptr;
    volatile uint16_t opening_time;
    uint16_t key_code[3];
    uint16_t prev_key_code[3];
    const uint16_t super_master_key_code[3];
    volatile uint8_t reader_enable_flag;
} iButton_key_data_t;

void ibutton_fsm_init();
void ibutton_fsm_change_state();
void ibutton_read();
void make_sound(uint8_t mode, uint16_t time);
void ibutton_fsm_put_input(inputs_t input);
int compare_key(uint16_t *key1, uint16_t *key2);
void ibutton_user_info_mode_service();
void ibutton_timeout_service();

ibutton_fsm_t ibutton_fsm;
uint_fast16_t reader_polling_ms;
volatile uint8_t reader_polling_flag;
uint_fast16_t reader_disable_ms;

/** Input to serve */

#endif /* FSM_H_ */
