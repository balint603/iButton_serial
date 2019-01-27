/*
 * fsm.c
 *
 *  Created on: 2018. okt. 27.
 *      Author: MAJXAAPPTE
 *
 *
 *  Description:
 *  This module is a finite state machine. Using state functions and a global pointer which points to a state function.
 */

#include <msp430.h>
#include <inttypes.h>
#include "fsm.h"
#include "uart.h"
#include "flash.h"
#include "ibutton.h"

/** LED blinks */
#define INFO_NONE 0
#define INFO_2_BEEPS 2
#define INFO_3_BEEPS 3
#define INFO_ONLY_GREEN 4
#define INFO_ONLY_RED 5
#define INFO_BOTH 6
#define INFO_BOTH_ONLY_LIGHT 7

#define INFO_SHORT 200
#define INFO_LONG 1600


/** State functions_________________________________________________________________________________________________*/
static void shorted_reader(inputs_t input);
static void access_allow_bistable(inputs_t input);
static void access_allow_bistable_same_key(inputs_t input);
static void access_allow(inputs_t input);
static void access_denied(inputs_t input);
static void fast_add_mode(inputs_t input);
static void master_delete(inputs_t input);
static void master_mode(inputs_t input);
static void add_master_key(inputs_t input);
static void check_touch(inputs_t input);
/** State functions end_____________________________________________________________________________________________*/


/** FLASH pointers initialization */
iButton_key_data_t iButton_data = {
                                    .master_key_code_ptr = (uint16_t*)SETTINGS_START,
                                    .opening_time_ptr = (uint16_t*)FLASH_TIME_ADDR,
                                    .mode_ptr = (uint16_t*)FLASH_MODE_ADDR,
                                    .reader_enable_flag = 1,
};

/** STATIC FUNCTIONS declare________________________________________________________________________________________ */
static void init_ports();
static void make_sound(uint8_t mode, uint16_t time);
static void refresh_timing();
/** STATIC FUNCTIONS end____________________________________________________________________________________________ */

/** iButton reading. */
uint8_t prev_presence = 0;
uint8_t curr_presence = 0;

uint_fast16_t LED_ms = 125;
volatile uint8_t LED_flag;

volatile uint_fast16_t timeout_ms;
volatile uint8_t timeout_ticking;

volatile uint16_t uart_timeot_ms = 1000;
volatile uint8_t uart_timeout_ticking;

volatile uint_fast16_t user_info_ms;
volatile uint8_t user_info_flag;
volatile uint8_t user_info_mode;

volatile uint16_t piezo_on_time;
volatile uint8_t piezo_ticking;

/** UART MODULE variables */
volatile uint8_t RX_is_packet;
volatile uint8_t TX_is_packet;


#define TIMEOUT(time) {timeout_ms = time; timeout_ticking = 1;}
#define SEND_USER_INFO(mode,sound_mode,sound_length)\
                        make_sound(sound_mode, sound_length);\
                        user_info_mode = mode;\
                        user_info_flag = 1;

#define PUT_INPUT(put_this) {ibutton_fsm.input = put_this; ibutton_fsm.input_to_serve = 1;}

#define READ_PUSHBUTTON {\
    if( !(GPIO_GET_INPUT(PUSHBUTTON_PORT,PUSHBUTTON_PIN) || ibutton_fsm.input_to_serve) ){\
        PUT_INPUT(button_pressed);\
    }\
}

/** STATIC functions____________________________________________________________________________________ */
static void init_ports(){
    PIEZO_PORT_DIR |= PIEZO_BIT;
    P2SEL &= ~BIT7;
    GPIO_SET_INPUT_PULL_UP_VCC(JUMPER_A_PORT,JUMPER_A_PIN);
    GPIO_SET_INPUT_PULL_UP_VCC(JUMPER_B_PORT,JUMPER_B_PIN);
    GPIO_SET_INPUT_PULL_UP_VCC(JUMPER_M_PORT,JUMPER_M_PIN);

    GPIO_SET_INPUT_PULL_UP_VCC(PUSHBUTTON_PORT,PUSHBUTTON_PIN);

    REL_PORT_DIR |= REL_BIT;
    P2OUT &= ~REL_BIT;  // relay
    REL_OFF;
}


static void refresh_timing(){
    uint16_t time = 0;

    if(GPIO_GET_INPUT(JUMPER_A_PORT, JUMPER_A_PIN))
        time += OPENING_TIME_J_A;
    if(GPIO_GET_INPUT(JUMPER_B_PORT, JUMPER_B_PIN))
        time += OPENING_TIME_J_B;
    if(time)
        iButton_data.opening_time = time;
    else
        iButton_data.opening_time = *(iButton_data.opening_time_ptr);
}




/** STATIC functions end___________________________________________________________________________________ */


/** PUBLIC functions end___________________________________________________________________________________ */

/**
 * Initialization.
 * If the
 * */
void ibutton_fsm_init(){
    uint16_t data_ff[] = {0xFFFF,0xFFFF,0xFFFF};
    uint16_t basic_time = OPENING_TIME_BASIC;

    init_ports();

    reader_polling_ms = READ_POLLING_TIME;
    reader_polling_flag = 1;

    ibutton_fsm.input = key_away;
    ibutton_fsm.input_to_serve = 0;
    ibutton_fsm.current_state = check_touch;

    if((*iButton_data.opening_time_ptr) == 0xFFFF)                              // Opening time empty?
        if(flash_change_settings(FLASH_TIME, &basic_time, 1)){
            //uart_send_str("Change settings error!", 1);
            SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
        }
    refresh_timing();

    if(!compare_key(data_ff, (uint16_t*)iButton_data.master_key_code_ptr)){     // Master code place empty?
        //uart_send_str("First start.", 1);

        // todo memory check

        //uart_send_str("Please touch a master key!", 1);
        LED_TURN_ON_GR;
        LED_TURN_ON_RE;
        user_info_mode = INFO_BOTH_ONLY_LIGHT;
        user_info_flag = 1;
        ibutton_fsm.current_state = add_master_key;
    }
    else if( !(GET_INPUT) && GPIO_GET_INPUT(JUMPER_M_PORT,JUMPER_M_PIN) ){       // Check shorted and enabled
        SEND_USER_INFO(INFO_ONLY_GREEN,0,INFO_LONG);
        ibutton_fsm.current_state = shorted_reader;
        TIMEOUT(INFO_LONG);
    }else{
        make_sound(0, INFO_SHORT);
        LED_TURN_OFF_RE;
    }
}

void ibutton_read(){

    if( curr_presence = ibutton_test_presence()){
        if(!iButton_data.reader_enable_flag){
            reader_disable_ms = READ_DISABLE_TIME;
            READ_PUSHBUTTON;
        }
        else if(!ibutton_read_it(iButton_data.key_code)){
            if(compare_key((uint16_t*)iButton_data.master_key_code_ptr, iButton_data.key_code)){
                PUT_INPUT(key_touched);
            }
            else{
                PUT_INPUT(master_key_touched);
            }
            ibutton_fsm.input_to_serve = 1;
            iButton_data.reader_enable_flag = 0;
        }
    }
    else if(prev_presence){
        reader_disable_ms = READ_DISABLE_TIME;
    }
    else{
        READ_PUSHBUTTON;
    }
    prev_presence = curr_presence;
    reader_polling_flag = 0;
}

void ibutton_fsm_change_state(){
    ibutton_fsm.current_state(ibutton_fsm.input);
}


/**
 * \ret 0  Key codes match.
 * \ret 1  Key codes do not match.
 * \ret -1 Null parameters.
 * */
int compare_key(uint16_t *key1, uint16_t *key2){
    uint8_t i;
    if(!(key1 && key2))
        return -1;
    for(i=3; i > 0; i--)
        if(*(key1++) != *(key2++))
            return 1;
    return 0;
}

int copy_key(uint16_t *from, uint16_t *to){
    if(!(from || to))
        return 1;
    uint8_t i;
    for(i = 3; i > 0; i--)
        *(to++) = *(from++);
    return 0;
}

/**
 * \param mode: toggle or reset/set output mode.
 * */
static void make_sound(uint8_t mode, uint16_t time){
    if(mode)
        TA0CCTL1 = OUTMOD_4;
    else
        TA0CCTL1 = OUTMOD_7;
    PIEZO_PORT_DIR |= PIEZO_BIT;
    piezo_on_time = time;
    PIEZO_PORT_SEL |= PIEZO_BIT; // start beep
}

void ibutton_user_info_mode_service(){
    switch(user_info_mode){
        case INFO_ONLY_GREEN:
            LED_BLINK_GR;
            user_info_ms = 999;
            break;
        case INFO_ONLY_RED:
            LED_BLINK_RE;
            user_info_ms = 999;
            break;
        case INFO_BOTH:
            make_sound(0, 120);
            LED_BLINK_GR;
            LED_BLINK_RE;
            user_info_ms = 999;
            break;
        case INFO_BOTH_ONLY_LIGHT:
            LED_BLINK_GR;
            LED_BLINK_RE;
            user_info_ms = 999;
            break;
        case 3:
            make_sound(0, INFO_SHORT);
            user_info_mode--;
            user_info_ms = 299;
            break;
        case 2:
            make_sound(0, INFO_SHORT);
            user_info_mode--;
            user_info_ms = 299;
            break;
        case 1:
            make_sound(0, 349);
            user_info_mode--;
            break;
        default:
            user_info_mode = INFO_NONE;
            break;
    }
}
/** PUBLIC functions end___________________________________________________________________________________ */




/* State functions start___________________________________________________________________________________ */

static void shorted_reader(inputs_t input){
    if(!timeout_ticking){
        if(!GET_INPUT && GPIO_GET_INPUT(JUMPER_M_PORT,JUMPER_M_PIN)){                  // Still pulled down and enabled
            ibutton_fsm.current_state = add_master_key;
            user_info_mode = INFO_BOTH_ONLY_LIGHT;
            user_info_flag = 1;
            LED_TURN_ON_GR;
            LED_TURN_ON_RE;
            TIMEOUT(60000);
        }
        else{
            ibutton_fsm.current_state = check_touch;
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            user_info_mode = INFO_NONE;
        }
    }
}

static void access_allow_bistable(inputs_t input){

    uint16_t addr = SEGMENT_0;

    REL_ON;
    LED_TURN_OFF_GR;
    switch (input){
    case key_touched:
        if((flash_search_key(iButton_data.key_code, &addr))){
            //uart_send_str("RELAY=OFF", 1);
            LED_TURN_ON_GR;
            make_sound(1,INFO_SHORT);
            REL_OFF;
            ibutton_fsm.current_state = check_touch;
        }
        else{
            //uart_send_str("Wrong key!",1);
            SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
        }
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void access_allow_bistable_same_key(inputs_t input){
    REL_ON;
    LED_TURN_OFF_GR;
    switch (input){
    case key_touched:
        if(!compare_key(iButton_data.prev_key_code, iButton_data.key_code)){
            //uart_send_str("RELAY=OFF", 1);
            LED_TURN_ON_GR;
            make_sound(1,INFO_SHORT);
            REL_OFF;
            ibutton_fsm.current_state = check_touch;
        }
        else{
            //uart_send_str("Wrong key!",1);
            SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
        }
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void access_allow(inputs_t input){
    LED_TURN_OFF_GR;
    REL_ON;
    if(!timeout_ticking){
        if(!(--iButton_data.opening_time)){
           // uart_send_str("RELAY=OFF", 1);
            SEND_USER_INFO(INFO_NONE,0,INFO_SHORT);
            LED_TURN_ON_GR;
            // test uart

            uart_send((uint8_t*)iButton_data.key_code, 1, 6);
            REL_OFF;
            ibutton_fsm.current_state = check_touch;
            refresh_timing();
        }else{
            make_sound(0, INFO_SHORT);
            TIMEOUT(1000);   // Wait 500ms again...
        }
    }
    ibutton_fsm.input_to_serve = 0;

}

static void access_denied(inputs_t input){
    switch (input){
    case key_away:
        TIMEOUT(2000);
        break;
    case timeout:
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        ibutton_fsm.current_state = check_touch;
        break;
    case button_pressed:
        //uart_send_str("RELAY=ON", 1);
        REL_ON;
        make_sound(1, INFO_SHORT);
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        if(*iButton_data.mode_ptr){
            ibutton_fsm.current_state = access_allow;
            refresh_timing();
            TIMEOUT(200);
            break;
        }
    }
    ibutton_fsm.input_to_serve = 0;
}

static void fast_add_mode(inputs_t input){
    uint16_t address;
    uint16_t *flash_ptr;
    switch (input) {
        case timeout:
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            SEND_USER_INFO(INFO_NONE,1,INFO_LONG);
            //uart_send_str("Normal mode>", 1);
            ibutton_fsm.current_state = check_touch;
            break;
        case key_touched:
            TIMEOUT(60000);
            LED_TURN_ON_RE;
            if(flash_search_key(iButton_data.key_code, &address)){
                //uart_send_str("Already added!", 1);
                make_sound(0, INFO_SHORT);
            }
            else{
                flash_write_data(iButton_data.key_code, 3, address);
            }
            flash_ptr = (uint16_t*)address;
            if(compare_key(flash_ptr, iButton_data.key_code)){
                //uart_send_str("Flash write error!",1);
                SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
            }
            else
                make_sound(0, INFO_SHORT);
            LED_TURN_OFF_RE;
            break;
        case master_key_touched:
            user_info_ms = 1000;
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            //uart_send_str("Normal mode>", 1);
            SEND_USER_INFO(INFO_NONE,0,INFO_SHORT);
            ibutton_fsm.current_state = check_touch;
            break;
        default:
            break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void add_master_key(inputs_t input){
    switch(input){
    case key_touched:
        if(flash_change_settings(FLASH_MASTER_CODE, iButton_data.key_code, 3)){
            SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
            //uart_send_str("Change settings error!", 1);
        }
        if(!compare_key(iButton_data.key_code, iButton_data.master_key_code_ptr)){
            //uart_send_str("Fast add mode#", 1);
            SEND_USER_INFO(INFO_ONLY_GREEN,0,INFO_SHORT);
            TIMEOUT(60000);
            ibutton_fsm.current_state = fast_add_mode;
        }
        else{
            //uart_send_str("ADD M_key error!", 1);
            ibutton_fsm.current_state = check_touch;
            SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
        }
        break;
    case master_key_touched:
        //uart_send_str("Fast add mode#",1);
        TIMEOUT(60000);
        SEND_USER_INFO(INFO_ONLY_GREEN,0,INFO_SHORT);
        ibutton_fsm.current_state = fast_add_mode;
        break;
    }
    LED_TURN_OFF_RE;
    ibutton_fsm.input_to_serve = 0;
}

static void master_delete(inputs_t input){
    switch (input) {
        case timeout:
            LED_TURN_OFF_RE;
            LED_TURN_ON_GR;
            SEND_USER_INFO(INFO_NONE,1,INFO_LONG);
            ibutton_fsm.current_state = check_touch;
            break;
        case master_key_touched:
            make_sound(1, 1000);
            LED_TURN_OFF_GR;
            LED_TURN_ON_RE;
            segment_erase(0);   // All segment erase!
            flash_init();
            LED_TURN_OFF_RE;
            LED_TURN_ON_GR;
            SEND_USER_INFO(INFO_ONLY_GREEN,0,INFO_SHORT);
            //uart_send_str("Fast add mode!", 1);
            TIMEOUT(60000);
            ibutton_fsm.current_state = fast_add_mode;
        default:
            break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void master_mode(inputs_t input){
    uint16_t address;
    switch(input){
    case timeout:
        //uart_send_str("Normal mode>", 1);
        LED_TURN_ON_GR;
        SEND_USER_INFO(INFO_NONE,1,INFO_LONG);
        user_info_ms = 1000;
        ibutton_fsm.current_state = check_touch;
        break;
    case master_key_touched:
        //uart_send_str("Erase key code data?", 1);
        LED_TURN_OFF_RE;
        LED_TURN_ON_GR;
        SEND_USER_INFO(INFO_BOTH,0,INFO_SHORT);
        TIMEOUT(6000);
        ibutton_fsm.current_state = master_delete;
        break;
    case key_touched:
        if(flash_search_key(iButton_data.key_code, &address)){
            flash_delete_key(address);
            //uart_send_str("Key has been deleted.", 1);
            SEND_USER_INFO(INFO_2_BEEPS,0,INFO_SHORT);
        }
        else{
            if(!flash_write_data(iButton_data.key_code, 3, address)){
                //uart_send_str("Key has been added.", 1);
                SEND_USER_INFO(INFO_NONE,0,INFO_SHORT);
            }
            else{
                //uart_send_str("Key-code save failed!", 1);
                SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
            }
        }
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        ibutton_fsm.current_state = check_touch;
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void check_touch(inputs_t input){
    ibutton_fsm.input_to_serve = 0;
    uint16_t addr = SEGMENT_0;

    timeout_ms = 0;

    switch (ibutton_fsm.input){
    case master_key_touched:
        if(GPIO_GET_INPUT(JUMPER_M_PORT,JUMPER_M_PIN)){         // Master enable jumper off
            //uart_send_str("Master mode#", 1);
            TIMEOUT(60000);
            SEND_USER_INFO(INFO_ONLY_GREEN,0,INFO_SHORT);
            LED_TURN_OFF_GR;
            ibutton_fsm.current_state = master_mode;
        }
        else{
            //uart_send_str("RELAY=ON", 1);
            REL_ON;
            make_sound(1, INFO_SHORT);
            // todo periodic beeps
            switch (*iButton_data.mode_ptr) {           // Bistable modes or normal mode
                case MODE_BISTABLE_SAME_K:
                    copy_key(iButton_data.key_code, iButton_data.prev_key_code);
                    ibutton_fsm.current_state = access_allow_bistable_same_key;
                    break;
                case MODE_BISTABLE:
                    ibutton_fsm.current_state = access_allow_bistable;
                    break;
                default:
                    ibutton_fsm.current_state = access_allow;
                    refresh_timing();
                    TIMEOUT(1000);
                    break;
            }
            LED_TURN_OFF_GR;
        }
        break;
    case key_touched:
        if(flash_search_key(iButton_data.key_code, &addr)){
            //uart_send_str("RELAY=ON", 1);
            REL_ON;
            make_sound(1, INFO_SHORT);
            switch (*iButton_data.mode_ptr) {                   // Bistable modes or normal mode
                case MODE_BISTABLE_SAME_K:
                    copy_key(iButton_data.key_code, iButton_data.prev_key_code);
                    ibutton_fsm.current_state = access_allow_bistable_same_key;
                    break;
                case MODE_BISTABLE:
                    ibutton_fsm.current_state = access_allow_bistable;
                    break;
                default:
                    ibutton_fsm.current_state = access_allow;
                    refresh_timing();
                    TIMEOUT(1000);
                    break;
            }
            LED_TURN_OFF_GR;
        }
        else{
            //uart_send_str("ACCESS DENIED!",1);
            LED_TURN_OFF_GR;
            LED_TURN_ON_RE;
            SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
            ibutton_fsm.current_state = access_denied;
        }
            break;
    case button_pressed:
        if(*iButton_data.mode_ptr == MODE_NORMAL){
            //uart_send_str("RELAY=ON", 1);
            REL_ON;
            make_sound(1, INFO_SHORT);
            LED_TURN_OFF_RE;
            ibutton_fsm.current_state = access_allow;
            refresh_timing();
            TIMEOUT(1000);
            LED_TURN_OFF_GR;
        }
        break;
    }
}

/* State functions stop____________________________________________________________________________________ */


/**
 * Interrupt routines:
 * */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void){
    if(!(--LED_ms)){    // DEBUG LED
        LED_flag = 1;
        LED_ms = 1000;
    }
    if(timeout_ticking){
        if(!timeout_ms--){
            PUT_INPUT(timeout);
            ibutton_fsm.input_to_serve = 1;
            timeout_ticking = 0;
        }
    }
    if(uart_timeout_ticking){
        if(!(--uart_timeot_ms)){
            uart_timeout_ticking = 0;
            uart_timeout();
        }
    }
    if(!piezo_on_time--){
         PIEZO_PORT_SEL &= ~PIEZO_BIT;
         PIEZO_PORT_DIR &= ~PIEZO_BIT;
    }
    if(user_info_mode){
        if(!user_info_ms--){
            user_info_flag = 1;
        }
    }
    if( !( --reader_polling_ms )){
        reader_polling_flag = 1;
        reader_polling_ms = READ_POLLING_TIME;
    }
    if( !(--reader_disable_ms)){
        iButton_data.reader_enable_flag = 1;
        PUT_INPUT(key_away);
    }
}
