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

/** LED blink */
#define NONE 0
#define ONLY_GREEN 1
#define ONLY_RED 2
#define BOTH 3

/** State functions */
static void access_allow_bistable(inputs_t input);
static void access_allow(inputs_t input);
static void access_denied(inputs_t input);
static void fast_add_mode(inputs_t input);
static void master_delete(inputs_t input);
static void master_mode(inputs_t input);
static void add_master_key(inputs_t input);
static void check_touch(inputs_t input);


/** iButton reading. */
uint8_t prev_presence = 0;
uint8_t curr_presence = 0;

uint8_t time_s;
/**
 * Initialization.
 * If the
 * */
void ibutton_fsm_init(){
    uint16_t data_ff[] = {0xFFFF,0xFFFF,0xFFFF};

    reader_polling_ms = READ_POLLING_TIME;
    reader_polling_flag = 1;

    ibutton_fsm.input = key_away;
    ibutton_fsm.input_to_serve = 0;
    ibutton_fsm.current_state = check_touch;

    if(*iButton_data.opening_time_ptr == 0xFFFF)
        flash_write_word(OPENING_TIME_BASIC, FLASH_TIME);

    if(!compare_key(data_ff, (uint16_t*)iButton_data.master_key_code_ptr)){
        uart_send_str("First start.", 1);

        // todo check memory is all FFFFh?

        uart_send_str("Please touch a master key!", 1);
        LED_TURN_OFF_RE;
        led_blink_enable = ONLY_GREEN;
        ibutton_fsm.current_state = add_master_key;
        // fast key adding mode
    }
    else if(!(GET_INPUT)){ // todo Check multiple times!
        LED_TURN_OFF_RE;
        led_blink_enable = ONLY_GREEN;
        uart_send_str("Please touch a master key!", 1);
        ibutton_fsm.current_state = add_master_key;
    }
    LED_TURN_OFF_RE;
}

void ibutton_read(){

    if( curr_presence = ibutton_test_presence()){
        if(!iButton_data.reader_enable_flag)
            reader_disable_ms = READ_DISABLE_TIME;
        else if(!ibutton_read_it(iButton_data.key_code)){

            if(compare_key((uint16_t*)iButton_data.master_key_code_ptr, iButton_data.key_code))
                put_input(key_touched);
            else
                put_input(master_key_touched);
            ibutton_fsm.input_to_serve = 1;
            iButton_data.reader_enable_flag = 0;
        }
    }
    else if(prev_presence)
        reader_disable_ms = READ_DISABLE_TIME;
    prev_presence = curr_presence;
    reader_polling_flag = 0;

}

void ibutton_fsm_change_state(){
    ibutton_fsm.current_state(ibutton_fsm.input);
}

void put_input(inputs_t input){
    ibutton_fsm.input_to_serve = 1;
    ibutton_fsm.input = input;
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

int search_key_and_write(){
      return 0;
}


/* State functions start___________________________________________________________________________________ */

static void access_allow_bistable(inputs_t input){
/*
    uint16_t addr = 0;
    uint16_t addr_ff = 0;

    REL_ON;
     LED_TURN_OFF_GR;
     switch (input){
     case key_touched:
         if(EEPROM_get_key_or_empty_place(iButton_data.key_code, &addr, &addr_ff, EEPROM_LAST_KEY_SPACE, 0) == 1){
             uart_send_str("RELAY=OFF", 1);
             LED_TURN_ON_GR;
             REL_OFF;
             ibutton_fsm.current_state = check_touch;
         }
         break;
     }
     ibutton_fsm.input_to_serve = 0;*/
}

static void access_allow(inputs_t input){
    REL_ON;
    LED_TURN_OFF_GR;
    switch (input){
    case timeout:
        uart_send_str("RELAY=OFF", 1);
        LED_TURN_ON_GR;
        REL_OFF;
        ibutton_fsm.current_state = check_touch;
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void access_denied(inputs_t input){
    switch (input){
    case key_away:
        timeout_ms = 1000;
        break;
    case timeout:
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        ibutton_fsm.current_state = check_touch;
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void fast_add_mode(inputs_t input){
    uint16_t address;
    uint16_t *flash_ptr;
    switch (input) {
        case timeout:
            led_blink_enable = 0;
            led_blink_ms = 500;
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            uart_send_str("Normal mode>", 1);
            ibutton_fsm.current_state = check_touch;
            break;
        case key_touched:
            timeout_ms = 30000;
            // test EEPROM WP
            LED_TURN_ON_RE;
            if(flash_search_key(iButton_data.key_code, &address)){
                uart_send_str("Already added!", 1);
            }
            else{
                flash_write_data(iButton_data.key_code, 3, address);
            }
            flash_ptr = (uint16_t*)address;
            if(compare_key(flash_ptr, iButton_data.key_code))
                uart_send_str("Flash write error!",1);
            LED_TURN_OFF_RE;
            break;
        case master_key_touched:
            led_blink_enable = 0;
            led_blink_ms = 500;
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            uart_send_str("Normal mode>", 1);
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
        if(flash_write_data(iButton_data.key_code, 3, FLASH_MASTER_CODE))
            uart_send_str("Flash write error", 1);
        if(!compare_key(iButton_data.key_code, iButton_data.master_key_code_ptr)){
            uart_send_str("Fast add mode#", 1);
            timeout_ms = 30000;
            ibutton_fsm.current_state = fast_add_mode;
        }
        else{
            uart_send_str("ADD M_key error!", 1);
            ibutton_fsm.current_state = check_touch;
        }
        break;
    case master_key_touched:
        uart_send_str("Fast add mode#",1);
        timeout_ms = 30000;
        ibutton_fsm.current_state = fast_add_mode;
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void master_delete(inputs_t input){
    switch (input) {
        case timeout:
            led_blink_enable = NONE;
            LED_TURN_OFF_RE;
            LED_TURN_ON_GR;
            ibutton_fsm.current_state = check_touch;
            break;
        case master_key_touched:
            LED_TURN_OFF_GR;
            LED_TURN_ON_RE;
            segment_erase(0);   // All segment erase!
            segment_erase(FLASH_MASTER_CODE);       // Erase information memory.
            flash_write_word(OPENING_TIME_BASIC, FLASH_TIME);
            flash_init();
            LED_TURN_OFF_RE;
            LED_TURN_ON_GR;
            led_blink_enable = ONLY_GREEN;
            uart_send_str("Please touch a master key!", 1);
            ibutton_fsm.current_state = add_master_key;
        default:
            break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void master_mode(inputs_t input){
    uint16_t address;
    switch(input){
    case timeout:
        uart_send_str("Normal mode>", 1);
        led_blink_enable = 0;
        LED_TURN_ON_GR;
        led_blink_ms = 500;
        // todo stop blinking, reset blinking timer variable
        ibutton_fsm.current_state = check_touch;
        break;
    case master_key_touched:
        uart_send_str("Erase key code data?", 1);
        LED_TURN_OFF_RE;
        LED_TURN_ON_GR;
        led_blink_enable = BOTH;
        timeout_ms = 3000;
        ibutton_fsm.current_state = master_delete;
        break;
    case key_touched:
        if(flash_search_key(iButton_data.key_code, &address)){
            flash_delete_key(address);
            uart_send_str("Key has been deleted.", 1);
        }
        else{
            flash_write_data(iButton_data.key_code, 3, address);
            uart_send_str("Key has been added.", 1);
        }
        led_blink_enable = 0;
        led_blink_ms = 500;
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        ibutton_fsm.current_state = check_touch;
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void check_touch(inputs_t input){

    uint16_t addr = SEGMENT_0;

    timeout_ms = 0;

    switch (ibutton_fsm.input){
    case key_touched:
        if(flash_search_key(iButton_data.key_code, &addr)){
            uart_send_str("RELAY=ON", 1);
            if(*iButton_data.mode_ptr){
                timeout_ms = *iButton_data.opening_time_ptr;
                ibutton_fsm.current_state = access_allow;
            }
            else{
                ibutton_fsm.current_state = access_allow_bistable;
            }
            LED_TURN_OFF_GR;
            REL_ON;
        }
        else{
            uart_send_str("ACCESS DENIED!",1);
            LED_TURN_OFF_GR;
            LED_TURN_ON_RE;
            ibutton_fsm.current_state = access_denied;
        }
            break;
    case master_key_touched:
        uart_send_str("Master mode#", 1);
        timeout_ms = 30000;
        led_blink_enable = 1;
        //REL_ON;
        ibutton_fsm.current_state = master_mode;
        break;
    case button_pressed:
        if(*iButton_data.button_enable_ptr){
            uart_send_str("RELAY=ON", 1);
            timeout_ms = 2000;
            REL_ON;
            ibutton_fsm.current_state = access_allow;
            ibutton_fsm.input_to_serve = 0;
        }
        else
            uart_send_str("Button disabled.", 1);

        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

/* State functions stop____________________________________________________________________________________ */
