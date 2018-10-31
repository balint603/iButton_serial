/*
 * fsm.c
 *
 *  Created on: 2018. okt. 27.
 *      Author: MAJXAAPPTE
 */

#include <msp430.h>
#include <inttypes.h>
#include "fsm.h"
#include "uart.h"
#include "EEPROM.h"
#include "ibutton.h"

#define NONE 0
#define ONLY_GREEN 1
#define ONLY_RED 2
#define BOTH 3



/** iButton reading. */
uint8_t prev_presence = 0;
uint8_t curr_presence = 0;



void ibutton_fsm_init(){
    reader_polling_ms = READ_POLLING_TIME;
    reader_polling_flag = 1;

    ibutton_fsm.input = key_away;
    ibutton_fsm.input_to_serve = 0;
    ibutton_fsm.current_state = check_touch;
    EEPROM_key_read(iButton_data.master_key_code, EEPROM_MASTER_KEY_PLACE);
    if(iButton_data.master_key_code[0] != 0x01){
        uart_send_str("First start EEPROM erase.", 1);

        LED_TURN_OFF_GR;
        led_blink_enable = 2;

        EEPROM_clear_ff();

        led_blink_enable = 1;

        uart_send_str("Please touch a master key!", 1);
        ibutton_fsm.current_state = add_master_key;
        __delay_cycles(60000);
        __delay_cycles(60000);
        // fast key adding mode
    }
    else if(!(GET_INPUT)){ // todo 1s
        led_blink_enable = 1;
        uart_send_str("Please touch a master key!", 1);
        ibutton_fsm.current_state = add_master_key;
    }
    LED_TURN_OFF_RE;
}

void ibutton_read(){
    if( reader_polling_flag ){
        if( curr_presence = ibutton_test_presence()){
            if(!iButton_data.reader_enable_flag)
                reader_disable_ms = READ_DISABLE_TIME;
            else if(!ibutton_read_it(iButton_data.key_code)){
                if(compare_key(iButton_data.master_key_code, iButton_data.key_code))
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
}

void put_input(inputs_t input){
    ibutton_fsm.input_to_serve = 1;
    ibutton_fsm.input = input;
}

int compare_key(uint8_t *key1, uint8_t *key2){
    int i;
    if(!(key1 && key2))
        return -1;
    for(i=7; i >= 0; i--)
        if(key1[i] != key2[i])
            return 1;
    return 0;
}

int copy_key(uint8_t *from, uint8_t *to){
    if(!(from || to))
        return 1;
    uint8_t i;
    for(i = 8; i > 0; i--)
        *(to++) = *(from++);
    return 0;
}

void ibutton_fsm_change_state(){
    ibutton_fsm.current_state(ibutton_fsm.input);
}

void ibutton_fsm_switch_state_to(p_state_handler state){
    ibutton_fsm.current_state = state;
}

int search_key_and_write(){
    uint16_t addr = 0;
    uint16_t addr_ff = 0;
    uint8_t data_r[8];
    uint8_t i;
    uint8_t deleted_key[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    switch(EEPROM_get_key_or_empty_place(iButton_data.key_code, &addr, &addr_ff, EEPROM_LAST_KEY_SPACE, 0)){
    case 0:
        if(EEPROM_key_write(iButton_data.key_code, addr_ff)){
            uart_send_str("COM ERROR",1);
        }
        __delay_cycles(60000);
        __delay_cycles(60000);
        EEPROM_key_read(data_r, addr_ff);
        for(i = 7; i > 0; i--){
            if(data_r[i] != iButton_data.key_code[i]){
            uart_send_str("ERROR: WRITE INTO FREE SPACE", 1);
            return 1;
            }
        }
        uart_send_str("KEY SUCCESSFULLY ADDED", 1);
        break;
    case 1:
        if(EEPROM_key_write(deleted_key, addr))
            uart_send_str("COM error", 1);
        __delay_cycles(60000);
        __delay_cycles(60000);
        EEPROM_key_read(data_r, addr);
        for(i = 7; i > 0; i--){
            if(deleted_key[i] != data_r[i]){
                uart_send_str("ERROR: WRITE INTO KEY", 1);
                return 1;
            }
        }
        uart_send_str("KEY SUCCESSFULLY DELETED", 1);

        break;
    default:
        uart_send_str("EEPROM ERROR", 1);

    }
    return 0;
}


/* State functions start___________________________________________________________________________________ */


void access_allow(inputs_t input){
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

void access_denied(inputs_t input){
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

void change_relay_time(){

}

int save_key_ff_mode(){

    uint8_t i;
    for(i = 0; i < iButton_data.buffer_cnt; i += 8)                                             // Key code has already been written into buffer
        if(!compare_key(iButton_data.key_code, iButton_data.buffer + i))
            return 1;
    if(EEPROM_search_key(iButton_data.key_code, EEPROM_LAST_KEY_SPACE))
        return 2;
    if(iButton_data.buffer_cnt >= 64)
        return 3;
    copy_key(iButton_data.key_code, iButton_data.buffer+iButton_data.buffer_cnt);
    iButton_data.buffer_cnt += 8;

    if( !( (iButton_data.first_free_address + iButton_data.buffer_cnt) & 0b0000000000111111 ) ){  // integer multiply of 64, means page border! todo test it
        EEPROM_write_n_byte(iButton_data.buffer, iButton_data.buffer_cnt, iButton_data.first_free_address);
        iButton_data.buffer_cnt = 0;
        __delay_cycles(60000);
        __delay_cycles(60000);
        EEPROM_get_first_free_addr(&iButton_data.first_free_address);
    }
    return 0;
}

void fast_add_mode(inputs_t input){

    uart_send_byte(48+iButton_data.buffer_cnt);
    uart_send_byte(48+iButton_data.first_free_address);
    switch (input) {
        case timeout:
            if(iButton_data.buffer_cnt > 0){
                EEPROM_write_n_byte(iButton_data.buffer, iButton_data.buffer_cnt, iButton_data.first_free_address);
                iButton_data.buffer_cnt = 0;
            }
            led_blink_enable = 0;
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            uart_send_str("Normal mode>", 1);
            ibutton_fsm.current_state = check_touch;
            break;
        case key_touched:
            timeout_ms = 30000;
            // test EEPROM WP
            switch (save_key_ff_mode()) {
                case 0:
                    uart_send_str("Saved.", 1);
                    break;
                case 1:
                    uart_send_str("Already saved.", 1);
                    break;
                case 2:
                    uart_send_str("Already stored.", 1);
                    break;
                default:
                    uart_send_str("HIBA!",1);
                    break;
            }
            break;
        case master_key_touched:
            uart_send_byte(48+iButton_data.buffer_cnt);
            if(iButton_data.buffer_cnt > 0){
                EEPROM_write_n_byte(iButton_data.buffer, iButton_data.buffer_cnt, iButton_data.first_free_address);
                iButton_data.buffer_cnt = 0;
            }
            led_blink_enable = 0;
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

void add_master_key(inputs_t input){
    switch(input){
    case key_touched:
        P1OUT |= BIT0;
        EEPROM_key_write(iButton_data.key_code, EEPROM_MASTER_KEY_PLACE);
        __delay_cycles(60000);
        __delay_cycles(60000);

        if(EEPROM_key_read(iButton_data.master_key_code, EEPROM_MASTER_KEY_PLACE))
            uart_send_str("EEPROM read error.", 1);
        else{
            uart_send_ibutton_data(iButton_data.master_key_code, 1);
            uart_send_str("is master", 1);
            if(compare_key(iButton_data.key_code, iButton_data.master_key_code))
                uart_send_str("ADD M_key error!", 1);
            else{
                uart_send_str("Fast add mode#",1);
                if(EEPROM_get_first_free_addr(&iButton_data.first_free_address)){
                    uart_send_str("First free ERROR", 1);
                    uart_send_byte(48+iButton_data.first_free_address);
                    ibutton_fsm.current_state = check_touch;
                }
                else{
                    timeout_ms = 30000;
                    iButton_data.buffer_cnt = 0;
                    ibutton_fsm.current_state = fast_add_mode;
                }
            }
        }
        break;
    case master_key_touched:
        uart_send_str("Fast add mode#",1);
        timeout_ms = 30000;
        if(EEPROM_get_first_free_addr(&iButton_data.first_free_address)){
            uart_send_str("First free ERROR", 1);
            uart_send_byte(48+iButton_data.first_free_address);
            ibutton_fsm.current_state = check_touch;
        }
        else{
            iButton_data.buffer_cnt = 0;
            ibutton_fsm.current_state = fast_add_mode;
            break;
        }
    }
    ibutton_fsm.input_to_serve = 0;
}

void master_delete(inputs_t input){
    switch (input) {
        case timeout:
            led_blink_enable = NONE;
            LED_TURN_OFF_RE;
            LED_TURN_ON_GR;
            ibutton_fsm.current_state = check_touch;
            break;
        case master_key_touched:
            LED_TURN_OFF_GR;
            led_blink_enable = ONLY_RED;
            EEPROM_clear_ff();
            __delay_cycles(60000);
            __delay_cycles(60000);
            iButton_data.master_key_code[0] = 0xFF;
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

void master_mode(inputs_t input){
    ibutton_fsm.input_to_serve = 0;
    switch(input){
    case timeout:
        uart_send_str("Normal mode>", 1);
        led_blink_enable = 0;
        LED_TURN_ON_GR;
        // todo stop blinking, reset blinking timer variable
        ibutton_fsm.current_state = check_touch;
        break;
    case master_key_touched:
        uart_send_str("Erase EEPROM?", 1);
        LED_TURN_OFF_RE;
        LED_TURN_ON_GR;
        led_blink_enable = BOTH;
        timeout_ms = 3000;
        ibutton_fsm.current_state = master_delete;
        break;
    case key_touched:
        search_key_and_write();
        led_blink_enable = 0;
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        ibutton_fsm.current_state = check_touch;
        break;
    }
}

void check_touch(inputs_t input){

    uint16_t addr = 0;
    uint16_t addr_ff = 0;

    timeout_ms = 0;

    switch (ibutton_fsm.input){
    case key_touched:
        if(EEPROM_get_key_or_empty_place(iButton_data.key_code, &addr, &addr_ff, EEPROM_LAST_KEY_SPACE, 0) == 1){
            uart_send_str("RELAY=ON", 1);
            // \todo Check opening time settings here.
            timeout_ms = 2000;
            LED_TURN_OFF_GR;
            REL_ON;
            ibutton_fsm.current_state = access_allow;
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
        uart_send_str("RELAY=ON", 1);
        timeout_ms = 2000;
        REL_ON;
        ibutton_fsm.current_state = access_allow;
        ibutton_fsm.input_to_serve = 0;
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

/* State functions stop____________________________________________________________________________________ */
