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



ibutton_fsm_t ibutton_fsm;

void ibutton_fsm_init(){
    ibutton_fsm.current_state = check_touch;
}

void put_input(inputs_t input){
    fsm_input_flag = 1;
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
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        ibutton_fsm.current_state = check_touch;
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
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        ibutton_fsm.current_state = check_touch;
        break;
    default:
        uart_send_str("EEPROM ERROR", 1);
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        ibutton_fsm.current_state = check_touch;
    }
    return 0;
}


/* State functions start___________________________________________________________________________________ */
void add_master_key(inputs_t input){
    LED_TURN_ON_RE;
    LED_TURN_OFF_GR;
    switch(input){
    case key_touched:
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
                LED_TURN_OFF_RE;
                LED_TURN_ON_GR;
                fsm_input_flag = 0;
                ibutton_fsm.current_state = check_touch;
            }
        }
    }
}

void access_allow(inputs_t input){
    REL_ON;
    LED_TURN_OFF_GR;
    if(input == timeout){
        uart_send_str("RELAY=OFF", 1);
        LED_TURN_ON_GR;
        REL_OFF;
        ibutton_fsm.current_state = check_touch;
        fsm_input_flag = 0;
    }
}

void access_denied(inputs_t input){
    if(input == timeout){
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        ibutton_fsm.current_state = check_touch;
        fsm_input_flag = 0;
    }
}
/**
 *
 * */
/*void add_key_mode(){
    uint8_t number;

    uint8_t data_r[8];
    uint8_t i;
    uint16_t addr = 0;
    uint16_t addr_ff = 0;
    uint8_t deleted_key[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    uint16_t ret = 0;

    if(timeout_flag){
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        timeout_flag = 0;
        timeout_ms = 0;
        g_current_state = check_touch;
    }
    else if(uart_get_buffer_bytes() >= 2){
        char str[] = "00";

        str[0] = (char)uart_get_byte();
        str[1] = (char)uart_get_byte();
        uart_send_str(str, 0);
        if(!hex_char_to_number(str[0], str[1], &number))
            iButton_data.key_code[iButton_data.key_code_n++] = number;
        else{
            uart_send_str("Invalid number!", 1);
            iButton_data.key_code_n = 0;
            iButton_data.compare_flag = 0;
        }

        if(iButton_data.key_code_n > 7){
            uart_send_str("->",1);
            iButton_data.key_code_n = 0;
            if(iButton_data.key_code[0] != 0x01){
                uart_send_str("Invalid key code: family code.", 1);
                LED_TURN_OFF_RE;
                LED_TURN_ON_GR;
                g_current_state = check_touch;
            }else if(ibutton_crc8(iButton_data.key_code)) {
                uart_send_str("Invalid key code: CRC.", 1);
                LED_TURN_OFF_RE;
                LED_TURN_ON_GR;
                g_current_state = check_touch;
            }else{
                search_key_and_write(); // todo test it
            }
        }
    }
}
*/
void change_relay_time(){

}

/*void wait_for_master_key(){
    char MSch, LSch;
    uint8_t number;

    if(timeout_flag){
        uart_send_str("Timeout", 1);
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        timeout_flag = 0;
        g_current_state = check_touch;
    }
    else if(uart_get_buffer_bytes() >= 2){
        timeout_ms = 30000;
        if(MSch = uart_get_byte() == 'v' || LSch = uart_get_byte() == 'v'){
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            iButton_data.key_code_n = 0;
            iButton_data.compare_flag = 0;
            uart_send_str("Check touch", 1);
            g_current_state = check_touch;

            char a,b;
            hex_byte_to_char(number, &a, &b);
            uart_send_byte((char)a);
            uart_send_byte((char)b);

        }
        hex_char_to_number(MSch, LSch, &number);






        if(number != iButton_data.master_key_code[iButton_data.key_code_n])
            iButton_data.compare_flag++;
        if(iButton_data.key_code_n++ >= 7){     // Comparing end
            if(!iButton_data.compare_flag){     // Master code ok
                uart_send_str("->", 1);
                uart_send_str("Wrong master code!", 1);
                iButton_data.key_code_n = 0;
                iButton_data.compare_flag = 0;
                uart_send_str(' ', 1);
            }else{
                iButton_data.key_code_n = 0;
                iButton_data.compare_flag = 0;
                uart_send_str("->", 1);
                uart_send_str("Wrong master code!", 1);
                LED_TURN_ON_GR;
                g_current_state = check_touch;
            }
        }
    }
} */


void master_mode(inputs_t input){
    fsm_input_flag = 0;
    switch(input){
    case timeout:
        uart_send_str("Normal mode>", 1);
        led_blink_enable = 0;
        LED_TURN_ON_GR;
        // todo stop blinking, reset blinking timer variable
        ibutton_fsm.current_state = check_touch;
        break;
    case key_touched:
        if(!compare_key(iButton_data.master_key_code, iButton_data.key_code)){           // If master key.
            uart_send_str("Normal mode>", 1);
            led_blink_enable = 0;
            LED_TURN_ON_GR;
            ibutton_fsm.current_state = check_touch;
        }
        else
            search_key_and_write();

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

        }else if(!compare_key(iButton_data.key_code, iButton_data.master_key_code)){
            uart_send_str("Master mode#", 1);
            timeout_ms = 30000;
            led_blink_enable = 1;
            //REL_ON;
            ibutton_fsm.current_state = master_mode;
        }
        else{
            uart_send_str("ACCESS DENIED!",1);
            timeout_ms = 1000;
            LED_TURN_OFF_GR;
            LED_TURN_ON_RE;
            ibutton_fsm.current_state = access_denied;
        }
        fsm_input_flag = 0;
        break;
    case button_pressed:
        uart_send_str("RELAY=ON", 1);
        timeout_ms = 2000;
        REL_ON;
        ibutton_fsm.current_state = access_allow;
        fsm_input_flag = 0;
        break;
    }
}

/* State functions stop____________________________________________________________________________________ */
