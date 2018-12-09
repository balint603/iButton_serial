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
#define ONLY_NOISE 4

#define SHORT 256
#define LONG 1200




/** State functions */
static void shorted_reader(inputs_t input);
static void access_allow_bistable(inputs_t input);
static void access_allow(inputs_t input);
static void access_denied(inputs_t input);
static void fast_add_mode(inputs_t input);
static void master_delete(inputs_t input);
static void master_mode(inputs_t input);
static void add_master_key(inputs_t input);
static void check_touch(inputs_t input);

iButton_key_data_t iButton_data = {
                                    .master_key_code_ptr = (uint16_t*)SETTINGS_START,
                                    .opening_time_ptr = (uint16_t*)FLASH_TIME_ADDR,
                                    .mode_ptr = (uint16_t*)FLASH_MODE_ADDR,
                                    .reader_enable_flag = 1,
};



/** iButton reading. */
uint8_t prev_presence = 0;
uint8_t curr_presence = 0;

uint_fast16_t LED_ms = 125;
volatile uint8_t LED_flag;

volatile uint_fast16_t timeout_ms;
volatile uint8_t timeout_ticking;
volatile uint_fast16_t led_blink_ms = 500;
volatile uint8_t led_blink_flag;
volatile uint8_t user_feedback;

volatile uint16_t piezo_on_time;
volatile uint8_t piezo_ticking;

uint8_t sound_mode = 1;



#define TIMEOUT(time) {timeout_ms = time; timeout_ticking = 1;}



/**
 * Initialization.
 * If the
 * */
void ibutton_fsm_init(){
    uint16_t data_ff[] = {0xFFFF,0xFFFF,0xFFFF};
    uint16_t basic_time = 30;

    PIEZO_PORT_DIR |= PIEZO_BIT;
    P2SEL &= ~BIT7;
    GPIO_SET_INPUT_PULL_UP_VCC(JUMPER_A_PORT,JUMPER_A_PIN);
    GPIO_SET_INPUT_PULL_UP_VCC(JUMPER_B_PORT,JUMPER_B_PIN);
    GPIO_SET_INPUT_PULL_UP_VCC(JUMPER_M_PORT,JUMPER_M_PIN);

    GPIO_SET_INPUT_PULL_UP_VCC(PUSHBUTTON_PORT,PUSHBUTTON_PIN);

    P2OUT &= ~REL_BIT;  // relay
    REL_OFF;

    reader_polling_ms = READ_POLLING_TIME;
    reader_polling_flag = 1;

    ibutton_fsm.input = key_away;
    ibutton_fsm.input_to_serve = 0;
    ibutton_fsm.current_state = check_touch;

    if((*iButton_data.opening_time_ptr) == 0xFFFF)                              // Opening time empty?
        if(flash_change_settings(FLASH_TIME, &basic_time, 1)){
            uart_send_str("Change settings error!", 1);
            make_sound(0, LONG);
        }
    refresh_timing();

    if(!compare_key(data_ff, (uint16_t*)iButton_data.master_key_code_ptr)){     // Master code place empty?
        uart_send_str("First start.", 1);

        // todo memory check

        uart_send_str("Please touch a master key!", 1);
        user_feedback = ONLY_GREEN;
        ibutton_fsm.current_state = add_master_key;
    }
    else if( !(GET_INPUT) && GPIO_GET_INPUT(JUMPER_M_PORT,JUMPER_M_PIN) ){       // Check shorted and enabled
        user_feedback = ONLY_GREEN;
        ibutton_fsm.current_state = shorted_reader;
        TIMEOUT(1200);
        make_sound(0, LONG);
    }
    LED_TURN_OFF_RE;
    make_sound(0, SHORT);
}

void read_pushbutton(){
    if( !(GPIO_GET_INPUT(PUSHBUTTON_PORT,PUSHBUTTON_PIN) || ibutton_fsm.input_to_serve) ){
        put_input(button_pressed);
    }
}

void ibutton_read(){

    if( curr_presence = ibutton_test_presence()){
        if(!iButton_data.reader_enable_flag){
            reader_disable_ms = READ_DISABLE_TIME;
            read_pushbutton();
        }
        else if(!ibutton_read_it(iButton_data.key_code)){
            if(compare_key((uint16_t*)iButton_data.master_key_code_ptr, iButton_data.key_code))
                put_input(key_touched);
            else
                put_input(master_key_touched);
            ibutton_fsm.input_to_serve = 1;
            iButton_data.reader_enable_flag = 0;
        }
    }
    else if(prev_presence){
        reader_disable_ms = READ_DISABLE_TIME;
    }
    else
        read_pushbutton();
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
/**
 * \param mode: toggle or reset/set output mode.
 * */
void make_sound(uint8_t mode, uint16_t time){
    if(mode)
        TA0CCTL1 = OUTMOD_4;
    else
        TA0CCTL1 = OUTMOD_7;
    PIEZO_PORT_DIR |= PIEZO_BIT;
    piezo_on_time = time;
    piezo_ticking = 1;
    PIEZO_PORT_SEL |= PIEZO_BIT; // start beep
}

void refresh_timing(){
    iButton_data.opening_time = 0;
    if(GPIO_GET_INPUT(JUMPER_A_PORT, JUMPER_A_PIN)){
        if(GPIO_GET_INPUT(JUMPER_B_PORT, JUMPER_B_PIN))
            iButton_data.opening_time = 150;
        else
            iButton_data.opening_time = 100;
    }
    else{
        if(GPIO_GET_INPUT(JUMPER_B_PORT, JUMPER_B_PIN))
            iButton_data.opening_time = 50;
        else
            iButton_data.opening_time = *(iButton_data.opening_time_ptr);
    }
}

void ibutton_user_feedback_service(){
    switch(user_feedback){
        case 1:
            LED_BLINK_GR;
            break;
        case 2:
            LED_BLINK_RE;
            break;
        case 3:
            sound_mode = !sound_mode;
            make_sound(sound_mode, 120);
            LED_BLINK_GR;
            LED_BLINK_RE;
            break;
        case 4:
            make_sound(0, 80);
        default:
        break;
    }
}


/* State functions start___________________________________________________________________________________ */

static void shorted_reader(inputs_t input){
    if(!timeout_ticking){
        if(!GET_INPUT && GPIO_GET_INPUT(JUMPER_M_PORT,JUMPER_M_PIN)){                  // Still pulled down and enabled
            ibutton_fsm.current_state = add_master_key;
            user_feedback = ONLY_GREEN;
            TIMEOUT(60000);
        }
        else{
            ibutton_fsm.current_state = check_touch;
            LED_TURN_ON_GR;
            user_feedback= NONE;
        }
    }
}

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
    if(!timeout_ticking){
        if(!(--iButton_data.opening_time)){
            uart_send_str("RELAY=OFF", 1);
            make_sound(0, 200);
            LED_TURN_ON_GR;
            user_feedback = NONE;
            REL_OFF;
            ibutton_fsm.current_state = check_touch;
            refresh_timing();
        }else
            TIMEOUT(200);   // Wait 100ms again...
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
        uart_send_str("RELAY=ON", 1);
        REL_ON;
        led_blink_ms = 2000;
        make_sound(1, SHORT);
        user_feedback = ONLY_NOISE;
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
            user_feedback = 0;
            led_blink_ms = 1000;
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            make_sound(1, LONG);
            uart_send_str("Normal mode>", 1);
            ibutton_fsm.current_state = check_touch;
            break;
        case key_touched:
            TIMEOUT(60000);
            // test EEPROM WP
            LED_TURN_ON_RE;
            if(flash_search_key(iButton_data.key_code, &address)){
                uart_send_str("Already added!", 1);
                make_sound(1, SHORT);
            }
            else{
                flash_write_data(iButton_data.key_code, 3, address);
            }
            flash_ptr = (uint16_t*)address;
            if(compare_key(flash_ptr, iButton_data.key_code)){
                uart_send_str("Flash write error!",1);
                make_sound(0, LONG);
            }
            else
                make_sound(1, SHORT);
            LED_TURN_OFF_RE;
            break;
        case master_key_touched:
            user_feedback = 0;
            led_blink_ms = 1000;
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            uart_send_str("Normal mode>", 1);
            make_sound(1, SHORT);
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
        if(flash_change_settings(FLASH_MASTER_CODE, iButton_data.key_code, 3))
            uart_send_str("Change settings error!", 1);
        if(!compare_key(iButton_data.key_code, iButton_data.master_key_code_ptr)){
            uart_send_str("Fast add mode#", 1);
            make_sound(1, SHORT);
            TIMEOUT(60000);
            ibutton_fsm.current_state = fast_add_mode;
        }
        else{
            uart_send_str("ADD M_key error!", 1);
            ibutton_fsm.current_state = check_touch;
        }
        break;
    case master_key_touched:
        uart_send_str("Fast add mode#",1);
        make_sound(0, SHORT);
        TIMEOUT(60000);
        ibutton_fsm.current_state = fast_add_mode;
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

static void master_delete(inputs_t input){
    switch (input) {
        case timeout:
            user_feedback = NONE;
            LED_TURN_OFF_RE;
            LED_TURN_ON_GR;
            make_sound(1, LONG);
            ibutton_fsm.current_state = check_touch;
            break;
        case master_key_touched:
            make_sound(1, 1000);
            LED_TURN_OFF_GR;
            LED_TURN_ON_RE;
            segment_erase(0);   // All segment erase!
            segment_erase(FLASH_MASTER_CODE);       // Erase information memory.
            flash_write_word(OPENING_TIME_BASIC, FLASH_TIME);
            flash_init();
            LED_TURN_OFF_RE;
            LED_TURN_ON_GR;
            user_feedback = ONLY_GREEN;
            uart_send_str("Please touch a master key!", 1);
            ibutton_fsm.current_state = add_master_key;
            make_sound(1, 1);
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
        user_feedback = 0;
        LED_TURN_ON_GR;
        make_sound(1, LONG);
        led_blink_ms = 1000;
        ibutton_fsm.current_state = check_touch;
        break;
    case master_key_touched:
        uart_send_str("Erase key code data?", 1);
        LED_TURN_OFF_RE;
        LED_TURN_ON_GR;
        user_feedback = BOTH;
        TIMEOUT(6000);
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
        user_feedback = NONE;
        make_sound(1, SHORT);
        led_blink_ms = 1000;
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
            uart_send_str("Master mode#", 1);
            make_sound(1, SHORT);
            TIMEOUT(60000);
            user_feedback = ONLY_GREEN;
            LED_TURN_OFF_GR;
            ibutton_fsm.current_state = master_mode;

        }
        else{
            uart_send_str("RELAY=ON", 1);
            REL_ON;
            led_blink_ms = 2000;
            make_sound(1, SHORT);
            user_feedback = ONLY_NOISE;
            if(*iButton_data.mode_ptr){
                ibutton_fsm.current_state = access_allow;
                refresh_timing();
                TIMEOUT(200);
            }
            else{
                ibutton_fsm.current_state = access_allow_bistable;
            }
            LED_TURN_OFF_GR;
        }
        break;
    case key_touched:
        if(flash_search_key(iButton_data.key_code, &addr)){
            uart_send_str("RELAY=ON", 1);
            REL_ON;
            led_blink_ms = 2000;
            make_sound(1, SHORT);
            user_feedback = ONLY_NOISE;
            if(*iButton_data.mode_ptr){
                ibutton_fsm.current_state = access_allow;
                refresh_timing();
                TIMEOUT(200);
            }
            else{
                ibutton_fsm.current_state = access_allow_bistable;
            }
            LED_TURN_OFF_GR;
        }
        else{
            uart_send_str("ACCESS DENIED!",1);
            LED_TURN_OFF_GR;
            LED_TURN_ON_RE;
            make_sound(0, 2000);
            ibutton_fsm.current_state = access_denied;
        }
            break;
    case button_pressed:
        if(*iButton_data.button_enable_ptr){
            uart_send_str("RELAY=ON", 1);
            REL_ON;
            led_blink_ms = 2000;
            make_sound(1, SHORT);
            user_feedback = ONLY_NOISE;
            if(*iButton_data.mode_ptr){
                ibutton_fsm.current_state = access_allow;
                refresh_timing();
                TIMEOUT(200);
            }
            LED_TURN_OFF_GR;
        }
        else
            uart_send_str("Button disabled.", 1);

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
        LED_ms = 125;
    }
    if(timeout_ticking){
        if(!timeout_ms--){
            put_input(timeout);
            ibutton_fsm.input_to_serve = 1;
            timeout_ticking = 0;
        }
    }

    if(piezo_ticking){
        if(!piezo_on_time--){
             PIEZO_PORT_SEL &= ~PIEZO_BIT;
             PIEZO_PORT_DIR &= ~PIEZO_BIT;

             piezo_ticking = 0;
        }
    }
    switch(user_feedback){
    case NONE:
        break;
    case ONLY_NOISE:
        if(!(--led_blink_ms)){
            led_blink_flag = 1;
            led_blink_ms = 2000;
        }
        break;
    default:
        if(!(--led_blink_ms)){
            led_blink_flag = 1;
            led_blink_ms = 1000;
        }
        break;
    }

    if( !( --reader_polling_ms )){
        reader_polling_flag = 1;
        reader_polling_ms = READ_POLLING_TIME;
    }
    if( !(--reader_disable_ms)){
        iButton_data.reader_enable_flag = 1;
        put_input(key_away);
    }
}
