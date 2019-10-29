/**
 * fsm.c
 *
 *  Created on: 2018. okt. 27.
 *      Author: E-Kontakt Bt.
 *  Description:
 *  This module is a finite state machine. Using state functions and a global pointer which points to a state function.
 */

#include <msp430.h>
#include <inttypes.h>
#include "fsm.h"
#include "uart.h"
#include "flash.h"
#include "ibutton.h"

/** LED blink modes. */
#define INFO_NONE       0
#define INFO_2_BEEPS    2
#define INFO_3_BEEPS    3
#define INFO_ONLY_GREEN 4
#define INFO_ONLY_RED   5
#define INFO_BOTH       6
#define INFO_BOTH_ONLY_LIGHT 7

/** Length of beeping. */
#define INFO_SHORT      200
#define INFO_LONG       1600


/** \brief Special object for FSM.
 *  Contain informations about keys, operating modes.
 *  Variable mode_ptr: If this equals to:
 *      - 0xFFFF - normal mode is active,
 *      - 0xAAAA - bistable,
 *      - 0x5555 - bistable closing gate by the same key.
 */
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

/** Input types of state machine. */
typedef enum inputs {
    key_touched,
    master_key_touched,
    button_pressed,
    key_away,
    timeout,
    relay_timeout
} inputs_t;

/** Pointer to the actual state. */
typedef void ( *p_state_handler )(inputs_t input);

typedef struct fsm{
    inputs_t input;
    p_state_handler current_state;
    uint8_t input_to_serve;
} ibutton_fsm_t;

/** \brief Constant pointers initialization.
 *  These pointers are references to flash memory places. At these flash memory places, settings can be find.
 */
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

/** VARIABLES______________________________________________________________________________________________________ */
/** Key touch flags. */
uint8_t prev_presence = 0;
uint8_t curr_presence = 0;

/** FSM timeout counter. */
volatile uint_fast16_t timeout_ms;
/** FSM timeout counter is going flag. */
volatile uint8_t timeout_ticking;
/** UART timeout counter. */
volatile uint16_t uart_timeot_ms = 1000;
/** UART timeout counter is going flag. */
volatile uint8_t uart_timeout_ticking;

/** User information program. */
volatile uint_fast16_t user_info_ms;
volatile uint8_t user_info_flag;
volatile uint8_t user_info_mode;
volatile uint16_t piezo_on_time;
volatile uint8_t piezo_ticking;

/** UART MODULE. */
volatile uint8_t RX_is_packet;
volatile uint8_t TX_is_packet;
volatile packet_t RX_packet;

/** iButton reader flags, counters. */
ibutton_fsm_t ibutton_fsm;
uint_fast16_t reader_polling_ms;
volatile uint8_t reader_polling_flag;
uint_fast16_t reader_disable_ms;

/** VARIABLES end__________________________________________________________________________________________________ */


/** MACRO__________________________________________________________________________________________________________ */
/** Set the FSM timer. */
#define TIMEOUT(time) {timeout_ms = time; timeout_ticking = 1;}

/** Send user information, lights and/or sound. */
#define SEND_USER_INFO(mode,sound_mode,sound_length)\
                        make_sound(sound_mode, sound_length);\
                        user_info_mode = mode;\
                        user_info_flag = 1;

/** Put input to FSM. */
#define PUT_INPUT(put_this) {ibutton_fsm.input = put_this; ibutton_fsm.input_to_serve = 1;}

/** Get the door opening pushbutton state. */
#define READ_PUSHBUTTON {\
    if( !(GPIO_GET_INPUT(PUSHBUTTON_PORT,PUSHBUTTON_PIN) || ibutton_fsm.input_to_serve) ){\
        PUT_INPUT(button_pressed);\
    }\
}
/** MACRO end______________________________________________________________________________________________________ */


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

/** Refresh the current time settings, set by jumpers on the board. */
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

/** Make noise with PWM settings. Change the current port settings to output. */
static void make_sound(uint8_t mode, uint16_t time){
    if(mode)
        TA0CCTL1 = OUTMOD_4;
    else
        TA0CCTL1 = OUTMOD_7;
    PIEZO_PORT_DIR |= PIEZO_BIT;
    piezo_on_time = time;
    PIEZO_PORT_SEL |= PIEZO_BIT; // start beep
}

/** \brief Key code save or delete.
 *  Delete the parameter key code from the flash memory if it exists,
 *  save to flash if it does not.
 *  \param key_code will be saved or deleted.
 */
static void delete_or_add_key(uint16_t *key_code){
    uint16_t address;
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
}

/** Copy the content of a word array to a byte array and change order.
 *  \param wordarr pointer to a word array size: n_words.
 *  \param bytearr pointer to a byte array size: n_words * 2.
 *  \param n_words size of arrays in words.
 * */
void copy_wordarr_to_bytearr(uint16_t *wordarr, uint8_t *bytearr, uint8_t n_words) {
    if ( !(wordarr && bytearr) )
        return;
    while ( n_words-- ) {
        *(bytearr++) = (uint8_t) (*(wordarr));
        *(bytearr++) = (uint8_t) (*(wordarr++) >> 8);
    }
}

/** STATIC functions end___________________________________________________________________________________ */


/** PUBLIC functions_______________________________________________________________________________________ */


/** \brief Check if there is an input to serve. */
uint8_t ibutton_fsm_is_input() {
    return ibutton_fsm.input_to_serve ? 1 : 0;
}

/** \brief Check if the polling of the reader is enable. */
uint8_t ibutton_fsm_get_polling_flag() {
    return reader_polling_flag;
}

/** \brief Check if there is a packet to process. */
uint8_t ibutton_fsm_is_packet() {
    return RX_is_packet;
}

/**
 * \brief Initialization of the state machine.
 * A lot of initial processes run:
 *      - Initialize reader timer.
 *      - Bring FSM into starter state.
 *      - Save default opening time if it has not set yet.
 *      - Check if the memory place of master key,
 *          - if it is cleared to full of 0xFFFF, FSM will jump to the key adding state,
 *          - when the reader is shorted, FSM state will be shorted_reader to enable master key add.
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
        LED_TURN_ON_RE;
        LED_TURN_OFF_GR;
    }

}

/** \brief iButton reader function to check the touches.
 * When an iButton device is successfully read, reader polling will be stopped for defined time (READ_DISABLE_TIME).
 * Reset the defined at every iButton presence.
 * Clear the polling flag set by timer.
 */
void ibutton_read() {

     if ( curr_presence = ibutton_test_presence() ) {
        if( !iButton_data.reader_enable_flag ) {       // Disabled reader
            reader_disable_ms = READ_DISABLE_TIME;  // Disable again for defined seconds.
            READ_PUSHBUTTON;
        }
        else if ( !ibutton_read_it(iButton_data.key_code) ) {
            if ( compare_key((uint16_t*)iButton_data.master_key_code_ptr, iButton_data.key_code) ) {
                PUT_INPUT(key_touched);
            }
            else {
                PUT_INPUT(master_key_touched);
            }
            ibutton_fsm.input_to_serve = 1;
            iButton_data.reader_enable_flag = 0;
        }
    }
    else if ( prev_presence ) {     // True value means that the key is removed from reader.
        reader_disable_ms = READ_DISABLE_TIME;
    }
    else{
        READ_PUSHBUTTON;
    }
    prev_presence = curr_presence;
    reader_polling_flag = 0;
}

/** Check the input, change the state of FSM. */
void ibutton_fsm_change_state( ){
    ibutton_fsm.current_state(ibutton_fsm.input);
}

/** \brief Send informations to user.
 * \param mode: toggle or reset/set output mode.
 * */
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

/** Send settings via UART. */
void send_settings_data() {
    uint8_t *data = (uint8_t*)(FLASH_MASTER_CODE + SETTINGS_START);
    uart_send_packet(data, TYPE_GET_SETTINGS_RE, SETTINGS_RANGE * 2);
}

/** Overwrite settings. */
void write_settings_data() {
    uint8_t info_msg;
    int ret;
    if ( RX_packet.data_size != SETTINGS_RANGE * 2 ) {
        info_msg = MSG_ERR_SIZE;
    } else {
        ret = flash_change_settings(FLASH_MASTER_CODE, (uint16_t*)RX_packet.data, SETTINGS_RANGE);
        switch ( ret ) {
        case -1:
            info_msg = MSG_ERR_DATA;
            break;
        case 1:
            info_msg = MSG_ERR_WRITE;
            break;
        case 0:
            info_msg = MSG_OK;
        default: break;
        }
    }
    uart_send_packet(&info_msg, TYPE_INFO, 1);
}

static void send_err_packet(uint8_t data) {
    uart_send_packet(&data, TYPE_INFO, 1);
}

/** \brief Procedure to make pointer from a two byte large address array. LSB first!. */
uint8_t *make_address(uint8_t addr_arr[]) {
    uint16_t temp_addr;

    temp_addr = (uint16_t)(addr_arr[1]);  // Get request data is an address
    temp_addr <<= 8;
    temp_addr |= (uint16_t)(addr_arr[0]);
    return (uint8_t*)temp_addr;
}

/**
 * Process the TYPE_GET_FLASHSEGM command.
 * First, make address from data.
 * */
static void process_get_segment(packet_t *RX_packet) {
    uint8_t *flash_ptr = make_address(RX_packet->data);
    if ( (uint8_t*)SEGMENT_0 <= flash_ptr && flash_ptr <= (uint8_t*)(SEGMENT_8 + 512) )
        uart_send_flash_segment(flash_ptr);
    else
        send_err_packet(MSG_ERR_RANGE);
}

/**
 * Process the TYPE_WRITE_FLASHSEGM command.
 */
void process_write_to_flash(packet_t *RX_packet) {
    uint8_t *flash_ptr = make_address(RX_packet->data); // TODO RESPONSE
    /*if ( (uint8_t*)SEGMENT_0 <= flash_ptr && flash_ptr <= (uint8_t*)(SEGMENT_8 + 512) )
        //flash_write_data(data, size, address)   // TODO response
    else
        send_err_packet(ERR_RANGE);*/
}

/** \brief Processing incoming UART commands.
 * See defined commands.
 * */
void ibutton_process_command() {
    if ( RX_packet.data_size > RX_DATA_SIZE ) {
        send_err_packet(MSG_ERR_SIZE);
        return;
    }

    uint16_t crc_val = 0xFFFF;
    uint8_t preamble[2];
    preamble[0] = RX_packet.type_b;
    preamble[1] = RX_packet.data_size;
    crc_do(preamble, 2, &crc_val);
    crc_do(RX_packet.data, RX_packet.data_size, &crc_val);
    if ( crc_val != RX_packet.crc ) {               // CRC check
        send_err_packet(MSG_ERR_CRC);
    }
    switch ( RX_packet.type_b ) {                   // TYPE selector
        case TYPE_ECHO:
            if ( !uart_send_packet(RX_packet.data, RX_packet.type_b, RX_packet.data_size) )
                ;

            break;
        case TYPE_TEST:
            PUT_INPUT(button_pressed);
            break;
        case TYPE_GET_SETTINGS:
            send_settings_data();
            break;
        case TYPE_WRITE_SETTINGS:
            write_settings_data();
            break;
        case TYPE_ERASE_ALL:
            segment_erase(0);
            uart_send_packet(0, TYPE_ERASE_ALL_RE, 1);
            break;
        case TYPE_WRITE_A_KEY:
            delete_or_add_key((uint16_t*)RX_packet.data);
            break;
        case TYPE_GET_FLASHSEGM:
            if ( RX_packet.data_size == 2 )
                process_get_segment(&RX_packet);
            else
                send_err_packet(MSG_ERR_SIZE);
            break;
        case TYPE_WRITE_FLASHSEGM:
            process_write_to_flash(&RX_packet);
            break;
        default:

            break;
    }
    RX_is_packet = 0;
}

/**
 * \return 0  Key codes match.
 * \return 1  Key codes do not match.
 * \return -1 Null parameters.
 * */
int compare_key(uint16_t *key1, uint16_t *key2) {
    uint8_t i;
    if ( !(key1 && key2) )
        return -1;
    for (i=3; i > 0; i--)
        if ( *(key1++) != *(key2++) )
            return 1;
    return 0;
}

int copy_key(uint16_t *from, uint16_t *to) {
    if(!(from || to))
        return 1;
    uint8_t i;
    for(i = 3; i > 0; i--)
        *(to++) = *(from++);
    return 0;
}
/** PUBLIC functions end___________________________________________________________________________________ */




/* State functions start___________________________________________________________________________________ */

/**
 * Fsm get to this state when reader was being shorted at initialization.
 * Wait 1 second and check the reader again, it must be still shorted to get to add_master_key state.
 */
static void shorted_reader(inputs_t input) {
    if( !timeout_ticking ) {
        if( !GET_INPUT && GPIO_GET_INPUT(JUMPER_M_PORT,JUMPER_M_PIN) ) {                  // Still pulled down and enabled
            ibutton_fsm.current_state = add_master_key;
            user_info_mode = INFO_BOTH_ONLY_LIGHT;
            user_info_flag = 1;
            LED_TURN_ON_GR;
            LED_TURN_ON_RE;
        }
        else {
            ibutton_fsm.current_state = check_touch;
            LED_TURN_ON_RE;
            LED_TURN_OFF_GR;
            user_info_mode = INFO_NONE;
        }
    }
}

/**
 * Gate is opened in this state until a valid key has been touched.
 */
static void access_allow_bistable(inputs_t input) {
    uint16_t addr = SEGMENT_0;
    REL_ON;
    LED_TURN_OFF_GR;

    switch (input){
    case key_touched:
        if( (flash_search_key(iButton_data.key_code, &addr)) ) {
            //uart_send_str("RELAY=OFF", 1);
            LED_TURN_ON_RE;
            make_sound(1,INFO_SHORT);
            REL_OFF;
            ibutton_fsm.current_state = check_touch;
        }
        else {
            //uart_send_str("Wrong key!",1);
            SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
        }
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

/**
 * Gate is opened until the key has touched which opened the gate.
 */
static void access_allow_bistable_same_key(inputs_t input) {
    REL_ON;
    LED_TURN_OFF_GR;
    switch (input) {
    case key_touched:
        if( !compare_key(iButton_data.prev_key_code, iButton_data.key_code) ) {
            //uart_send_str("RELAY=OFF", 1);
            LED_TURN_ON_RE;
            make_sound(1,INFO_SHORT);
            REL_OFF;
            ibutton_fsm.current_state = check_touch;
        }
        else {
            //uart_send_str("Wrong key!",1);
            SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
        }
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

/**
 * Gate is opened for a time.
 */
static void access_allow(inputs_t input) {
    REL_ON;
    if( !timeout_ticking ) {
        if( !(--iButton_data.opening_time) ) {
           // uart_send_str("RELAY=OFF", 1);
            SEND_USER_INFO(INFO_NONE,0,INFO_SHORT);
            LED_TURN_ON_RE;
            LED_TURN_OFF_GR;
            // test uart

            REL_OFF;
            ibutton_fsm.current_state = check_touch;
            refresh_timing();
        } else {
            make_sound(0, INFO_SHORT);
            TIMEOUT(1000);   // Wait 500ms again...
        }
    }
    ibutton_fsm.input_to_serve = 0;
}

/**
 * Invalid key has been touched. Make some noise.
 */
static void access_denied(inputs_t input) {
    switch (input) {
    case key_away:
        TIMEOUT(2000);
        break;
    case timeout:
        LED_TURN_ON_RE;
        LED_TURN_OFF_GR;
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

/**
 * \brief Special key saver mode.
 * Keys can be saved by touching them to the reader sequentially.
 * All touched key will be safe (beeping sound) if any error does not occurs.
 */
static void fast_add_mode(inputs_t input) {
    uint16_t address;
    uint16_t *flash_ptr;
    switch (input) {
        case timeout:
            LED_TURN_ON_RE;
            LED_TURN_OFF_GR;
            SEND_USER_INFO(INFO_NONE,1,INFO_LONG);
            //uart_send_str("Normal mode>", 1);
            ibutton_fsm.current_state = check_touch;
            break;
        case key_touched:
            TIMEOUT(60000);
            LED_TURN_ON_RE;
            if ( flash_search_key(iButton_data.key_code, &address) ) {
                //uart_send_str("Already added!", 1);
                make_sound(0, INFO_SHORT);
            }
            else {
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
            LED_TURN_ON_RE;
            LED_TURN_OFF_GR;
            //uart_send_str("Normal mode>", 1);
            SEND_USER_INFO(INFO_NONE,0,INFO_SHORT);
            ibutton_fsm.current_state = check_touch;
            break;
        default:
            break;
    }
    ibutton_fsm.input_to_serve = 0;
}

/**
 * Change master key by key touch.
 * If the current master key touched, next state is fast_add_mode.
 */
static void add_master_key(inputs_t input) {
    switch (input) {
    case key_touched:
        if ( flash_change_settings(FLASH_MASTER_CODE, iButton_data.key_code, 3) ) {
            SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
            //uart_send_str("Change settings error!", 1);
        }
        if( !compare_key(iButton_data.key_code, iButton_data.master_key_code_ptr) ) {
            //uart_send_str("Fast add mode#", 1);
            SEND_USER_INFO(INFO_ONLY_GREEN,0,INFO_SHORT);
            TIMEOUT(60000);
            ibutton_fsm.current_state = fast_add_mode;
        }
        else {
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

/**
 * All segment erase state.
 * */
static void master_delete(inputs_t input) {
    switch (input) {
        case timeout:
            LED_TURN_ON_RE;
            LED_TURN_OFF_GR;
            SEND_USER_INFO(INFO_NONE,1,INFO_LONG);
            ibutton_fsm.current_state = check_touch;
            break;
        case master_key_touched:
            make_sound(1, 1000);
            LED_TURN_ON_GR;
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

/**
 * Save the current key or delete it from database.
 * Two beeps means: deleted.
 * One beep means: added.
 * In case of a master key touch, the next state will be he master_delete.
 * */
static void master_mode(inputs_t input) {
    switch (input) {
    case timeout:
        //uart_send_str("Normal mode>", 1);
        LED_TURN_ON_RE;
        LED_TURN_OFF_GR;
        SEND_USER_INFO(INFO_NONE,1,INFO_LONG);
        user_info_ms = 1000;
        ibutton_fsm.current_state = check_touch;
        break;
    case master_key_touched:
        //uart_send_str("Erase key code data?", 1);
        LED_TURN_OFF_GR;
        LED_TURN_ON_RE;
        SEND_USER_INFO(INFO_BOTH,0,INFO_SHORT);
        TIMEOUT(6000);
        ibutton_fsm.current_state = master_delete;
        break;
    case key_touched:
        delete_or_add_key(iButton_data.key_code);
        LED_TURN_ON_RE;
        LED_TURN_OFF_GR;
        ibutton_fsm.current_state = check_touch;
        break;
    }
    ibutton_fsm.input_to_serve = 0;
}

/**
 * Basic state to wait for a touch.
 */
static void check_touch(inputs_t input) {
    ibutton_fsm.input_to_serve = 0;
    uint16_t addr = SEGMENT_0;
    uint8_t msg_type = 0;
    uint8_t msg_data[7];
    timeout_ms = 0;

    switch (ibutton_fsm.input) {
    case master_key_touched:
        if (GPIO_GET_INPUT(JUMPER_M_PORT,JUMPER_M_PIN)) {         // Master enable jumper off
            //uart_send_str("Master mode#", 1);
            TIMEOUT(60000);
            SEND_USER_INFO(INFO_ONLY_RED,0,INFO_SHORT);
            LED_TURN_OFF_RE;
            ibutton_fsm.current_state = master_mode;
        }
        else {
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
            LED_TURN_OFF_RE;
            LED_TURN_ON_GR;

        }
        msg_type = MSG_M_KEY_TOUCHED;
        break;
    case key_touched:
        if ( flash_search_key(iButton_data.key_code, &addr) ) {
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
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            msg_type = MSG_KEY_TOUCHED;
        }
        else {
            //uart_send_str("ACCESS DENIED!",1);
            LED_TURN_ON_GR;
            LED_TURN_ON_RE;
            SEND_USER_INFO(INFO_3_BEEPS,0,INFO_SHORT);
            ibutton_fsm.current_state = access_denied;
            msg_type = MSG_KEY_TOUCHED_ILL;
        }
        break;
    case button_pressed:
        if ( *iButton_data.mode_ptr == MODE_NORMAL ) {
            //uart_send_str("RELAY=ON", 1);
            REL_ON;
            make_sound(1, INFO_SHORT);
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            ibutton_fsm.current_state = access_allow;
            refresh_timing();
            TIMEOUT(1000);

        }
        msg_type = MSG_OPEN_BUTTON;
        break;
    default:
        return;
    }
    msg_data[0] = msg_type;
    copy_wordarr_to_bytearr(iButton_data.key_code, &msg_data[1], 3);
    uart_send_packet(msg_data, TYPE_INFO, 7);
}

/* State functions stop____________________________________________________________________________________ */


/**
 * Interrupt routines:
 * */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void) {
    if ( timeout_ticking ) {            // FSM timing
        if ( !timeout_ms-- ) {
            PUT_INPUT(timeout);
            ibutton_fsm.input_to_serve = 1;
            timeout_ticking = 0;
        }
    }
    if ( !( --reader_polling_ms )){  // iButton reader timing
        reader_polling_flag = 1;
        reader_polling_ms = READ_POLLING_TIME;
    }
    if ( !(--reader_disable_ms )) {    // iButton reader disable time
        iButton_data.reader_enable_flag = 1;
        PUT_INPUT(key_away);
    }
    if ( uart_timeout_ticking ) {       // UART timing
        if( !(--uart_timeot_ms) ) {
            uart_timeout_ticking = 0;
            uart_timeout();
        }
    }
    if ( !piezo_on_time-- ) {           // Piezo timing
         PIEZO_PORT_SEL &= ~PIEZO_BIT;
         PIEZO_PORT_DIR &= ~PIEZO_BIT;
    }
    if ( user_info_mode ) {             // User feedback information timing
        if ( !user_info_ms-- ) {
            user_info_flag = 1;
        }
    }
}
