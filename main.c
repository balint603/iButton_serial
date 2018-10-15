#include <msp430.h> 
#include <inttypes.h>
#include <intrinsics.h>

#include "ibutton.h"
#include "uart.h"
#include "EEPROM.h"

#define reader_polling_ms 20

#define REL_PORT_DIR P2OUT
#define REL_BIT BIT3
#define REL_ON REL_PORT_DIR |= REL_BIT
#define REL_OFF REL_PORT_DIR &= ~REL_BIT

/** UART EXTERN */
volatile uint8_t uart_rx_buffer_not_empty_flag = 0;

uint_fast16_t LED_ms = 500;
volatile uint8_t LED_flag;

uint_fast16_t reader_ms = reader_polling_ms;
volatile uint8_t reader_flag;

uint_fast16_t timeout_ms = 0;

volatile uint16_t delay_ms;

volatile uint8_t key_code[8];

uint8_t super_M_key[] = {0x01,0x56,0xf1,0xbb,0x1a,0x00,0x00,0xef};

uint8_t M_key[] = {0x01,0x56,0xf1,0xbb,0x1a,0x00,0x00,0xef};

uint8_t deleted_key[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/** State machine START*/
#define N_STATE_INPUTS 10

typedef enum inputs{
    super_M_key_touch,
    key_touch,
    exit,
    timeout,
    no_input
} input_t;

typedef void ( *p_state_handler )( input_t next_input);

typedef struct state_machine {
    p_state_handler current_state;
    uint8_t input;
    uint8_t n_of_inputs;
} state_machine_t;

/** State functions */
void check_touch(input_t next_input);
void super_M_mode(input_t next_input);

state_machine_t state = {
                                .input = no_input,
                                .n_of_inputs = 0
                              };

input_t inputs[N_STATE_INPUTS];


input_t get_input(){

    if(state.n_of_inputs){
        state.n_of_inputs--;
        return inputs[state.n_of_inputs];
    }
    else
        return no_input;
}

void put_input(input_t new_input){

    if(state.n_of_inputs !=  0 || state.n_of_inputs != N_STATE_INPUTS - 1){
        inputs[state.n_of_inputs] = new_input;
        state.n_of_inputs++;
    }
    else    // 0 or queue is full, overwrite IT!
            inputs[state.n_of_inputs - 1] = new_input;  // ?
}
/** END */



void init_clk(){
    BCSCTL3 = LFXT1S_2;             // ACLK VLO source
    IFG1 &= ~OFIFG;                 // Delete Oscillator Fault bit
    DCOCTL = CALDCO_12MHZ;           // Calibrated data
    BCSCTL1 = CALBC1_12MHZ;
    BCSCTL1 |= DIVA_0;
    BCSCTL2 = SELM_0 +
              DIVM_0 +
              DIVS_0;
    BCSCTL2 &= ~SELS;
}

void init_system_timer(){

    /**
     * System Timer (1ms)
     * */
    TACCR0 = 11999;
    TACTL = MC_1 | ID_0 |TASSEL_2 | TACLR;
    TACCTL0 |= CCIE;
}

void init_ports(){
    P1DIR |= BIT0 + BIT6;
    P2DIR |= BIT5;
    P1OUT &= ~(BIT0 + BIT6);
    P2OUT &= ~(BIT5 + BIT4 + BIT3 + BIT2 + BIT1 + BIT0);
    P2DIR &= ~BIT3;
}


/** STATE FUNCTIONS START_____________________________________________________________________________________________ */

void open_relay(input_t next_input){
    REL_ON;
    switch(next_input){
    case timeout:
        REL_OFF;
        state.current_state = check_touch;
    }
}

void master_mode(input_t next_input){
    uint8_t data[8];
    uint8_t data_r[8];
    uint8_t i;
    uint16_t addr = 0;
    uint16_t addr_ff = 0;

    if(uart_get_buffer_bytes() >= 8){
        for(i = 8; i > 0; i--)
            data[8-i] = uart_get_byte();
        switch(EEPROM_get_key_or_empty_place(data, &addr, &addr_ff, 256, 0)){
        case 0:
            EEPROM_key_write(data, addr_ff);
            __delay_cycles(60000);
            __delay_cycles(60000);
            EEPROM_key_read(data_r, addr_ff);
            for(i = 8; i > 0 && data_r[8-i] == data[8-i]; i--)
                ;
            if(!i){
                uart_send_str("ERROR: EEPROM WRITE", 1);
            }
            else{
                uart_send_str("KEY SUCCESFULLY ADDED", 1);
            }
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            state.current_state = check_touch;
            break;
        case 1:
            EEPROM_key_write(deleted_key, addr);
            __delay_cycles(60000);
            __delay_cycles(60000);
            EEPROM_key_read(data_r, addr);
            for(i = 8; i > 0 && deleted_key[8-i] == data_r[8-i]; i--)
                ;
            if(!i)
                uart_send_str("ERROR: EEPROM WRITE", 1);
            else
                uart_send_str("KEY SUCCESFULLY DELETED", 1);
            LED_TURN_ON_GR;
            LED_TURN_OFF_RE;
            state.current_state = check_touch;
            break;
        default:
            uart_send_str("EEPROM ERROR", 1);
        }
    }
    switch(next_input){
    case timeout:
        state.current_state = check_touch;
    }
}

void wait_for_master_key(input_t next_input){
    uint8_t data[8];
    uint8_t i;
    if(ibutton_test_presence()){
        if(!ibutton_read_it(data)){

            for(i = 8; data[i] == M_key[i]; i--)
                ;
            if(!i){
                uart_send_str("<Master mode>", 1);
                state.current_state = master_mode;
                timeout_ms = 60000;
                LED_TURN_ON_RE;
            }else
                state.current_state = check_touch;
        }
    }else if(uart_get_buffer_bytes() >= 8){
        for(i = 8; (i > 0) && (M_key[8-i] == uart_get_byte()); i--)
            ;
        uart_send_byte(i);
        if(!i){
            state.current_state = master_mode;
            timeout_ms = 60000;
            uart_send_str("<Master mode>", 1);
            LED_TURN_ON_RE;
        }else
            state.current_state = check_touch;
    }
}

void check_touch(input_t next_input){
    uint16_t addr = 0;
    uint16_t addr_ff = 0;
    uint8_t data[8];

    if(ibutton_test_presence()){
        if(!ibutton_read_it(data)){
            if(EEPROM_get_key_or_empty_place(data, &addr, &addr_ff, 512, 0) == 1){
                state.current_state = open_relay;
                timeout_ms = 5000;
                REL_ON;
            }
        }
    }
    else if(uart_rx_buffer_not_empty_flag){
        if('M' == uart_get_byte())
            LED_TURN_OFF_GR;
            state.current_state = wait_for_master_key;
            timeout_ms = 60000;
            uart_send_str("Please send/touch master key!", 1);
    }
}

void add_master_key(input_t next_input){
    LED_TURN_ON_RE;
    LED_TURN_OFF_GR;
    uint8_t data[8];
    if(ibutton_test_presence()){
        if(!ibutton_read_it(data)){
            EEPROM_key_write(data, EEPROM_MASTER_KEY_PLACE);
            // \todo RECHECK MEM!
            uart_send_str("Please touch a master key.", 1);
            state.current_state = check_touch;
            LED_TURN_OFF_RE;
            LED_TURN_ON_GR;
        }
    }
}
/** STATE FUNCTIONS END_________________________________________________________________________________________________ */

/**
 *
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    init_clk();
    init_ports();
    init_system_timer();
    uart_init();
    ibutton_init();

	state.current_state = check_touch;

	LED_TURN_ON_GR;


	uint8_t super_M_key[] = {0x01, 0xCA, 0xFE, 0xBE, 0xEF, 0xAB, 0xCD, 0xEF};
	uint8_t key[8];
	EEPROM_key_read(key, EEPROM_MASTER_KEY_PLACE);
	if(key[0] != 0x01){
	    EEPROM_clear_ff();
	    uart_send_str("Please touch a master key.", 1);
	    state.current_state = add_master_key;
	    __delay_cycles(60000);
	    __delay_cycles(60000);
	}

	EEPROM_key_read(M_key, EEPROM_MASTER_KEY_PLACE);

	__enable_interrupt();

	while(1){

	    state.current_state(get_input());
	    if(LED_flag){
	        P1OUT ^= BIT6;
	        LED_flag = 0;
	    }
	}
	return 0;
}

/**
 * Interrupt routines:
 * */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void){

    if(!(--LED_ms)){
        LED_flag = 1;
        LED_ms = 500;
    }
    if(!(--reader_ms)){
        reader_flag = 1;
        reader_ms = reader_polling_ms;
    }
    if(!(--timeout_ms)){
        put_input(timeout);
    }
}
