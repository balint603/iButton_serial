#include <msp430.h> 
#include <inttypes.h>
#include <intrinsics.h>

#include "ibutton.h"
#include "uart.h"
#include "EEPROM.h"

#define reader_polling_ms 20

#define REL_PORT P2OUT
#define REL_BIT BIT3
#define REL_ON REL_PORT |= REL_BIT
#define REL_OFF REL_PORT &= ~REL_BIT

uint_fast16_t LED_ms = 500;
volatile uint8_t LED_flag;

uint_fast16_t reader_ms = reader_polling_ms;
volatile uint8_t reader_flag;

uint_fast16_t timeout_ms = 1000;

volatile uint16_t delay_ms;

volatile uint8_t key_code[8];

uint8_t super_M_key[] = {0x01,0x56,0xf1,0xbb,0x1a,0x00,0x00,0xef};

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
    P2DIR |= BIT3 + BIT5;
    P1OUT &= ~(BIT0 + BIT6);
    P2OUT &= ~(BIT5 + BIT4 + BIT2 + BIT1 + BIT0);
}


/** STATE FUNCTIONS START_____________________________________________________________________________________________ */
void super_M_mode(input_t next_input){
    switch(next_input){
    case super_M_key_touch:
        state.current_state = check_touch;
        LED_TURN_OFF_RE;
    }

}

void check_touch(input_t next_input){
    switch(next_input){
    case super_M_key_touch:
        state.current_state = super_M_mode;
        LED_TURN_ON_RE;
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

	uint8_t super_M_key[] = {0x01, 0xCA, 0xFE, 0xBE, 0xEF, 0xAB, 0xCD, 0xEF};

	__enable_interrupt();

	uint16_t ret;
	uint16_t ret_H;

	uint16_t addr = 0;
	uint16_t addr_H;

	uint16_t ff_addr = 0;
	uint16_t ff_addr_H = 0;

	//EEPROM_key_write(data2, 16);
	ret = EEPROM_get_key_or_empty_place(data2, &addr, &ff_addr, EEPROM_MASTER_KEY_PLACE-8, 0);
	ret_H = ret >> 8;
	addr_H = addr >> 8;
	ff_addr_H = ff_addr >> 8;
//	int a = 0;
	while(1){
	    if(LED_flag){
	        P1OUT ^= BIT6;
	        LED_flag = 0;

	            uart_send_byte((uint8_t)ret_H);
	            uart_send_byte((uint8_t)ret);
	            uart_send_byte((uint8_t)addr_H);
	            uart_send_byte((uint8_t)addr);
	            uart_send_byte((uint8_t)ff_addr_H);
	            uart_send_byte((uint8_t)ff_addr);
	        /*EEPROM_key_read(data, a);
	        uart_send_ibutton_data(data, 0);
	        a += 8;*/
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
        timeout_ms = 30000;
    }
}
