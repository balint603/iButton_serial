#include <msp430.h> 
#include <inttypes.h>
#include <intrinsics.h>

#include "ibutton.h"
#include "uart.h"
#include "EEPROM.h"
#include "fsm.h"

/** GLOBAL VARIABLES______________________________________________________________________________ */

extern uint16_t reader_polling_ms;
extern volatile uint8_t reader_polling_flag;
extern uint16_t reader_disable_ms;

iButton_key_data_t iButton_data = { .super_master_key_code = {0x01,0x56,0xf1,0xbb,0x1a,0x00,0x00,0xef},
                                    .reader_enable_flag = 1};

volatile uint8_t uart_rx_buffer_not_empty_flag = 0;

uint_fast16_t LED_ms = 125;
volatile uint8_t LED_flag;

volatile uint_fast16_t timeout_ms;
volatile uint_fast16_t led_blink_ms = 500;
volatile uint8_t led_blink_flag;
uint8_t led_blink_enable;

volatile uint8_t key_code[8];
/** GLOBAL VARIABLES END___________________________________________________________________________ */

/** INITIALIZATION FUNCTIONS START_______________________________________________________________ */
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
/** INITIALIZATION FUNCTIONS END_________________________________________________________________ */

/** FUNCTIONS____________________________________________________________________________________ */


/** FUNCTIONS END________________________________________________________________________________ */

/** STATE FUNCTIONS START________________________________________________________________________ */

/** STATE FUNCTIONS END__________________________________________________________________________ */
/**
 * Initialization.
 * Get the key code if iButton has touched to the reader.
 * Generating input for the state machine by checking the presence of an iButton key.
 * After a key had been successfully read by the reader, key-reading is disabled. Reading will be enabled defined seconds after the key is not present.
 * */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    init_clk();
    init_ports();
    init_system_timer();
    uart_init();
    __enable_interrupt();
    __delay_cycles(60000);
    __delay_cycles(60000);
    __delay_cycles(60000);
    __delay_cycles(60000);
    ibutton_init();
    ibutton_fsm_init();


    EEPROM_read_byte(&iButton_data.opening_time, EEPROM_TIME_DATA);
    if(iButton_data.opening_time == 0xFF){
        iButton_data.opening_time = 2;
    }
    EEPROM_read_byte(&iButton_data.mode, EEPROM_MODE_DATA);

	// \todo sys check.
	uart_send_str("System OK", 1);

	uint8_t key[8];
	uint8_t pos = 0;
	while(1){
	    // todo implement button check (maybe interrupt)
	    ibutton_read();

	    if(ibutton_fsm.input_to_serve){
	        ibutton_fsm_change_state();
	    }

	    if(uart_rx_buffer_not_empty_flag)
	        ibutton_command();

	    /* Debug START____________________________________________________*/
	    if(LED_flag){
	        P1OUT ^= BIT6;
	        LED_flag = 0;

	        // DEBUG INFO
	        if(pos == 0){
	            uart_send_str("debug info start>", 1);
	            EEPROM_key_read(key, pos);
                uart_send_ibutton_data(key, 1);
                uart_send_str("",1);

	            pos += 8;
	        }
	        else if(pos <= 80){

	            EEPROM_key_read(key, pos);
	            uart_send_ibutton_data(key, 1);
	            uart_send_str("",1);
	            pos += 8;

	        }else if(pos == 88){
	            EEPROM_key_read(key, EEPROM_MASTER_KEY_PLACE);
	            uart_send_ibutton_data(key, 1);
	            uart_send_str("<debug info end", 1);
	            pos += 8;
	        }
	    }
	    /* Debug END_____________________________________________________*/
	}
	return 0;
}

/**
 * Interrupt routines:
 * */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void){

    if(!(--LED_ms)){    // DEBUG LED
        LED_flag = 1;
        LED_ms = 125;
    }
    if(!(--timeout_ms)){
        put_input(timeout);
        ibutton_fsm.input_to_serve = 1;
    }
    if(led_blink_ms){
        led_blink_ms--;
    }
    else{
        switch (led_blink_enable){
            case 1:
                LED_BLINK_GR;
                led_blink_ms = 499;
                break;
            case 2:
                LED_BLINK_RE;
                led_blink_ms = 499;
                break;
            case 3:
                LED_BLINK_GR;
                LED_BLINK_RE;
                led_blink_ms = 499;
                break;
            default:

                break;
        }
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
