#include <msp430.h> 
#include <inttypes.h>
#include <intrinsics.h>

#include "ibutton.h"
#include "uart.h"

#define reader_polling_ms 50



uint_fast16_t LED_ms = 100;
volatile uint8_t LED_flag;

uint_fast16_t reader_ms = reader_polling_ms;
volatile uint8_t reader_flag;

uint_fast16_t command_interpret_ms = 1000;
volatile uint8_t command_interpret_flag;


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
     * System Timer (1ms), LCD refreshing TIMING.
     * */
    TACCR0 = 11999;
    TACTL = MC_1 | ID_0 |TASSEL_2 | TACLR;
    TACCTL0 |= CCIE;
}

void init_ports(){
    P1DIR |= BIT0 + BIT6;
    P1OUT &= ~(BIT0 + BIT6);
    P2OUT &= ~(BIT2 + BIT1 + BIT0);
}

/**
 * Test Reading function.
 * It gets the button key and write it to serial port with USCI in every defined ms.
 * Currently no CRC is working.
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    init_clk();
    init_ports();
    init_system_timer();
    uart_init();
    ibutton_init();

	__enable_interrupt();

	uint8_t ib_data[8];

	while(1){

	    if(LED_flag){
	        P1OUT ^= BIT6;
	        LED_flag = 0;
	    }

	    if(command_interpret_flag){
	        switch ( uart_get_byte() ){
	        case 'o':
	            LED_TURN_OFF_RE;
	            uint8_t i;
	            char msg[] = "RED switched off.";
	            for(i = 0; i < sizeof(msg); i++)
	                uart_send_byte(msg[i]);
	            break;

	        case 'i':
	            LED_TURN_ON_RE;
	            uint8_t k;
	            char msg2[] = "RED switched on.";
	            for(k = 0; k < sizeof(msg2); k++)
	                uart_send_byte(msg2[k]);
	            break;
	        }
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
        LED_ms = 100;
    }
    if(!(--reader_ms)){
        reader_flag = 1;
        reader_ms = reader_polling_ms;
    }

    if(!(--command_interpret_ms)){
        command_interpret_flag = 1;
        command_interpret_ms = 10;
    }
}
