#include <msp430g2553.h>
#include <inttypes.h>
#include <intrinsics.h>

#include "ibutton.h"
#include "uart.h"
#include "flash.h"
#include "fsm.h"

/** GLOBAL VARIABLES______________________________________________________________________________ */

volatile uint8_t uart_rx_buffer_not_empty_flag = 0;

extern uint8_t user_info_mode;
extern volatile uint8_t user_info_flag;
extern volatile uint8_t LED_flag;
extern volatile uint_fast16_t timeout_ms;

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
    TACCR0 = 1499;  // 1KHz
    TACTL = MC_1 | ID_2 |TASSEL_2 | TACLR;
    TACCTL0 |= CCIE;
    /**
     * Piezo SETTINGS
     * */
    TACCR1 = 749;   // 2KHz

}

void init_ports(){
    P1DIR |= BIT0;
    P1OUT &= ~(BIT0);
}
/** INITIALIZATION FUNCTIONS END_________________________________________________________________ */

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
    ibutton_init();
    ibutton_fsm_init();
    flash_init();

	uart_send_str("System Start", 1);

    __enable_interrupt();
	while(1){

	    if(ibutton_fsm.input_to_serve){
	        ibutton_fsm_change_state();
	    }else{
	        if(reader_polling_flag){
                ibutton_read();
            }
	    }

        if(user_info_flag){
            ibutton_user_info_mode_service();
            user_info_flag = 0;
        }
        if(LED_flag){
	        P1OUT ^= BIT0;

	        LED_flag = 0;
        }
	}
	return 0;
}
