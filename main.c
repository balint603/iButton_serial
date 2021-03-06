#include <msp430g2553.h>
#include <inttypes.h>
#include <intrinsics.h>

#include "ibutton.h"
#include "uart.h"
#include "flash.h"
#include "fsm.h"

/** GLOBAL VARIABLES______________________________________________________________________________ */

extern volatile uint8_t user_info_flag;

/** GLOBAL VARIABLES END___________________________________________________________________________ */

/** INITIALIZATION FUNCTIONS START_______________________________________________________________ */
void init_clk(){
    BCSCTL3 = LFXT1S_2;
    IFG1 &= ~OFIFG;
    DCOCTL = CALDCO_12MHZ;           // Calibrated data
    BCSCTL1 = CALBC1_12MHZ;
    BCSCTL1 |= DIVA_0;
    BCSCTL2 = SELM_0 +
              DIVM_0 +
              DIVS_3;
    BCSCTL2 &= ~SELS;
}

void init_system_timer(){

    /**
     * System Timer (1ms)
     * */
    TACCR0 = 749;  // 1KHz
    TACTL = MC_1 | ID_0 |TASSEL_2 | TACLR;
    TACCTL0 |= CCIE;
    /**
     * Piezo SETTINGS
     * */
    TACCR1 = 325;   // 2KHz

}

/** INITIALIZATION FUNCTIONS END_________________________________________________________________ */

/**
 * Initialization.
 * Get the key code if iButton has touched to the reader.
 * Generating input for the state machine by checking the presence of an iButton key.
 * After a key had been successfully read by the reader, key-reading is disabled. Reading will be enabled defined miliseconds after.
 * */
int main(void)
{
    WATCHDOG_STOP;   // stop watchdog timer

    init_clk();
    init_system_timer();

    uart_init();
    ibutton_init();
    ibutton_fsm_init();
    flash_init();
    uint8_t data = MSG_BOOT;
	uart_send_packet(&data, TYPE_INFO, 1);
	WATCHDOG_RESET;
	__delay_cycles(120000);
    __enable_interrupt();

	while ( 1 ) {
	    if ( ibutton_fsm_is_input() ){
	        ibutton_fsm_change_state();
	    } else {
	        if ( ibutton_fsm_get_polling_flag() ){
                ibutton_read();
            }
	    }

	    if ( ibutton_fsm_is_packet() )
	        ibutton_process_command();

        if ( user_info_flag ) {
            ibutton_user_info_mode_service();
            user_info_flag = 0;
        }
        WATCHDOG_RESET;   // stop watchdog timer
	}
	return 0;
}
