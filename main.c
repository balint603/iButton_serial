#include <msp430.h> 
#include <inttypes.h>
#include <intrinsics.h>

#include "ibutton.h"
#include "uart.h"
#include "EEPROM.h"

#define REL_PORT_DIR P2DIR
#define REL_BIT BIT3
#define REL_ON REL_PORT_DIR |= REL_BIT
#define REL_OFF REL_PORT_DIR &= ~REL_BIT

#define READ_DISABLE_TIME 500 /* ms */
#define READ_POLLING_TIME 10 /* ms */

/** GLOBAL VARIABLES______________________________________________________________________________ */

typedef struct iButton_key_data{
    uint8_t key_code[8];
    uint8_t master_key_code[8];
    const uint8_t super_master_key_code[8];
    uint8_t key_code_n;
    uint8_t compare_flag;
    volatile uint8_t reader_enable_flag;
    uint8_t key_to_process;
    char command;
} iButton_key_data_t;

iButton_key_data_t iButton_data = { .super_master_key_code = {0x01,0x56,0xf1,0xbb,0x1a,0x00,0x00,0xef},
                                    .command = '0',
                                    .reader_enable_flag = 1};

volatile uint8_t uart_rx_buffer_not_empty_flag = 0;

uint_fast16_t LED_ms = 500;
volatile uint8_t LED_flag;

uint_fast16_t timeout_ms = 0;
uint_fast16_t reader_polling_ms = READ_POLLING_TIME;
volatile uint8_t reader_polling_flag = 1;
uint_fast16_t reading_disable_ms;


volatile uint8_t key_code[8];
/** GLOBAL VARIABLES END___________________________________________________________________________ */

/** STATE MACHINE START_____________________________________________________________________________*/

typedef void ( *p_state_handler )();
/** States: */
void access_allow();
void access_denied();
void add_key_mode();
void change_relay_time();
void master_mode();
void wait_for_master_key();
void add_master_key();
void check_touch();

uint8_t timeout_flag = 0;



p_state_handler g_current_state = check_touch;
/** STATE_MACHINE END____________________________________________________________________________ */

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

int compare_key(uint8_t *key1, uint8_t *key2){
    int i;
    if(!(key1 && key2))
        return -1;
    for(i=7; i >= 0; i--)
        if(key1[i] != key2[i])
            return 1;
    return 0;
}

/** FUNCTIONS END________________________________________________________________________________ */

/** STATE FUNCTIONS START________________________________________________________________________ */
void add_master_key(){
    LED_TURN_ON_RE;
    LED_TURN_OFF_GR;
    uint8_t data[8];
    if(ibutton_test_presence()){
        if(!ibutton_read_it(data)){
            EEPROM_key_write(data, EEPROM_MASTER_KEY_PLACE);
            __delay_cycles(60000);
            __delay_cycles(60000);

            if(EEPROM_key_read(iButton_data.master_key_code, EEPROM_MASTER_KEY_PLACE))
                uart_send_str("EEPROM read error.", 1);
            else{
                uart_send_ibutton_data(iButton_data.master_key_code, 1);
                uart_send_str("is master", 1);
            }

            int i;
            for(i=7; i>0 && data[i] == iButton_data.master_key_code[i]; i--)
                ;
            if(i)
                uart_send_str("ADD M_key error!", 1);

            g_current_state = check_touch;
            LED_TURN_OFF_RE;
            LED_TURN_ON_GR;
        }
    }
}

void access_allow(){
    REL_ON;
    LED_TURN_OFF_GR;
    if(timeout_flag){
        uart_send_str("RELAY=OFF", 1);
        LED_TURN_ON_GR;
        REL_OFF;
        g_current_state = check_touch;
        timeout_flag = 0;
    }
}

void access_denied(){
    if(timeout_flag){
        LED_TURN_ON_GR;
        LED_TURN_OFF_RE;
        g_current_state = check_touch;
        timeout_flag = 0;
    }
}
/**
 *
 * */
void add_key_mode(){
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
                switch(ret = EEPROM_get_key_or_empty_place(iButton_data.key_code, &addr, &addr_ff, EEPROM_LAST_KEY_SPACE, 0)){
                case 0:
                    if(EEPROM_key_write(iButton_data.key_code, addr_ff))
                        uart_send_str("COM ERROR",1);
                    __delay_cycles(60000);
                    __delay_cycles(60000);
                    EEPROM_key_read(data_r, addr_ff);
                    for(i = 7; i > 0 && data_r[i] == iButton_data.key_code[i]; i--)
                        ;
                    if(i){
                        uart_send_str("ERROR: WRITE INTO FREE SPACE", 1);
                    }
                    else{
                        uart_send_str("KEY SUCCESSFULLY ADDED", 1);
                    }
                    LED_TURN_ON_GR;
                    LED_TURN_OFF_RE;
                    g_current_state = check_touch;
                    break;

                case 1:
                    if(EEPROM_key_write(deleted_key, addr))
                        uart_send_str("COM error", 1);

                    __delay_cycles(60000);
                    __delay_cycles(60000);
                    EEPROM_key_read(data_r, addr);
                    for(i = 7; i > 0 && deleted_key[i] == data_r[i]; i--)
                        ;
                    if(i)
                        uart_send_str("ERROR: WRITE INTO KEY", 1);
                    else{
                        uart_send_str("KEY SUCCESSFULLY DELETED", 1);
                    }
                    LED_TURN_ON_GR;
                    LED_TURN_OFF_RE;
                    g_current_state = check_touch;
                    break;
                default:
                    uart_send_str("EEPROM ERROR", 1);
                    char a, b;
                    hex_byte_to_char((uint8_t)ret, &a, &b);
                    uart_send_byte(a);
                    uart_send_byte(b);
                    uart_send_byte(10);
                    uart_send_byte(13);
                    LED_TURN_ON_GR;
                    LED_TURN_OFF_RE;
                    g_current_state = check_touch;
                }
            }
        }
    }
}

void change_relay_time(){

}

void master_mode(){
    P1OUT |= BIT0;
}

void wait_for_master_key(){
    char MSch, LSch;
    uint8_t number;

    if(timeout_flag){
        uart_send_str("Timeout", 1);
        LED_TURN_ON_GR;
        timeout_flag = 0;
        g_current_state = check_touch;
    }
    else if(uart_get_buffer_bytes() >= 2){
        timeout_ms = 30000;
        MSch = uart_get_byte();
        LSch = uart_get_byte();
        hex_char_to_number(MSch, LSch, &number);

        /**debug*/
        char a,b;
        hex_byte_to_char(number, &a, &b);
        uart_send_byte((char)a);
        uart_send_byte((char)b);

        /**debug end*/

        if(number != iButton_data.master_key_code[iButton_data.key_code_n])
            iButton_data.compare_flag++;

        if(iButton_data.key_code_n++ >= 7){     // Comparing end
            if(!iButton_data.compare_flag){     // Master code ok
                iButton_data.key_code_n = 0;
                iButton_data.compare_flag = 0;

                switch (iButton_data.command){
                case 'a':
                    uart_send_str("->", 1);
                    uart_send_str("Switched to add key mode", 1);
                    g_current_state = add_key_mode;
                    break;
                case 't':
                    uart_send_str("->", 1);
                    uart_send_str("Change relay on-time:", 0);
                    g_current_state = change_relay_time;
                    break;
                default:
                    uart_send_str("Wrong cmd!", 1);
                    break;
                }
                LED_TURN_ON_RE;
                timeout_ms = 30000;
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
}

void check_touch(){
    uint16_t addr = 0;
    uint16_t addr_ff = 0;
    uint8_t data[8];

    timeout_ms = 0;

    if(ibutton_test_presence()){
        if(!ibutton_read_it(data)){
            if(EEPROM_get_key_or_empty_place(data, &addr, &addr_ff, 512, 0) == 1){
                uart_send_str("RELAY=ON", 1);
                timeout_ms = 2000;
                REL_ON;
                g_current_state = access_allow;

            }else if(!compare_key(data, iButton_data.master_key_code)){
                uart_send_str("Master mode", 1);
                timeout_ms = 30000;
                REL_ON;
                g_current_state = master_mode;
            }
            else{
                uart_send_str("ACCESS DENIED!",1);
                timeout_ms = 1000;
                LED_TURN_OFF_GR;
                LED_TURN_ON_RE;
                g_current_state = access_denied;
            }
        }
    }
    else if(uart_rx_buffer_not_empty_flag){
        switch (iButton_data.command = uart_get_byte()){            // Get current command.
        case 'm':
            uart_send_str("Please send/touch master key!", 1);
            timeout_ms = 30000;
            LED_TURN_OFF_GR;
            g_current_state = wait_for_master_key;
            break;
        default:
            uart_send_str("Invalid cmd!", 1);   // \todo disable UART for a while!

            break;
        }
    }
}

/** STATE FUNCTIONS END__________________________________________________________________________ */

/** FOR DEBUG */
uint8_t key_data[8];
uint16_t address = 0;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    init_clk();
    init_ports();
    init_system_timer();
    uart_init();
    ibutton_init();

	LED_TURN_ON_GR;

	EEPROM_key_read(iButton_data.master_key_code, EEPROM_MASTER_KEY_PLACE);
	if(iButton_data.master_key_code[0] != 0x01){
	    uart_send_str("First start EEPROM erase.", 1);
	    EEPROM_clear_ff();
	    uart_send_str("Please touch a master key.", 1);
	    g_current_state = add_master_key;
	    __delay_cycles(60000);
	    __delay_cycles(60000);
	}

	// \todo sys check.
	uart_send_str("System OK", 1);
	__enable_interrupt();

	uint8_t key[8];
	uint8_t pos = 0;

	uint8_t prev_presence = 0;
	uint8_t curr_presence = 0;
	while(1){

	    if( reader_polling_flag ){

	        if( curr_presence = ibutton_test_presence()){
	            if(!iButton_data.reader_enable_flag)
	               reading_disable_ms = READ_DISABLE_TIME;
               else if(!ibutton_read_it(iButton_data.key_code)){
                   iButton_data.key_to_process = 1;
                   iButton_data.reader_enable_flag = 0;
                }
	        }
	        else if(prev_presence)
                reading_disable_ms = READ_DISABLE_TIME;
	        prev_presence = curr_presence;
            reader_polling_flag = 0;
	    }
	    // test
	    if(iButton_data.key_to_process){
	        iButton_data.key_to_process = 0;
	        P1OUT ^= BIT0;
	    }

	    //g_current_state();


	    /* Debug START___________________________________________________*/
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
	        else if(pos <= 32){

	            EEPROM_key_read(key, pos);
	            uart_send_ibutton_data(key, 1);
	            uart_send_str("",1);
	            pos += 8;

	        }else if(pos == 40){
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

    if(!(--LED_ms)){
        LED_flag = 1;
        LED_ms = 250;
    }
    if(!(--timeout_ms)){
        timeout_flag = 1;
    }
    if( !( --reader_polling_ms )){
        reader_polling_flag = 1;
        reader_polling_ms = READ_POLLING_TIME;
    }
    if( !(--reading_disable_ms)){
        iButton_data.reader_enable_flag = 1;
    }
}
