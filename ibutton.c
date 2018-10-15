/*
 * ibutton.c
 *
 *  Created on: 2018. szept. 29.
 *      Author: Major János Bálint (E-Kontakt Bt.)
 *
 *  Description: This module is created for MSP430 controllers
 *               can be used for reading 01 family of iButtons.
 */

#include "ibutton.h"
#include <msp430.h>
#include <inttypes.h>
#include <intrinsics.h>

#define wait_us(n) ( __delay_cycles(n_cycle_to_us*n) )

const uint8_t command_byte = 0x33;



/**
 *  Write 33H value to the data line.
 *  This function is used before the incoming data.
 *  Uses command_byte constant.
 * */
static void write_command(){
    int i;
    uint8_t data = command_byte;
    for(i = 8; i > 0; i--){
        if(data & 0x01){                    // next bit is 1
            PULL_DOWN;
            wait_us(10);
            RELEASE;
            wait_us(60);
        }
        else{
            PULL_DOWN;
            wait_us(60);
            RELEASE;
            wait_us(10);
        }
        data >>= 1;
    }

}



/**
 *  Initialization function.
 * */
void ibutton_init(){
    LED_TURN_OFF_RE;
    LED_TURN_ON_GR;
}



/**
 *  Test if an iButton device is connected to the reader or not. It filters the short circuit.
 *
 *  \ret 0 when the input level was low before reset pulse send, or no answer after reset pulse. (See one-wire protocol).
 *  \ret 1 when the input had been pulled down only after the reset pulse, in this case this function assumes that an iButton device had given a presence pulse.
 * */
int ibutton_test_presence(){
    uint8_t pin_state_temp = GET_INPUT;
    wait_us(100);
    if( !(pin_state_temp && GET_INPUT) )              // Input is pulled down to GND, it means shorted.
        return 0;
    PULL_DOWN;                                      // Start RESET pulse, return whether the device had pulled down the line, or had not.
    wait_us(480);
    RELEASE;
    wait_us(70);
    pin_state_temp = GET_INPUT;
    wait_us(410);
    return pin_state_temp ? 0 : 1 ;
}
/**
 * Read the iButton ROM into the input buffer.
 * This function must be called, when an iButton has just been connected to the reader.
 * \param uint8_t *data must be a buffer, with size of 8 byte.
 * \ret 0 when the computed crc equals the MSB from the ROM data and LSB equals 01h (iButton family code).
 * \ret 1 when the computed crc OR the family code does not match.
 * */
int ibutton_read_it(uint8_t *data){
    int i,k;
    uint8_t databyte, databit, crc = 0;
    uint8_t temp;

    write_command();
    wait_us(480);

    for(i = 0; i < 8; i++){     // READING process start here
        databyte = 0;
        for(k = 0; k < 8; k++){
            PULL_DOWN;
            wait_us(5);
            RELEASE;
            wait_us(10);
            databit = GET_INPUT;
            databyte = databyte | (databit << k);
            if(i < 7){
                temp = (crc & 0x01) ^ databit;
                crc >>= 1;
                if(temp)
                    crc ^= 0x8C;
            }
            wait_us(50);
        }
        *(data+i) = databyte;
    }   // READING process end here
    return *(data+7) != crc || *(data) != 0x01 ? 1 : 0;        // CRC!
}
