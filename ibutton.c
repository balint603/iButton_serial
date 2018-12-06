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

static const uint8_t command_byte = 0x33;



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
    P2OUT &= ~(LED_PIN_GR + LED_PIN_RE + DATA_PIN);
    RELEASE;
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
 * \param uint16_t *data must be a buffer, with size of 3 word.
 * \ret 0 when the computed crc equals the MSB from the ROM data and LSB equals 01h (iButton family code).
 * \ret 1 when the computed crc OR the family code does not match.
 * */
int ibutton_read_it(uint16_t *data){
    uint8_t i,k;
    uint8_t databit, crc = 0, first_byte, crc_byte;
    uint8_t temp;

    write_command();
    wait_us(480);

    for(i = 8; i > 0; i--){     // Read family code
        PULL_DOWN;
        wait_us(5);
        RELEASE;
        wait_us(10);
        first_byte >>= 1;
        if(databit = GET_INPUT)
            first_byte |= 128;
        temp = (crc & 0x01) ^ databit;
        crc >>= 1;
        if(temp)
            crc ^= 0x8C;
        wait_us(50);
    }
    if(first_byte != 0x01)
        return 1;
    for(i = 3; i > 0; i--){     // Read serial number part
        for(k = 16; k > 0; k--){
            PULL_DOWN;
            wait_us(5);
            RELEASE;
            wait_us(10);
            *(data) >>= 1;
            if(databit = GET_INPUT)
                *(data) |= 0x8000;
            temp = (crc & 0x01) ^ databit;
            crc >>= 1;
            if(temp)
                crc ^= 0x8C;
            wait_us(50);
        }
        data++;
    }
    for(i = 8; i > 0; i--){     // Read CRC
        PULL_DOWN;
        wait_us(5);
        RELEASE;
        wait_us(10);
        crc_byte >>= 1;
        if(GET_INPUT)
            crc_byte |= 128;
        wait_us(50);
    }
    return crc == crc_byte ? 0 : 1;
}

/*uint8_t ibutton_crc8(uint8_t *data) {
    uint8_t i;
    uint8_t k;
    uint8_t temp;
    uint8_t databyte;
    uint8_t crc = 0;
    uint8_t len = 8;

    for (i = 0; i < len; i++) {
        databyte = data[i];
        for (k = 0; k < 8; k++) {
            temp = (crc ^ databyte) & 0x01;
            crc >>= 1;
            if (temp)
                crc ^= 0x8C;

            databyte >>= 1;
        }
    }

    return crc;
}*/
