/*
 * EEPROM.c
 *
 *  Created on: 2018. okt. 10.
 *      Author: MAJXAAPPTE
 */


#include "EEPROM.h"
#include <msp430.h>
#include <inttypes.h>
#include <intrinsics.h>



#define SDA_1 SDA_PORT_DIR &= ~SDA_PIN
#define SDA_0 SDA_PORT_DIR |= SDA_PIN
#define GET_INPUT SDA_PORT_INPUT & SDA_PIN

#define SCL_1 SCL_PORT_OUT |= SCL_PIN
#define SCL_0 SCL_PORT_OUT &= ~SCL_PIN

#define wait_us(n) ( __delay_cycles(n_cycle_to_us*n - 4) )
#define START_BIT SCL_0; SDA_1; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME/2); SDA_0; wait_us(BIT_TIME/2 + BIT_TIME)
#define STOP_BIT SCL_0; SDA_0; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME/2); SDA_1; wait_us(BIT_TIME/2); wait_us(BIT_TIME)
#define ACK_SKIP SCL_0; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME)
#define ACK_CHECK SCL_0; SDA_1; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME/2); /*if(GET_INPUT) return 1;*/ wait_us(BIT_TIME/2)
#define ACK_CREATE SCL_0; SDA_0; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME)

static void shift_byte(uint8_t byte);
static void read_byte(uint8_t *byte);

void EEPROM_init(){

}


const uint8_t write_command = 0b10100000;

static void shift_byte(uint8_t byte){
    int i;
    for(i = 8; i > 0; i--){
        SCL_0;
        if(byte & 0x80)
            SDA_1;
        else
            SDA_0;
        wait_us(BIT_TIME);
        SCL_1;
        wait_us(BIT_TIME);
        byte <<= 1;
    }
}

static void read_byte(uint8_t *byte){
    int i;
    for(i = 8; i > 0; i--){
        SCL_0;
        wait_us(BIT_TIME);
        (*byte) <<= 1;
        SCL_1;
        wait_us(BIT_TIME/2);
        if(GET_INPUT)
            (*byte) |= BIT0;
        else
            (*byte) &= ~BIT0;
        wait_us(BIT_TIME/2);
    }
}

int key_write(uint8_t *data, uint8_t datasize, uint16_t address){
    uint16_t address_MS_byte = address;
    address_MS_byte >>= 8;
    START_BIT;
    shift_byte(write_command);                           // DEVICE ADDRESS
    SCL_1;
    wait_us(BIT_TIME/2);
    if(GET_INPUT)
        return 1;
    wait_us(BIT_TIME/2);
    SCL_0;
    wait_us(BIT_TIME);
    shift_byte((uint8_t)address_MS_byte);               // ADDRESS
    SCL_1;
    wait_us(BIT_TIME);
    SCL_0;
    wait_us(BIT_TIME);
    shift_byte((uint8_t)address);
    SCL_1;
    wait_us(BIT_TIME);
    SCL_0;
    wait_us(BIT_TIME);

    for(--datasize; datasize > 0; datasize--){
        read_byte((data+7-datasize));
        SCL_1;
        wait_us(BIT_TIME);
        SCL_0;
        wait_us(BIT_TIME);
    }
    read_byte((data+7));
    SCL_1;
    wait_us(BIT_TIME/2);
    if(GET_INPUT)
        return 1;
    wait_us(BIT_TIME/2);
    SCL_0;
    wait_us(BIT_TIME);
    STOP_BIT;
    return 0;
}
/** IT seems working. */
int key_read(uint8_t *data, uint8_t datasize, uint16_t address){
    SCL_0;
    uint16_t address_MS_byte = address;
    address_MS_byte >>= 8;
    if(datasize < 8)
        return 1;
    START_BIT;
    shift_byte(write_command);                           // DEVICE ADDRESS
    ACK_CHECK;
    shift_byte((uint8_t)address_MS_byte);                // ADDRESS
    ACK_SKIP;
    shift_byte((uint8_t)address);
    ACK_SKIP;

    START_BIT;                                          // SET DEVICE TO READ MODE
    shift_byte(write_command + 1);
    ACK_SKIP;
    int i;
    for(i=0; i < 7; i++ ){
        read_byte((data+i));
        ACK_CREATE;
    }
    read_byte((data+7));
    SCL_0;
    SDA_1;
    wait_us(BIT_TIME);
    SCL_1;
    wait_us(BIT_TIME);
    SCL_0;
    wait_us(BIT_TIME);
    SCL_1;
    wait_us(BIT_TIME);

    STOP_BIT;
    SCL_0;
    return 0;
}


















