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

#define wait_us(n) ( __delay_cycles(n_cycle_to_us*n - 4) )
#define START_BIT  SDA_1; SCL_1; wait_us(BIT_TIME/2); SDA_0; wait_us(BIT_TIME/2); SCL_0; wait_us(BIT_TIME)
#define STOP_BIT  SDA_0; SCL_1; wait_us(BIT_TIME/2); SDA_1; wait_us(BIT_TIME/2);  SCL_1; wait_us(BIT_TIME)

#define SDA_1 SDA_PORT_DIR &= ~SDA_PIN
#define SDA_0 SDA_PORT_DIR |= SDA_PIN
#define GET_INPUT SDA_PORT_INPUT & SDA_PIN

#define SCL_1 SCL_PORT_OUT |= SCL_PIN
#define SCL_0 SCL_PORT_OUT &= ~SCL_PIN


static void shift_byte(uint8_t byte);
static void read_byte(uint8_t *byte);

void EEPROM_init(){

}


const uint8_t write_command = 0b10100000;

static void shift_byte(uint8_t byte){
    int i;
    for(i = 8; i > 0; i--){
        if(byte & 0x80){
            SDA_1;
            wait_us(1);     // SLOW RISING EDGE
        }
        else
            SDA_0;
        SCL_1;
        wait_us(BIT_TIME);
        byte <<= 1;
        SCL_0;
        wait_us(BIT_TIME);
    }
    SDA_1;
}

static void read_byte(uint8_t *byte){
    int i;
    for(i = 8; i > 0; i--){
        *byte <<= 1;
        SCL_1;
        wait_us(BIT_TIME/2);
        if(GET_INPUT)
            byte++;
        wait_us(BIT_TIME/2);
        SCL_0;
        wait_us(BIT_TIME);
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
        shift_byte(*(data+7-datasize));
        SCL_1;
        wait_us(BIT_TIME);
        SCL_0;
        wait_us(BIT_TIME);
    }
    shift_byte(*(data+7-datasize));
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

int key_read(uint8_t *data, uint8_t datasize, uint16_t address){
    uint16_t address_MS_byte = address;
    address_MS_byte >>= 8;
    if(datasize < 8)
        return 1;
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

    START_BIT;
    shift_byte(write_command + 1);
    SCL_1;
    wait_us(BIT_TIME);
    SCL_0;
    wait_us(BIT_TIME);
    SDA_1;
    wait_us(1);
    for(--datasize; datasize > 0; datasize--){
        read_byte((data+7-datasize));
        SDA_0;
        SCL_1;
        wait_us(BIT_TIME);
        SCL_0;
        SDA_1;
        wait_us(BIT_TIME);

    }
    read_byte((data+7-datasize));
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


















