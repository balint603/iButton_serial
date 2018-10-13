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
#include <uart.h>



#define SDA_1 SDA_PORT_DIR &= ~SDA_PIN
#define SDA_0 SDA_PORT_DIR |= SDA_PIN
#define GET_INPUT SDA_PORT_INPUT & SDA_PIN

#define SCL_1 SCL_PORT_OUT |= SCL_PIN
#define SCL_0 SCL_PORT_OUT &= ~SCL_PIN

#define wait_us(n) ( __delay_cycles(n_cycle_to_us*n - 4) )
#define START_BIT SCL_0; SDA_1; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME/2); SDA_0; wait_us(BIT_TIME/2 + BIT_TIME)
#define STOP_BIT SCL_0; SDA_0; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME/2); SDA_1; wait_us(BIT_TIME/2); wait_us(BIT_TIME)
#define ACK_SKIP SCL_0; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME)
#define ACK_CHECK(n) ( {SCL_0; SDA_1; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME/2); if(GET_INPUT) return n; wait_us(BIT_TIME/2); })
#define ACK_CREATE SCL_0; SDA_0; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME)

static void shift_byte(uint8_t byte);
static void read_byte(uint8_t *byte);



const uint8_t write_command = 0b10100000;

int EEPROM_clear_ff(){

    uint8_t datasize = EEPROM_SIZE_OF_PAGE;
    uint16_t page_cnt = EEPROM_N_OF_PAGES;
    while(page_cnt-- > 0){
        SCL_0;
        START_BIT;
        shift_byte(write_command);                           // DEVICE ADDRESS
        ACK_CHECK(1);
        shift_byte(0);                // ADDRESS
        ACK_SKIP;
        shift_byte(0);
        ACK_SKIP;
        while(datasize-- > 0){
          shift_byte(0xFF);
          ACK_SKIP;
        }
        STOP_BIT;
        SCL_0;
    }
    return 0;
}

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
    SCL_0;
    SDA_1;
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

int EEPROM_key_write(uint8_t *data, uint16_t address){
    SCL_0;
    uint8_t datasize = 8;
    uint16_t address_MS_byte = address;
    address_MS_byte >>= 8;

    START_BIT;
    shift_byte(write_command);                           // DEVICE ADDRESS
    ACK_CHECK(1);
    shift_byte((uint8_t)address_MS_byte);                // ADDRESS
    ACK_SKIP;
    shift_byte((uint8_t)address);
    ACK_SKIP;
    while(datasize-- > 0){
        shift_byte(*(data++));
        ACK_SKIP;
    }

    STOP_BIT;
    SCL_0;
    return 0;
}

/*int EEPROM_page_write(uint8_t data, uint16_t address){
    SCL_0;
    uint8_t datasize = 64;
    uint16_t address_MS_byte = address;
    address_MS_byte >>= 8;

    START_BIT;
    shift_byte(write_command);                           // DEVICE ADDRESS
    ACK_CHECK(1);
    shift_byte((uint8_t)address_MS_byte);                // ADDRESS
    ACK_SKIP;
    shift_byte((uint8_t)address);
    ACK_SKIP;
    while(datasize-- > 0){
        shift_byte(data);
        ACK_SKIP;
    }

    STOP_BIT;
    SCL_0;
    return 0;

}*/
/** IT seems working. */
int EEPROM_key_read(uint8_t *data, uint16_t address){
    SCL_0;
    uint16_t address_MS_byte = address;
    address_MS_byte >>= 8;

    START_BIT;
    shift_byte(write_command);                           // DEVICE ADDRESS
    ACK_CHECK(1);
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

uint16_t EEPROM_get_key_or_empty_place(uint8_t *key_code, uint16_t *addr, uint16_t addr_limit, uint8_t reading_dir){
    if(addr_limit >= EEPROM_MASTER_KEY_PLACE)
        return 0xFFFC;
    if(reading_dir){
        if(*addr < addr_limit)
            return 0xFFFE;
    }
    else{
        if(*addr > addr_limit)
            return 0xFFFE;
    }

    uint16_t addr_MS_byte = *addr;
        addr_MS_byte >>= 8;

    uint8_t free_byte_cnt, key_cmp_cnt, i;
    uint8_t byte;

    START_BIT;
    shift_byte(write_command);
    ACK_CHECK(0xFFFD);
    shift_byte((uint8_t)addr_MS_byte);
    ACK_SKIP;
    shift_byte((uint8_t)*addr);
    ACK_SKIP;

    START_BIT;
    shift_byte(write_command + 1);
    ACK_CHECK(0xFFFD);

    while(*addr < addr_limit){
        uint8_t i;
        free_byte_cnt = 0;
        key_cmp_cnt = 0;

        for(i = 8; i > 0; i--){
            read_byte(&byte);
            ACK_CREATE;
            if((byte) == (*(key_code+8-i)))
                key_cmp_cnt++;
            if((byte) == 0xFF)
                free_byte_cnt++;
        }
        if(key_cmp_cnt == 8)
            return 1;               // KEY code found
        if(free_byte_cnt == 8)
            return 0;
        if(reading_dir)
            ;//*addr -= 8; FUNCTION NOT IMPLEMENTED
        else
            *addr += 8;
    }
    return 0xFFFF;
}













