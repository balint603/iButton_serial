/*
 * EEPROM.c
 *
 *  Created on: 2018. okt. 10.
 *      Author: MAJXAAPPTE
 *
 */


#include "EEPROM.h"
#include <msp430.h>
#include <inttypes.h>
#include <intrinsics.h>
#include <uart.h>

/** PINS */
#define SDA_1 SDA_PORT_DIR &= ~SDA_PIN
#define SDA_0 SDA_PORT_DIR |= SDA_PIN
#define GET_INPUT SDA_PORT_INPUT & SDA_PIN

#define SCL_1 SCL_PORT_OUT |= SCL_PIN
#define SCL_0 SCL_PORT_OUT &= ~SCL_PIN

/** MACROS for sequential instructions */
#define wait_us(n) ( __delay_cycles(n_cycle_to_us*n - 4) )
#define START_BIT SCL_0; SDA_1; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME/2); SDA_0; wait_us(BIT_TIME/2 + BIT_TIME)
#define STOP_BIT SCL_0; SDA_0; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME/2); SDA_1; wait_us(BIT_TIME/2); wait_us(BIT_TIME)
#define ACK_SKIP SCL_0; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME)
#define ACK_CHECK(n) ( {SCL_0; SDA_1; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME/2); if(GET_INPUT) return n; wait_us(BIT_TIME/2); })
#define ACK_CREATE SCL_0; SDA_0; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME)
#define ACK_NO SCL_0; SDA_1; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME)

static void shift_byte(uint8_t byte);
static void read_byte(uint8_t *byte);

/** See EEPROM datasheet */
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

/**
 * This function must be called first start of the system.
 * It writes 0xFF to all bytes.
 * Uses EEPROM acknowledgment polling, after 5s of unsuccessful acknowledge inquiry returns 1.
 * \ret 0 when cleared
 * \ret 1 when polling the EEPROM is failed.
 * */
int EEPROM_clear_ff(){

    uint8_t datasize = EEPROM_SIZE_OF_PAGE;
    uint16_t page_cnt = EEPROM_N_OF_PAGES;
    uint16_t addr = 0;
    uint16_t addr_MSB = 0;
    uint8_t tmp;
    uint16_t cnt = 5000;

    while(page_cnt-- > 0){
        do{
            wait_us(1000);
            if(!cnt--)
                return 1;
            START_BIT;
            shift_byte(write_command);                           // DEVICE ADDRESS
            SCL_0; SDA_1; wait_us(BIT_TIME); SCL_1; wait_us(BIT_TIME/2); tmp = GET_INPUT; wait_us(BIT_TIME/2);
        } while (tmp);
        shift_byte((uint8_t)addr_MSB);                // ADDRESS
        ACK_SKIP;
        shift_byte((uint8_t)addr);
        ACK_SKIP;
        while(datasize-- > 0){
          shift_byte(0xFF);
          ACK_CHECK(2);
        }
        STOP_BIT;
        SCL_0;
        addr += 64;
        addr_MSB = addr;
        addr_MSB >>= 8;
        cnt = 5000;
    }
    return 0;
}

/**
 * Writing uint8_t array (iButton key sized) into EEPROM.
 * \ret 0 if key data has been succesfully written.
 * \ret 1 if no acknowledgment was detected (After write_command) .
 * */
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
    ACK_CHECK(1);
    while(datasize-- > 0){
        shift_byte(*(data++));
        ACK_CHECK(1);
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
/**
 *  This function reads the 8 * uint8_t length data. (iButton sized).
 * */
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
    ACK_NO;
    ACK_SKIP;

    STOP_BIT;
    SCL_0;
    return 0;
}

/** Reads byte by address. */
int EEPROM_read_byte(uint8_t *byte, uint16_t address){
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

    read_byte(byte);

    SCL_0;                                              // After the byte, no acknowledgment bit is given to stop reading process.
    SDA_1;
    wait_us(BIT_TIME);
    SCL_1;
    wait_us(BIT_TIME);

    STOP_BIT;
    SCL_0;
    return 0;
}

/**
 *  Search the key in the EEPROM or get free space if it was not found.
 *  \todo data end flag? First byte of the keys are always 0x01.
 *  \param *key_code is the data to be find in the EEPROM.
 *  \param *addr Search from this address, key address will be in this variable when key had been found.
 *  \param addr_limit Max address.
 *  \param reading dir: Function not implemented yet.
 *  \ret 0 NO KEY FOUND.
 *  \ret 1 KEY FOUND.
 *  \ret 0xFFFC addr_limit is too high.
 *  \ret 0xFFFE when *addr is out of addr_limit.
 *  \ret 0xFFFD when EEPROM does not responds with acknowledgment bit after first command.
 * */
uint16_t EEPROM_get_key_or_empty_place(uint8_t *key_code, uint16_t *addr, uint16_t *first_free_addr, uint16_t addr_limit, uint8_t reading_dir){
    if(addr_limit >= EEPROM_MASTER_KEY_PLACE)           // check ptrs.
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

    uint8_t key_cmp_cnt;
    uint8_t byte;

    *first_free_addr = 1;

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
        key_cmp_cnt = 6;        // First byte is checked, after 6 byte the last byte is also checked out of the for cycle.

        read_byte(&byte);
        ACK_CREATE;
        if(byte != 0x01){
            if(byte == 0xFF){
                if(*first_free_addr == 1)
                    *first_free_addr = *addr;
                ACK_NO;
                STOP_BIT;
                return 0;
            }
            if(*first_free_addr == 1){
                *first_free_addr = *addr;
            }
        }
                         // When the first byte is equal to family code: 0x01, start a byte compare routine.

        for(i = 6; i > 0; i--){
            read_byte(&byte);
            ACK_CREATE;
            if(byte == *(key_code+7-i))
                key_cmp_cnt--;
        }
        read_byte(&byte);                               // Last byte
        if( (byte == *(key_code+7)) && !key_cmp_cnt){
            ACK_NO;
            STOP_BIT;
            return 1;
        }else
            ACK_CREATE;

        (*addr) += 8;  // First byte of the next key in memory.
    }
    ACK_NO;
    STOP_BIT;
    return 0xFFFF;
}
