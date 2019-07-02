/*
 * flash.c
 *
 *  Created on: 2018. nov. 7.
 *      Author: MAJXAAPPTE
 */

#include <msp430.h>
#include <inttypes.h>
#include "flash.h"

/**
 * Process to write blocks to FLASH memory. Call a RAM function after set the periphery.
 */
#define WRITE_BLOCK_PROCESS(data, size, address_ptr) \
    FCTL2 = FWKEY + FSSEL_2 + FN2; \
    FCTL3 = FWKEY; \
    FCTL1 = FWKEY + BLKWRT + WRT; \
    block_addressing_RAM(data, size, address_ptr); \
    FCTL1 = FWKEY;

/**
 * This function is stored in RAM because of the block writing criteria of flash memory.
 * Wait until FLASH memory is ready.
 * */
__attribute__((ramfunc))
static void block_addressing_RAM(uint16_t *data, uint8_t size, uint16_t **address_ptr){
    while ( size-- ) {
        *((*address_ptr)++) = *(data++);
        while ( !(FCTL3 & WAIT) )
            ;
    }
}

/** \brief Initial function.
 * Searching the segments storing data.
 * If the first (flag) word of the segment is 0xFFFF segment will erased and tagged by the flag word.
 * Check and count segment with flag 0xEAFF or 0xC0DE. If the current segment is not tagged, it will be tagged.
 * \return 0 Successfully initialized.
 * \return 1 Too much code segments (tagged by 0xC0DE)
 * \return 2 Too much reserved segments (tagged by 0xEAFF)
 * */
int flash_init() {
    uint16_t *flag_ptr = (uint16_t*)SEGMENT_0;
    uint8_t data_segment_cnt = 0, reserved_segment_cnt = 0;
    int ret_val = 0;
    uint16_t cur_flag;

    while ( (uint16_t)flag_ptr <= SEGMENT_8 ) {
        switch ( *flag_ptr ) {
            case 0xEAFF:                            // Reserved segment found.
                if ( ++reserved_segment_cnt >= 2 )
                    // todo Erase this segment, check if all 0xFF?
                    ret_val = 2;
                break;
            case 0xC0DE:                           // Segment including codes found.
                if(++data_segment_cnt >= 9)
                    // todo Erase this segment, check if all 0xFF?
                    ret_val = 1;
                break;
            default:                                // Erase and tag segment.
                // todo erase this segment
                if ( !reserved_segment_cnt ) {
                    cur_flag = 0xEAFF;
                    reserved_segment_cnt++;
                }
                else{
                    cur_flag = 0xC0DE;
                    data_segment_cnt++;
                }
                flash_write_word(cur_flag, (uint16_t)flag_ptr);
                if( *flag_ptr != cur_flag )
                    return 1;
                break;
        }
        flag_ptr += 256;
    }
    return ret_val;
}

/**
 * Write single byte into the flash. Address is not checked! It can be used to write byte too.
 * \return 0 Flash writing process end.
 * \return 1 Data was not saved correctly.
 * */

int flash_write_word(uint16_t word, uint16_t address) {
    WATCHDOG_STOP;
    while ( FCTL3 & BUSY )
        ;

    uint16_t *p_data = (uint16_t*)address;

    FCTL2 = FWKEY + FSSEL_2 + FN2;                  // 375KHz
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + WRT;

    *p_data = word;

    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
    WATCHDOG_CONTINUE;

    if ( *p_data != word ) {
        return 1;
    }
    return 0;
}

/** \brief Flash block writer.
 * Save max 128 words to flash memory  32 words. Check the write address range.
 * The first block border must be find to synchronize the writing because one block can be written at the same time. This is why do-while needed.
 * It does not check the memory data after write process.
 * \param data Ptr to words.
 * \param size count of words.
 * \param address starting address.
 * \return 0 writing process ends.
 * \return 1 data is NULL.
 * \return -1 address is out of write area.
 * */
int flash_write_data(uint16_t *data, uint8_t size, uint16_t address) {

    uint16_t *address_tmp = (uint16_t*)address;
    uint8_t size_tmp = 1;

    if ( !data || !size )
        return 1;
    if ( size > 128 )
        size = 128;
    if( !( ((uint16_t)address_tmp >= SEGMENT_0 && (uint16_t)address_tmp + size < 0xFE00)
        || ((uint16_t)address_tmp >= 0x1000 && (uint16_t)address_tmp + size <= 0x1012) ) )
        return -1;

    address_tmp = (uint16_t*)address;
    WATCHDOG_STOP;
    while ( FCTL3 & BUSY )
        ;

    while ( (size_tmp < size) && ( (uint16_t)(++address_tmp) & 0b0000000000111111) ) {  // Flash border search
            size_tmp++;
    }
    do {
        WRITE_BLOCK_PROCESS(data, size_tmp, (uint16_t**)&address);
        size -= size_tmp;
        data += size_tmp;
    if ( size > 32 )
        size_tmp = 32;
    else
        size_tmp = size;
    } while ( size );
    WATCHDOG_CONTINUE;
    return 0;
}

/** \brief Searching a key code in flash memory.
 * It checks every tagged 0xC0DE segments and compare the keys value to key param. A segment can contain max 85 key codes.
 * \ret 1 Flash memory contains the key code.
 * \ret 0 the key can not be find then *address parameter is changed to the first free place to store key if memory is not full.
 */
int flash_search_key(uint16_t *key, uint16_t *address) {
    uint8_t i, k;
    uint16_t *segment_ptr = (uint16_t*)SEGMENT_0;
    uint16_t *cur_ptr;

    *address = 0;
    while ( segment_ptr <= (uint16_t*)SEGMENT_8 ) {
        cur_ptr = segment_ptr;
        if ( *(cur_ptr++) == 0xC0DE ) {
            for ( i = 85; i > 0; i-- ) {            // todo maybe key compare function?
                k = 0;
                if ( *(cur_ptr+(k)) == *(key+(k)) ) {
                    k++;
                    if ( *(cur_ptr+(k)) == *(key+(k)) ) {
                        k++;
                        if ( *(cur_ptr+k) == *(key+k) ) {
                            *address = (uint16_t)cur_ptr;
                            return 1;   // Key found!
                        }
                    }
                }
                k = 0;
                if ( *(cur_ptr+(k++) ) == 0xFFFF )
                    if ( *(cur_ptr+(k++) ) == 0xFFFF )
                        if ( *(cur_ptr+k ) == 0xFFFF ) {
                            if ( !*address ) {
                                *address = (uint16_t)cur_ptr;
                                i = 0;  // Stop searching in this Segment! todo break
                            }
                        }
                cur_ptr += 3;
            }
        }   // Check segment flag end
        segment_ptr += 256;

    }
    return 0;
}

/** \brief Delete the specified key code.
 * "Delete" key data from the flash.
 * This function copies the data of the segment - containing the key_addr address - to the free segment (tagged by 0xEAFF).
 * In the case of no free segment had been found, key code data - started at key_addr address - will be zero: {0x0000, 0x0000, 0x0000}.
 * Key will not be deleted if the key parameter - key_addr - does not point to the first word of a key code data.
 * \param key_addr Data block (key code) started from this address will be deleted from flash.
 * \return 1 param key_addr not in the flash range.
 * \return 2 If free sector cannot be found, key code data will be zero.
 * \return 0 Key successfully deleted.
 * todo check again
 * */
int flash_delete_key(uint16_t key_addr) {
    uint16_t key_temp[21];
    uint16_t *flash_ptr;
    uint16_t *free_flash_ptr;
    uint16_t *start_free_flash_ptr;
    uint16_t *start_flash_ptr;
    uint8_t i,key_cnt,j,free_cnt;
    uint16_t zero_data[] = {0x0000,0x0000,0x0000};

    if ( key_addr > SEGMENT_8 + 510 )
        return 1;

    start_flash_ptr = (uint16_t*)key_addr;
    while ( (uint16_t)--start_flash_ptr & 511 ) {                      // Search the first address of segment containing the key to be deleted.
        if ( (uint16_t)start_flash_ptr < SEGMENT_0 )
            return 1;                   // Key_addr to low.
    }

    start_free_flash_ptr = (uint16_t*)SEGMENT_0;
    while ( *start_free_flash_ptr != 0xEAFF ) {                       // Free segment search.
        if ( start_free_flash_ptr > (uint16_t*)SEGMENT_8 ) {
            if ( !(key_addr & 511) ) {                                  // Do not write zeros in the flag word place.
                flash_write_data(zero_data, 3, key_addr);
                return 2;
            }
        }
        start_free_flash_ptr += 256;
    }

    flash_write_word(0xC0DE, (uint16_t)start_free_flash_ptr);       // Tag

    free_flash_ptr = start_free_flash_ptr + 1;
    flash_ptr = start_flash_ptr + 1;

    i = 85;
    key_cnt = 0;
    free_cnt = 3;
    while ( i && free_cnt ) {    // End of the segment?
        if ( (uint16_t)flash_ptr != key_addr ) {
            free_cnt = 3;

            for ( j = 3; j > 0; j-- ) {
                if ( *(flash_ptr) == 0xFFFF )
                    free_cnt--;
                key_temp[key_cnt++] = *(flash_ptr++);
            }
            if ( key_cnt == 21 ) {    // Check if buffer is full
                flash_write_data(key_temp, key_cnt, (uint16_t)free_flash_ptr);
                free_flash_ptr += key_cnt;
                key_cnt = 0;
            }
        }else   // Go to the next key code if the address is equal to the address of the key to be deleted.
            flash_ptr += 3;
        i--;
    }
    if ( key_cnt ) {
        if ( key_cnt <= (j =  512 - ((uint16_t)start_free_flash_ptr) - (uint16_t)free_flash_ptr ) )    // Check enough place: do not overwrite!
            flash_write_data(key_temp, key_cnt, (uint16_t)free_flash_ptr);
        else
            flash_write_data(key_temp, j, (uint16_t)free_flash_ptr);
    }
    segment_erase((uint16_t)start_flash_ptr);

    flash_write_word(0xEAFF, (uint16_t)start_flash_ptr);
    return 0;
}
/** \brief Change configuration.
 * \param change_from + FLASH_SETTING_START will be the first address.
 * \param words_to_change number of words to be changed.
 * \return -1 wrong parameter.
 * \return 1 flash write failed.
 */
int flash_change_settings(uint8_t change_from, uint16_t *data, uint8_t words_to_change) {

    uint16_t data_tmp[SETTINGS_RANGE];
    uint16_t *flash_ptr = (uint16_t*)SETTINGS_START;
    uint8_t i, k = 0;

    if ( words_to_change > SETTINGS_RANGE || data == 0 )
        return 1;
    for ( i = 0; i < SETTINGS_RANGE; i++ ) {                // Save current data
        if ( i >= change_from && i < change_from + words_to_change ) {
            data_tmp[i] = *(data+(k++));
            flash_ptr++;
        }
        else
            data_tmp[i] = *(flash_ptr++);
    }
    segment_erase(SETTINGS_START);
    flash_write_data(data_tmp, SETTINGS_RANGE, SETTINGS_START);

    flash_ptr = (uint16_t*)(change_from + SETTINGS_START);
    while ( words_to_change-- )
        if ( *(data++) != *(flash_ptr++) )
            return 1;
    return 0;
}

/**
 * Erase a segment or all.
 * If segment_addr is equal to 0 all used segment will be erased.
 */
void segment_erase(uint16_t segment_addr) {
    uint16_t *flash_ptr;
    uint8_t i;
    if ( segment_addr >= SEGMENT_0 && segment_addr <= SEGMENT_8
    || segment_addr == SETTINGS_START ) {     // Single segment erase
        i = 1;
        flash_ptr = (uint16_t*)segment_addr;
    }
    else if ( !segment_addr ) {
        i = 9;
        flash_ptr = (uint16_t*)SEGMENT_0;
    }
    else {
        i = 0;
    }
    WATCHDOG_STOP;
    for ( i; i > 0; i-- ) {
        FCTL2 = FWKEY + FSSEL_2 + FN2;                  // 375KHz
        FCTL3 = FWKEY;
        FCTL1 = FWKEY + ERASE;
        *flash_ptr = 0x0000;    // Dummy write
        FCTL3 = FWKEY + LOCK;
        flash_ptr += 256;
    }
    WATCHDOG_CONTINUE;
}














