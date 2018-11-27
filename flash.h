/*
 * flash.h
 *
 *  Created on: 2018. nov. 7.
 *      Author: MAJXAAPPTE
 */

#ifndef FLASH_H_
#define FLASH_H_

#include <inttypes.h>

#define SEGMENT_0 0xE400
#define SEGMENT_8 0xF400

#define SETTINGS_START        (0x1000)
#define FLASH_MASTER_CODE     (0x1000)
#define FLASH_TIME            (0x1006)
#define FLASH_MODE            (0x1008)
#define FLASH_ENABLE_BUTTON   (0x1010)
#define SETTINGS_RANGE 6 // Words


int flash_init();

void deinit();

int flash_write_word(uint16_t word, uint16_t address);

int flash_write_data(uint16_t *data, uint8_t size, uint16_t address);

int flash_search_key(uint16_t *key, uint16_t *address);

/**
 * \param setting_id TIME MODE or MASTER_CODE settings can be changed.
 * \ret 0 If new settings are saved.
 * \ret 1 Flash problem.
 * */
int flash_change_setting(uint8_t setting_id, uint16_t *value);

int flash_delete_key(uint16_t key_addr);

void segment_erase(uint16_t segment_addr);

#endif /* FLASH_H_ */
