/**
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
#define FLASH_MASTER_CODE     (0)   // Words
#define FLASH_TIME            (3)   // Words
#define FLASH_MODE            (4)   // Words
#define FLASH_ENABLE_BUTTON   (5)   // Words
#define SETTINGS_RANGE         6 // Words
#define SETTINGS_END          (SETTINGS_START + 2*SETTINGS_RANGE)

#define WATCHDOG_CONTINUE (WDTCTL = WDTPW)           // counting
#define WATCHDOG_STOP (WDTCTL = WDTPW | WDTHOLD)      // stop watchdog timer
#define WATCHDOG_RESET (WDTCTL = WDTPW | WDTCNTCL)

#define FLASH_TIME_ADDR 0x1006
#define FLASH_MODE_ADDR 0x1008
#define FLASH_ENABLE_BUTTON_ADDR 0x100A

int flash_init();

void deinit();

int flash_write_word(uint16_t word, uint16_t address);

int flash_write_data(uint16_t *data, uint8_t size, uint16_t address);

int flash_search_key(uint16_t *key, uint16_t *address);

int flash_change_settings(uint8_t change_from, uint16_t *data, uint8_t words_to_change);

int flash_delete_key(uint16_t key_addr);

void segment_erase(uint16_t segment_addr);

#endif /* FLASH_H_ */
