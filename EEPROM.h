/*
 * EEPROM.h
 *
 *  Created on: 2018. okt. 10.
 *      Author: MAJXAAPPTE
 */

#ifndef EEPROM_H_
#define EEPROM_H_


#include <inttypes.h>

#ifndef n_cycle_to_us
#define n_cycle_to_us 12
#endif

/*________________________________ START SETTINGS ________________________________ */

/*     PORTS      */
#define SDA_PIN         BIT4
#define SDA_PORT_INPUT  P2IN
#define SDA_PORT_DIR   P2DIR
#define EEPROM_SIZE 32 /* KB */

#define SCL_PIN BIT5
#define SCL_PORT_OUT P2OUT

/*     EEPROM specific parameters      */
#define EEPROM_LAST_KEY_SPACE 32744 /* B */
#define EEPROM_TIME_SETTING_PLACE (EEPROM_LAST_KEY_SPACE + 8)

#define EEPROM_MASTER_KEY_PLACE (EEPROM_LAST_KEY_SPACE + 16)
#define EEPROM_N_OF_PAGES 512
#define EEPROM_SIZE_OF_PAGE 64 /* B */

#define BIT_TIME 5 /* up/down time */
/*________________________________ END SETTINGS ________________________________ */

int EEPROM_clear_ff();

void EEPROM_init();

int EEPROM_key_write(uint8_t *data, uint16_t address);

int EEPROM_write_n_byte(uint8_t *data, uint8_t size, uint16_t address);

int EEPROM_key_read(uint8_t *data, uint16_t address);

int EEPROM_read_byte(uint8_t *byte, uint16_t address);

uint16_t EEPROM_get_first_free_addr(uint16_t *cur_addr);

int EEPROM_search_key(uint8_t *key_code, uint16_t addr_limit);

uint16_t EEPROM_get_key_or_empty_place(uint8_t *key_code, uint16_t *addr, uint16_t *first_free_addr, uint16_t addr_limit, uint8_t reading_dir);



#endif /* EEPROM_H_ */
