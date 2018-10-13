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

#define SDA_PIN         BIT4
#define SDA_PORT_INPUT  P2IN
#define SDA_PORT_DIR   P2DIR
#define EEPROM_SIZE 32              /**  KB */

#define EEPROM_MASTER_KEY_PLACE 32760
#define EEPROM_N_OF_PAGES 512
#define EEPROM_SIZE_OF_PAGE 64

#define SCL_PIN BIT5
#define SCL_PORT_OUT P2OUT

#define BIT_TIME 5

int EEPROM_clear_ff();

void EEPROM_init();

int EEPROM_key_read(uint8_t *data, uint16_t address);

int EEPROM_key_write(uint8_t *data, uint16_t address);

uint16_t EEPROM_get_key_or_empty_place(uint8_t *key_code, uint16_t *addr, uint16_t addr_limit, uint8_t reading_dir);



#endif /* EEPROM_H_ */
