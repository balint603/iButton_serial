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


#define SCL_PIN BIT5
#define SCL_PORT_OUT P2OUT

#define BIT_TIME 50

void EEPROM_init();

int key_read(uint8_t *data, uint8_t datasize, uint16_t address);

int key_write(uint8_t *data, uint8_t datasize, uint16_t address);



#endif /* EEPROM_H_ */
