/*
 * ibutton.h
 *
 *  Created on: 2018. szept. 29.
 *      Author: MAJXAAPPTE
 */

#ifndef IBUTTON_H_
#define IBUTTON_H_

#include <inttypes.h>


/** SETTINGS START */
#define n_cycle_to_us 12                // Processor counts to 1 microsecond

#define DATA_PIN         BIT0
#define DATA_PORT_INPUT  P2IN
#define DATA_PORT_DIR    P2DIR

#define LED_PIN_GR          BIT1
#define LED_PORT_GR         P2DIR
#define LED_PIN_RE          BIT2
#define LED_PORT_RE         P2DIR
/** SETTINGS END */



#define RELEASE DATA_PORT_DIR &= ~DATA_PIN
#define PULL_DOWN DATA_PORT_DIR |= DATA_PIN
#define GET_INPUT DATA_PORT_INPUT & DATA_PIN

#define LED_TURN_ON_GR LED_PORT_GR &= ~LED_PIN_GR
#define LED_TURN_OFF_GR LED_PORT_GR |= LED_PIN_GR
#define LED_TURN_ON_RE LED_PORT_RE &= ~LED_PIN_RE
#define LED_TURN_OFF_RE LED_PORT_RE |= LED_PIN_RE



void ibutton_init();

int ibutton_test_presence();

int ibutton_read_it(uint8_t *data);



#endif /* IBUTTON_H_ */
