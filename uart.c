/*
 * uart.c
 *
 *  Created on: 2018. szept. 29.
 *      Author: MAJXAAPPTE
 */


#include <msp430.h>
#include <inttypes.h>
#include <intrinsics.h>
#include <string.h>

#include "uart.h"
#include "fsm.h"

#define GET_START 0
#define GET_CMD 1
#define GET_length 2
#define DATA_START 3
#define GET_CRC_0 54
#define GET_CRC_1 55


volatile uint8_t RX_state;
volatile uint8_t TX_state;
uint8_t g_data_cnt;

typedef struct Packet {
    uint8_t cmd_b;
    uint8_t length;
    uint8_t data[RX_TX_DATA_SIZE];
    uint16_t crc;
} Packet_t;

Packet_t RX_packet;
Packet_t TX_packet;


void uart_init(){
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1 |= UCSSEL_2;

    const uint16_t baud_rate_val = BAUD_SETTING_VALUE;
    UCA0BR0 = (uint8_t)(baud_rate_val);
    UCA0BR1 = (uint8_t)(baud_rate_val >> 8);

    P1SEL |= BIT1 + BIT2;
    P1SEL2 |= BIT1 + BIT2;
    P1OUT |= BIT2;

    UCA0CTL1 &= ~UCSWRST;
    IE2 |= UCA0RXIE;
}


void uart_process_command(){
    switch (RX_packet.cmd_b) {
        case 69:
            ibutton_fsm_put_input(button_pressed);
            RX_is_packet = 0;
            break;
        default:
            break;
    }
}


#pragma vector=USCIAB0TX_VECTOR
__interrupt void UART_TX_ISR(void){

}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void UART_RX_ISR(void){
    if(!RX_is_packet){
        switch (RX_state) {
            case GET_START:
                if(UCA0RXBUF == 0x55){
                    g_data_cnt = 0;
                    RX_state++;
                }
                break;
            case GET_CMD:
                RX_packet.cmd_b = UCA0RXBUF;
                // todo check
                RX_state++;
                break;
            case GET_length:
                RX_packet.length = UCA0RXBUF;
                if(RX_packet.length == 2)
                    RX_state = GET_CRC_0;
                else
                    RX_state++;
                // todo check
                break;
            case GET_CRC_0:
                RX_packet.crc = (uint16_t)UCA0RXBUF;
                RX_state++;
                break;
            case GET_CRC_1:
                RX_packet.crc |= ( ((uint16_t)UCA0RXBUF) << 8 );
                RX_is_packet = 1;
                RX_state = 0;
                break;
            default:
                RX_packet.data[g_data_cnt++];
                if(RX_state >= RX_packet.length){
                    g_data_cnt = 0;
                    RX_state = GET_CRC_0;
                }
                else
                    RX_state++;
                break;
        }

    }
    // todo busy message to send
}

