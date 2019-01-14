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


/** UART command types */
#define CMD_ECHO 1

/** UART RX buffer type */
typedef struct Packet {
    uint8_t cmd_b;
    uint8_t data_size;
    uint8_t data[RX_TX_DATA_SIZE];
    uint16_t crc;
} Packet_t;

/** UART states */
typedef enum UART_state{GET_START,GET_CMD,GET_SIZE,GET_DATA,GET_CRC_1,GET_CRC_0} state_t;

state_t RX_state = GET_START;

volatile uint8_t TX_is_packet;


uint8_t RX_data_cnt;

uint8_t TX_buffer_cnt;
uint8_t TX_buffer_ptr;


Packet_t RX_packet;

uint8_t TX_buffer[56];

static uint16_t crc_do(uint8_t *data, int length);
int uart_send(uint8_t *data, uint8_t cmd, uint8_t data_size);

/**
 * UART initialization need to be called at initialization.
 * Set baudrate, HW UART, enable RX interrupt.
 * */
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

/**
 * Processing incoming UART commands.
 * See defined commands.
 * */
void uart_process_command(){
    switch (RX_packet.cmd_b) {
        case CMD_ECHO:
            if(!uart_send(RX_packet.data, 0, RX_packet.data_size));
                RX_is_packet = 0;
            break;
        default:
            break;
    }
}

/**
 * TX function
 * \param data_size: bytes of data, max 51 byte.
 * \param cmd: see defined commands.
 * \param *data: pointer to data.
 * \return 1 when: TX buffer is busy, size of data limitation, data is null.
 *
 * */
int uart_send(uint8_t *data, uint8_t cmd, uint8_t data_size){
    __disable_interrupt();
    uint8_t i = 0;
    uint16_t crc_value;
    uint16_t crc_value_msb = 0;
    if(TX_is_packet)
        return 1;
    if(!data)
        return 1;
    if(data_size > 51)
        return 1;

    TX_buffer_ptr = 0;
    TX_buffer[i++] = 0x55;
    TX_buffer[i++] = cmd;
    TX_buffer[i++] = data_size + 2;

    while(data_size--)
        TX_buffer[i++] = *(data++);

    crc_value = crc_do(TX_buffer, i);
    crc_value_msb = crc_value;
    crc_value_msb >>= 8;
    TX_buffer[i++] = (uint8_t)crc_value_msb;
    TX_buffer[i++] = (uint8_t)(crc_value);
    TX_buffer_cnt = i;
    TX_is_packet = 1;
    IE2 |= UCA0TXIE;
    __enable_interrupt();
    return 0;
}
/**
 * need to test
 * */
static uint16_t crc_do(uint8_t *data, int length){
    uint8_t i;
    uint16_t w_crc = 0xffff;
    while (length--) {
        w_crc ^= *(uint8_t *)data++ << 8;
        for (i = 8; i > 0; i--)
            w_crc = w_crc & 0x8000 ? (w_crc << 1) ^ 0x1021 : w_crc << 1;
    }
    return w_crc & 0xffff;
}


#pragma vector=USCIAB0TX_VECTOR
__interrupt void UART_TX_ISR(void){
    if(TX_buffer_cnt--){
        UCA0TXBUF = TX_buffer[TX_buffer_ptr++];
    }else{
        TX_is_packet = 0;
        IE2 &= ~UCA0TXIE;
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void UART_RX_ISR(void){
    if(!RX_is_packet){
        switch (RX_state) {
            case GET_START:
                if(UCA0RXBUF == 0x55){
                    RX_data_cnt = 0;
                    RX_state++;
                }
                break;
            case GET_CMD:
                RX_packet.cmd_b = UCA0RXBUF;
                // todo check
                RX_state++;
                break;
            case GET_SIZE:
                RX_packet.data_size = UCA0RXBUF;
                if(!RX_packet.data_size)
                    RX_state = GET_CRC_0;
                else
                    RX_state++;
                // todo check
                break;
            case GET_CRC_1:
                RX_packet.crc = ( ((uint16_t)UCA0RXBUF) << 8 );
                RX_state++;
                break;
            case GET_CRC_0:
                RX_packet.crc |= (uint16_t)UCA0RXBUF;
                RX_state = GET_START;
                RX_is_packet = 1;
                break;
            case GET_DATA:
                if(RX_data_cnt < RX_packet.data_size){
                    RX_packet.data[RX_data_cnt++] = UCA0RXBUF;
                }else{
                    RX_data_cnt = 0;
                    RX_state = GET_CRC_0;
                }
                break;
            default:
                //
                break;
        }
    }
    UC0IFG &= ~UCA0RXIFG;
    // todo busy message to send
}

