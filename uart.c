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
#include "flash.h"


/** UART states */
typedef enum UART_state{GET_START,GET_TYPE,GET_SIZE,GET_DATA,GET_CRC_1,GET_CRC_0} state_t;

/** UART RX buffer type */

volatile state_t RX_state = GET_START;
uint8_t RX_data_cnt;

/** UART TX */
typedef struct {
    uint8_t data_buf[50];
    uint8_t i_first;
    uint8_t i_last;
    uint8_t num_bytes;
}buf_t;

buf_t TX_buffer;
/*
volatile uint8_t TX_is_packet;
uint8_t TX_buffer_cnt;
uint8_t *TX_buffer_ptr;
uint8_t TX_buffer[56];
*/


int uart_send(uint8_t *data, uint8_t type, uint8_t data_size);
static void crc_do(uint8_t *data, int length, uint16_t *crc);

#define UART_RESET_TIMEOUT() {uart_timeout_ticking = 1; uart_timeot_ms = UART_TIMEOUT_MS;}


void uart_timeout(){
    uint8_t data = 69;
    RX_state = GET_START;
    uart_send_packet(&data, TYPE_INFO, 1);
}

/**
 * \param crc_init null, from 0x0000
 * */
static void crc_do(uint8_t *data, int length, uint16_t *crc){
    uint8_t i;
    while (length--) {
        (*crc) ^= *(uint8_t *)data++ << 8;
        for (i = 8; i > 0; i--)
            (*crc) = (*crc) & 0x8000 ? ((*crc) << 1) ^ 0x1021 : (*crc) << 1;
    }
    *crc &= 0xFFFF;
}

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

int uart_send_byte(uint8_t byte){
    __disable_interrupt();
    if(TX_buffer.num_bytes < TX_BUFFER_SIZE){
        TX_buffer.data_buf[TX_buffer.i_last++] = byte;
        TX_buffer.num_bytes++;
    }else
        return 1;

    if(TX_buffer.i_last == TX_BUFFER_SIZE)
        TX_buffer.i_last = 0;
    return 0;
}

/**
 * Transmit a short of data
 * \param type: see defined commands.
 * \param *data: pointer to data.
 * \return 1 when: TX buffer is busy, size of data limitation, data is null.
 *
 * */
int uart_send_packet(uint8_t *data, uint8_t type, uint8_t data_size){
    __disable_interrupt();
    uint16_t crc_value = 0xFFFF;
    uint16_t crc_value_msb = 0;

    if(!data)
        return 1;
    if(data_size > RX_DATA_SIZE)
        return 1;

    crc_do(&type, 1, &crc_value);
    crc_do(&data_size, 1, &crc_value);
    crc_do(data, data_size, &crc_value);
    crc_value_msb = crc_value;
    crc_value_msb >>= 8;

    uart_send_byte(0x55);
    uart_send_byte(type);
    uart_send_byte(data_size);
    while(data_size--)
        uart_send_byte(*(data++));
    uart_send_byte((uint8_t)crc_value_msb);
    uart_send_byte((uint8_t)crc_value);

    IE2 |= UCA0TXIE;
    __enable_interrupt();
    return 0;
}

/**
 * Stream out 256byte content of memory.
 * \param *flash_ptr starting address
 * */
void uart_send_flash_data(uint8_t *flash_ptr){
    uint8_t i;
    uint16_t k;
    uint8_t type_arr[] = {TYPE_SEND_DATA_CODES, 0};
    uint16_t crc_val[2];
    crc_val[0] = 0xFFFF;

    WATCHDOG_STOP;
    while(TX_buffer.num_bytes)
        ;

    while(!(IFG2 & UCA0TXIFG))
        ;
    UCA0TXBUF = 0x55;

    for(k = 0; k < 2; k++){
        while(!(IFG2 & UCA0TXIFG))
            ;
        UCA0TXBUF = type_arr[k];
        crc_do(&type_arr[k], 1, &crc_val[0]);
    }

    for(k = 256; k > 0; k--){
        while(!(IFG2 & UCA0TXIFG))
            ;
        UCA0TXBUF = *(flash_ptr);
        crc_do(flash_ptr++, 1, &crc_val[0]);
    }

    crc_val[1] = crc_val[0];
    crc_val[1] >>= 8;
    for(k = 2; k > 0; ){
        while(!(IFG2 & UCA0TXIFG))
            ;
        UCA0TXBUF = (uint8_t)crc_val[--k];
    }
    WATCHDOG_CONTINUE;
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void UART_TX_ISR(void){
    if(TX_buffer.num_bytes > 0){
        UCA0TXBUF = TX_buffer.data_buf[TX_buffer.i_first];

        if(TX_buffer.i_first == (TX_BUFFER_SIZE-1))
            TX_buffer.i_first = 0;
        else
            TX_buffer.i_first++;

        if(--TX_buffer.num_bytes == 0)
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
                    UART_RESET_TIMEOUT();
                }
                break;
            case GET_TYPE:
                RX_packet.type_b = UCA0RXBUF;
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
                uart_timeout_ticking = 0;
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

