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

#define TX_SIZE 50
#define RX_SIZE 200

typedef struct {
    uint8_t data_buf[TX_SIZE];
    uint16_t i_first;
    uint16_t i_last;
    uint16_t num_bytes;
}buf_out_t;

typedef struct {
    uint8_t data_buf[RX_SIZE];
    uint16_t i_first;
    uint16_t i_last;
    uint16_t num_bytes;
}buf_in_t;

buf_in_t buf_rx;
buf_out_t buf_tx;



void uart_init(){
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1 |= UCSSEL_2;

    const uint16_t baud_rate_val = BAUD_SETTING_VALUE;
    UCA0BR0 = (uint8_t)(baud_rate_val);
    UCA0BR1 = (uint8_t)(baud_rate_val >> 8);

    P1SEL |= BIT1 + BIT2;
    P1SEL2 |= BIT1 + BIT2;


    UCA0CTL1 &= ~UCSWRST;
    IE2 |= UCA0RXIE;
}

/**
 *  code_example[] = {0x01,0x55,0xf4,0x45,0xe5,0x3a,0x33,0x30}
 * */
static void code_conversion(uint8_t *code, char *buffer){
    int i;
    uint8_t byte_upper;
    uint8_t byte_lower = 0;

    for(i = 0; i < 8; i++){
        byte_upper = *(code+i);
        byte_lower = byte_upper & 0x0F;
        byte_upper >>= 4;

        *(buffer+15-i) = byte_upper;
        *(buffer+14-i) = byte_lower;
    }
}

/*int uart_send_byte_data(uint8_t data){
    P1OUT |= BIT0;
    UCA0TXBUF = data;
    P1OUT &= ~BIT0;
    return 0;
}*/

int uart_send_ibutton_data(uint8_t *ib_code){
    uint8_t i;
    for(i = 0; i < 8; i++){
        if(uart_send_byte(*(ib_code+i)))
                return 1;
    }
    return 0;
}

/*int uart_send_str(char *str){
    P1OUT |= BIT0;

    if(!str)
        return -1;
    int i;
    for(i = 0; i < str[i] != '\0' && i < (OUTPUT_SIZE -1) ; i++)
        uart.output_buf[i] = str[i];
    uart.output_buf[i] = '\0';

    uart.output_n = 1;
    UCA0TXBUF = uart.output_buf[0];
    IE2 |= UCA0TXIE;

    P1OUT &= ~BIT0;
    return 0;
}*/

int uart_send_byte(uint8_t byte){
    __disable_interrupt();
    if(buf_tx.num_bytes < TX_SIZE){
        buf_tx.data_buf[buf_tx.i_last] = byte;
        buf_tx.i_last++;
        buf_tx.num_bytes++;
    }else
        return 1;

    if(buf_tx.i_last == TX_SIZE)
        buf_tx.i_last = 0;
    IE2 |= UCA0TXIE;
    __enable_interrupt();
    return 0;
}

uint8_t uart_get_byte(){
    __disable_interrupt();
    uint8_t byte = 1;
    if(buf_rx.num_bytes > 0){
        byte = buf_rx.data_buf[buf_rx.i_first];
        buf_rx.num_bytes--;
        if(++buf_rx.i_first == RX_SIZE)
            buf_rx.i_first = 0;
    }
    __enable_interrupt();
    return byte;
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void UART_TX_ISR(void){
    P1OUT |= BIT0;
    if(buf_tx.num_bytes > 0){
        UCA0TXBUF = buf_tx.data_buf[buf_tx.i_first];

        if(buf_tx.i_first == (TX_SIZE-1))
            buf_tx.i_first = 0;
        else
            buf_tx.i_first++;

        if(--buf_tx.num_bytes == 0)
            IE2 &= ~UCA0TXIE;
    }



    P1OUT &= ~BIT0;
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void UART_RX_ISR(void){
    P1OUT |= BIT0;
    if(buf_rx.num_bytes < RX_SIZE){

        buf_rx.data_buf[buf_rx.i_last] = UCA0RXBUF;
        buf_rx.i_last++;
        buf_rx.num_bytes++;

        if(buf_rx.i_last == RX_SIZE)
                buf_rx.i_last = 0;
    }else
        UC0IFG  &= ~UCA0RXIFG;
    P1OUT &= ~BIT0;
}

