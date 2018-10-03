/*
 * uart.c
 *
 *  Created on: 2018. szept. 29.
 *      Author: MAJXAAPPTE
 */


#include <msp430.h>
#include <inttypes.h>
#include <intrinsics.h>

#include "uart.h"

typedef struct uart_data {
                               uint8_t g_uart_data_buffer[16];
                               volatile uint8_t g_uart_data_n;
} uart_data_t;

uart_data_t uart;


void uart_init(){
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1 |= UCSSEL_2;

    const uint16_t baud_rate_val = BAUD_SETTING_VALUE;
    UCA0BR0 = (uint8_t)(baud_rate_val);
    UCA0BR1 = (uint8_t)(baud_rate_val >> 8);

    P1SEL |= BIT1 + BIT2;
    P1SEL2 |= BIT1 + BIT2;


    UCA0CTL1 &= ~UCSWRST;

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

int uart_send_byte_data(uint8_t data){
    P1OUT |= BIT0;
    UCA0TXBUF = data;
    P1OUT &= ~BIT0;
    return 0;
}

int uart_send_ibutton_data(uint8_t *ib_code){
    P1OUT |= BIT0;
    if(!ib_code){

        return -1;
    }

    //code_conversion(ib_code, uart.g_uart_data_buffer);

    int i;
    for(i = 0; i < 8; i++)
        uart.g_uart_data_buffer[i] = ib_code[7-i];

    if(uart.g_uart_data_n){

        return 1;

    }

    uart.g_uart_data_n = 7;

    UCA0TXBUF = uart.g_uart_data_buffer[uart.g_uart_data_n];
    IE2 |= UCA0TXIE;
    P1OUT &= ~BIT0;

    return 0;
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void UART_TX_ISR(void){
    P1OUT |= BIT0;
    if(uart.g_uart_data_n){
        UCA0TXBUF = uart.g_uart_data_buffer[--uart.g_uart_data_n];
    }else
        IE2 &= ~UCA0TXIE;
    P1OUT &= ~BIT0;
}



