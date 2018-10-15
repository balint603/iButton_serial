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
 *  Hex byte to ASCII conversion.
 * */
void hex_byte_to_char(uint8_t code, char *MSB, char *LSB){
    uint8_t byte_upper;
    uint8_t byte_lower = 0;

    byte_upper = code;
    byte_upper >>= 4;
    byte_lower = code & 0x0F;
    if(byte_upper < 10)
        *MSB = byte_upper + '0';
    else
        *MSB = byte_upper + 'A' - 10;
    if(byte_lower < 10)
            *LSB = byte_lower + '0';
    else
        *LSB = byte_lower + 'A' - 10;
}

uint16_t uart_get_buffer_bytes(){
    return buf_rx.num_bytes;
}

/*int uart_send_byte_data(uint8_t data){
    P1OUT |= BIT0;
    UCA0TXBUF = data;
    P1OUT &= ~BIT0;
    return 0;
}*/
/**
 *  iButton data sending function with mode of ASCII or unsigned integer.
 * \param ib_code is a pointer to a 8 byte iButton data.
 * \param send_as_string is a lever: when not zero, the data will be sent as ASCII characters.
 * \return 1 if ib_code is NULL else return 0
 * */
int uart_send_ibutton_data(uint8_t *ib_code, uint8_t send_as_string){
    uint8_t i;
    P1OUT |= BIT0;
    if(!ib_code)
        return 1;

    if(!send_as_string){
        for(i = 0; i < 8; i++){
            if(uart_send_byte(*(ib_code+i)))
                    return 1;
        }
    }
    else{
        char MSB, LSB;
        for(i = 0; i < 8; i++){

            hex_byte_to_char(*(ib_code+i), &MSB, &LSB);
            uart_send_byte(MSB);
            uart_send_byte(LSB);
        }
    }

    return 0;
}
/**
 *  Send string via UART.
 * */
int uart_send_str(char *str, uint8_t new_line){

    if(!str)
        return -1;
    uint8_t i;
    for(i = 0; *(str+i) != '\0'; i++){
        if(i >= TX_SIZE)
            return 1;
        uart_send_byte(*(str+i));
    }
    uart_send_byte(*(str+i));       // char array end
    if(new_line){
        uart_send_byte('\n');
        uart_send_byte('\r');
    }

    return 0;
}
/**
 *  Send only one byte.
 */
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
    uint8_t byte = 0;
    if(buf_rx.num_bytes > 0){
        byte = buf_rx.data_buf[buf_rx.i_first];
        if(--buf_rx.num_bytes)
            uart_rx_buffer_not_empty_flag = 0;

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
    uart_rx_buffer_not_empty_flag = 1;
    P1OUT &= ~BIT0;
}

