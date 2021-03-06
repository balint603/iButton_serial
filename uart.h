/**
 * uart.h
 *
 *  Created on: 2018. szept. 29.
 *      Author: MAJXAAPPTE
 */

#ifndef UART_H_
#define UART_H_


/*________________________________ START SETTINGS ________________________________ */
#define UART_BAUD_RATE 9600
#define UART_CLK 1500000                                                      // SMCLK (Hz)
#define BAUD_SETTING_VALUE (UART_CLK / UART_BAUD_RATE)

#define RX_DATA_SIZE 130
#define UART_TIMEOUT_MS 400

#define TX_BUFFER_SIZE 50

#define START_BYTE 0x55
#define SIZE_FIELD_MAXVAL 255
/*________________________________ END SETTINGS __________________________________ */

/** UART command types */
enum UART_cmd_type{TYPE_ECHO = 1, TYPE_INFO, TYPE_TEST,
                   TYPE_GET_SETTINGS, TYPE_GET_FLASHSEGM, TYPE_ERASE_ALL,   // Not used by E-gate, used by reader program
                   TYPE_GET_SETTINGS_RE, TYPE_GET_FLASHSEGM_RE, TYPE_ERASE_ALL_RE, // E-gate response with data
                   TYPE_WRITE_SETTINGS, TYPE_WRITE_FLASHSEGM, TYPE_WRITE_A_KEY, // No response containing data, but error / ok info message
                   TYPE_WRITE_SETTINGS_RE, TYPE_WRITE_FLASHSEGM_RE, TYPE_WRITE_A_KEY_RE
                   };

/** UART error codes, used in TYPE_INFO, or response commands as data values. */
#define MSG_ERR_TIMEOUT     0x69
#define MSG_OK              0x00
#define MSG_ERR_CRC         0x02
#define MSG_ERR_SIZE        0x03
#define MSG_ERR_RANGE       0x04
#define MSG_ERR_WRITE       0x05
#define MSG_ERR_DATA        0x06
#define MSG_BOOT            0x07

#define MSG_KEY_TOUCHED     0x0A
#define MSG_KEY_TOUCHED_ILL 0x0B
#define MSG_M_KEY_TOUCHED   0x0C
#define MSG_OPEN_BUTTON     0x0D


/** UART RX packet */
typedef struct Packet {
    uint8_t type_b;
    uint8_t data_size;
    uint8_t data[RX_DATA_SIZE];
    uint16_t crc;
} packet_t;

void uart_init();
int uart_send_packet(uint8_t *data, uint8_t type, uint8_t data_size);
void uart_send_flash_segment(uint8_t *segm_start_ptr);
void uart_timeout();
void crc_do(uint8_t *data, int length, uint16_t *crc);

/** Declare these variables. */
volatile extern uint8_t RX_is_packet;
volatile extern packet_t RX_packet;
volatile extern uint16_t uart_timeot_ms;
volatile extern uint8_t uart_timeout_ticking;

#endif /* UART_H_ */
