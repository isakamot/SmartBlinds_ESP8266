/*
 * ESP8266.h
 *
 *  Created on: Apr 15, 2020
 *      Author: Itsuki Sakamoto
 */

#ifndef INC_ESP8266_H_
#define INC_ESP8266_H_

int esp8266_check_wifi_connection(UART_HandleTypeDef * UARTx);
int esp8266_reset(UART_HandleTypeDef * UARTx);
int esp8266_init(UART_HandleTypeDef * UARTx);
int esp8266_setupTCP(UART_HandleTypeDef * UARTx);
int esp8266_check_ip_address(UART_HandleTypeDef * UARTx);
int esp8266_connectWifi(UART_HandleTypeDef * UARTx, char * ssid, char * password);
int make_ATcommand_sendmsg(char * ATcommand, int message_length);
int find_receive_buffer_size(int message_length);
int esp8266_sendmsg(UART_HandleTypeDef * UARTx, char * message, int message_length);
void esp8266_getMACaddr(UART_HandleTypeDef * UARTx);

#define ESP8266_FAILED -1
#define ESP8266_SUCCESS 1

#endif /* INC_ESP8266_H_ */
