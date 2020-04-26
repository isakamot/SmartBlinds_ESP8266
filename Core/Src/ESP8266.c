#include "main.h"
#include "ESP8266.h"

int esp8266_reset(UART_HandleTypeDef * UARTx){
	uint8_t AT_RST[14] = "AT+RESTORE\r\n";

	//Reset ESP8266
	HAL_UART_Transmit(UARTx, AT_RST, sizeof(AT_RST), 100);

	HAL_Delay(1000);

	return ESP8266_SUCCESS;
}

int esp8266_init(UART_HandleTypeDef * UARTx){
	uint8_t AT[4] = "AT\r\n";
	uint8_t AT_resp[sizeof(AT)+7];

	uint8_t AT_CWMODE[13] = "AT+CWMODE=3\r\n";
	uint8_t AT_CWMODE_resp[sizeof(AT_CWMODE)+7];

	//Set up AP network name and pass word: AT+CWSAP\"<name>\", \"<password>\",1,4\r\n
	uint8_t AT_CWSAP[45] = "AT+CWSAP=\"SmartBlinds1\",\"group2blinds\",1,4\r\n";
	uint8_t AT_CWSAP_resp[sizeof(AT_CWSAP)+6];

	//Have a counter to re-send data if it fails the first time
	int tries;
	int success_flag;
	HAL_StatusTypeDef status;

	//Check if ESP8266 is connected
	tries = 0;
	success_flag = 0;
	status = HAL_ERROR;
	while (success_flag == 0){

		//Reset Receiver Buffer
		HAL_UART_AbortReceive(UARTx);

		tries++;
		if (tries == 5){
			return ESP8266_FAILED;
		}
		HAL_UART_Transmit(UARTx, AT, sizeof(AT), 100);
		status = HAL_UART_Receive(UARTx, AT_resp, sizeof(AT_resp), 100);
		if (status == HAL_OK){
			success_flag = 1;
		}
	}

	//Set up ESP8266 as Soft AP
	tries = 0;
	success_flag = 0;
	status = HAL_ERROR;
	while (success_flag == 0){

		//Reset Receiver Buffer
		HAL_UART_AbortReceive(UARTx);

		tries++;
		if (tries == 5){
			return ESP8266_FAILED;
		}
		HAL_UART_Transmit(UARTx, AT_CWMODE, sizeof(AT_CWMODE), 100);
		status = HAL_UART_Receive(UARTx, AT_CWMODE_resp, sizeof(AT_CWMODE_resp), 100);
		if (status == HAL_OK){
			success_flag = 1;
		}
	}

	//Set up ESP8266's Network
	tries = 0;
	success_flag = 0;
	status = HAL_ERROR;
	while (success_flag == 0){

		//Reset Receiver Buffer
		HAL_UART_AbortReceive(UARTx);

		tries++;
		if (tries == 10){
			return ESP8266_FAILED;
		}
		HAL_UART_Transmit(UARTx, AT_CWSAP, sizeof(AT_CWSAP), 100);
		status = HAL_UART_Receive(UARTx, AT_CWSAP_resp, sizeof(AT_CWSAP_resp), 300);
		if (status == HAL_OK){
			success_flag = 1;
		}
	}

	return ESP8266_SUCCESS;
}

int esp8266_setupTCP(UART_HandleTypeDef * UARTx){

	uint8_t AT_CIPMUX[13] = "AT+CIPMUX=1\r\n";
	uint8_t AT_CIPMUX_resp[sizeof(AT_CIPMUX)+7] = "";

	uint8_t AT_CIPSERVER[19] = "AT+CIPSERVER=1,80\r\n";
	uint8_t AT_CIPSERVER_resp[sizeof(AT_CIPSERVER)+7] = "";

	//Have a counter to re-send data if it fails the first time
	int tries;
	int success_flag;
	HAL_StatusTypeDef status;

	//Enable multiple connection - this is necessary for setting up TCP Server
	tries = 0;
	success_flag = 0;
	status = HAL_ERROR;
	while (success_flag == 0){

		//Reset Receiver Buffer
		HAL_UART_AbortReceive(UARTx);

		tries++;
		if (tries == 5){
			return ESP8266_FAILED;
		}
		HAL_UART_Transmit(UARTx, AT_CIPMUX, sizeof(AT_CIPMUX), 100);
		status = HAL_UART_Receive(UARTx, AT_CIPMUX_resp, sizeof(AT_CIPMUX_resp), 100);
		if (status == HAL_OK){
			success_flag = 1;
		}
	}

	//Setup TCP Server
	tries = 0;
	success_flag = 0;
	status = HAL_ERROR;
	while (success_flag == 0){

		//Reset Receiver Buffer
		HAL_UART_AbortReceive(UARTx);


		tries++;
		if (tries == 5){
			return ESP8266_FAILED;
		}
		HAL_UART_Transmit(UARTx, AT_CIPSERVER, sizeof(AT_CIPSERVER), 100);
		status = HAL_UART_Receive(UARTx, AT_CIPSERVER_resp, sizeof(AT_CIPSERVER_resp), 100);
		if (status == HAL_OK){
			success_flag = 1;
		}
	}

	//Reset Receiver Buffer
	HAL_UART_AbortReceive(UARTx);

	return ESP8266_SUCCESS;
}

int make_ATCWJAP_command(char * AT_command, char * ssid, char * password){
	int ssid_length, password_length;
    int command_len = 9;

	ssid_length = 1;
	password_length = 1;

	while(ssid[ssid_length-1] != '\0'){
		AT_command[9+ssid_length-1] = ssid[ssid_length-1];
		ssid_length ++;
		command_len++;
	}

	AT_command[9+ssid_length-1] = ',';
	command_len++;


	while(password[password_length-1] != '\0'){
		AT_command[9+(ssid_length)+(password_length-1)] = password[password_length-1];
		password_length ++;
		command_len++;
	}

	AT_command[command_len] = '\r';
	command_len++;
	AT_command[command_len] = '\n';
	command_len++;

	return command_len;
}

int esp8266_check_ip_address(UART_HandleTypeDef * UARTx){
	uint8_t at_command[10] = "AT+CIFSR\r\n";
	uint8_t rec[100];

	//Reset Receiver Buffer
	HAL_UART_AbortReceive(UARTx);

	//Send Command to get IP address
	HAL_UART_Transmit(UARTx, at_command, sizeof(at_command), 100);

	//Check IP address
	HAL_UART_Receive(UARTx, rec, sizeof(rec), 100);

	return ESP8266_SUCCESS;
}

void esp8266_getMACaddr(UART_HandleTypeDef * UARTx){
	uint8_t AT_GMR[8] = "AT+GMR\r\n";

	HAL_UART_Transmit(UARTx, AT_GMR, sizeof(AT_GMR), 100);


}

int esp8266_connectWifi(UART_HandleTypeDef * UARTx, char * ssid, char * password){
	uint8_t AT_CIPSERVER[16] = "AT+CIPSERVER=0\r\n";
	uint8_t AT_CIPSERVER_resp[sizeof(AT_CIPSERVER)+7];

	uint8_t AT_CWJAP[50] = "AT+CWJAP=";
	int AT_CWJAP_size = make_ATCWJAP_command((char*) AT_CWJAP, ssid, password);
	uint8_t resp[100];

	//Have a counter to re-send data if it fails the first time
	int tries;
	int success_flag;
	HAL_StatusTypeDef status;

	//Reset Receiver Buffer
	HAL_UART_AbortReceive(UARTx);

	//Close TCP Server
	tries = 0;
	success_flag = 0;
	status = HAL_ERROR;
	while (success_flag == 0){
		tries++;
		if (tries == 5){
			return ESP8266_FAILED;
		}
		HAL_UART_Transmit(UARTx, AT_CIPSERVER, sizeof(AT_CIPSERVER), 100);
		status = HAL_UART_Receive(UARTx, AT_CIPSERVER_resp, sizeof(AT_CIPSERVER_resp), 100);
		if (status == HAL_OK){
			success_flag = 1;
		}
	}

	//Connect to WiFi
	HAL_UART_Transmit(UARTx, AT_CWJAP, AT_CWJAP_size, 100);
	HAL_UART_Receive(UARTx, resp, 100, 6000);


	if (strstr((char*) resp, (char *) "WIFI CONNECTED") != NULL){
		return ESP8266_SUCCESS;
	}
	else{
		return ESP8266_FAILED;
	}

}

int make_ATcommand_sendmsg(char * ATcommand, int message_length){
	char string_num[5] = "";
	int ATcommand_index = 13;
	int string_num_index = 0;
	int command_len = 13;

	//Convert int number to string number
	itoa(message_length, string_num,10);

	//Construct AT command by first putting size of message
	while(string_num[string_num_index] != '\0'){
		ATcommand[ATcommand_index] = string_num[string_num_index];
		ATcommand_index++;
		string_num_index++;
		command_len++;
	}

	ATcommand[command_len] = '\r';
	command_len++;
	ATcommand[command_len] = '\n';
	command_len++;

	return command_len;
}


int esp8266_sendmsg(UART_HandleTypeDef * UARTx, char * message, int message_length){
	uint8_t at_command[50] = "AT+CIPSEND=0,";
	int command_len = make_ATcommand_sendmsg((char *) at_command, message_length);
	uint8_t at_command_resp[command_len+7];

	//Have a counter to re-send data if it fails the first time
	int tries;
	int success_flag;
	HAL_StatusTypeDef status;

	//Set up the ATcommand to send messsage
	tries = 0;
	success_flag = 0;
	status = HAL_ERROR;
	while (success_flag == 0){

		//Reset Receiver Buffer
		HAL_UART_AbortReceive(UARTx);

		tries++;
		if (tries == 5){
			return ESP8266_FAILED;
		}
		HAL_UART_Transmit(UARTx, at_command, command_len, 100);
		status = HAL_UART_Receive(UARTx, at_command_resp, sizeof(at_command_resp), 100);
		if (status == HAL_OK){
			success_flag = 1;
		}
	}

	//Send message
	HAL_UART_Transmit(UARTx, (uint8_t*) message, message_length, 100);
	//Wait for some time till message is sent
	HAL_Delay(800);

	return ESP8266_SUCCESS;
}

int esp8266_check_wifi_connection(UART_HandleTypeDef * UARTx){
	uint8_t AT_CIPSTATUS[14] = "AT+CIPSTATUS\r\n";
	uint8_t AT_CIPSTATUS_resp[50] = "";


	//Wait until ESP8266 boots up
	HAL_Delay(5000);

	HAL_UART_Transmit(UARTx, AT_CIPSTATUS, sizeof(AT_CIPSTATUS), 100);

	//Reset Receiver Buffer
	HAL_UART_AbortReceive(UARTx);

	HAL_UART_Receive(UARTx, AT_CIPSTATUS_resp, sizeof(AT_CIPSTATUS_resp),500);

	if (strstr((char*)AT_CIPSTATUS_resp, (char*) "5") != NULL){
		//Not connected to WiFi
		return 0;
	}

	else if (strstr((char*)AT_CIPSTATUS_resp, (char*) "2") != NULL) {
		return 1;
	}

	return -1;
}


