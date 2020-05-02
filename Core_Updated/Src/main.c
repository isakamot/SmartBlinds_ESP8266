/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ESP8266.h"
#include "string_manipulation.h"
#include "adc_stuff.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
enum states current_state;
uint8_t message[50];
int message_index;
int receive_flag;
int message_len;
int staip_size;
char wifi_name[50] = "";
char wifi_password[50] = "";
int configured_flag;
int close_temp;
int open_temp;
int bright_flag;
int dark_flag;
int curr_temp;
int curr_bat;
int curr_pos;
char time_open[10] = "";
char time_close[10] = "";
char manual_control[15] = "";
int manual_control_flag = 0;
char cur_time[50] = "";
float curr_milli = 0;
float milli = 0;
int Bright_Outside;
struct time time_cur, close_time, open_time;
int done_config;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Motor_Init(void)
{
	/* Previously used:
	   	   TIM2->CH1 for motor up (PA5)
		   TIM2->CH2 for motor down (PA1)
	       TIM2->CH3 for motor open/close (PA2)
	       TIM2->CH4 for LED (PA3)
	 */

	/* Currently used:
	 	 PA0: Motor Up (TIM2->CH1)
	 	 PA1: Motor Down (TIM2->CH2)
	 	 PA2: Motor Open (TIM3->CH3)
	 	 PA3: Motor Close (TIM3->CH4l8
	 */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;


	GPIOA->MODER &= ~(3 << (2 * 0));	//Clear Moder 0
	GPIOA->MODER &= ~(3 << (2 * 1));	//Clear Moder 1
	GPIOA->MODER &= ~(3 << (2 * 2));	//Clear Moder 2
	GPIOA->MODER &= ~(3 << (2 * 3));	//Clear Moder 3

	GPIOA->MODER |= (2 << (2 * 0));		//Set PA5 to Alternate Function Mode
	GPIOA->MODER |= (2 << (2 * 1));		//Set PA1 Alternative Function Mode
	GPIOA->MODER |= (2 << (2 * 2));		//Set PA2 to Alternative Function Mode
	GPIOA->MODER |= (2 << (2 * 3));		//Set PA3 to Alternative Function Mode

	GPIOA->AFR[0] &= ~(0xF << (4 * 0));		//Clear AF5
	GPIOA->AFR[0] &= ~(0xF << (4 * 1));		//Clear AF3
	GPIOA->AFR[0] &= ~(0xF << (4 * 2));		//Clear AF2
	GPIOA->AFR[0] &= ~(0xF << (4 * 3));		//Clear AF1

	GPIOA->AFR[0] |= (2 << (4 * 0));	//Set to PA5 to AF1
	GPIOA->AFR[0] |= (2 << (4 * 1));	//Set PA1 to AF1
	GPIOA->AFR[0] |= (2 << (4 * 2));	//Set PA2 to AF1
	GPIOA->AFR[0] |= (2 << (4 * 3));	//Set PA3 to AF1

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;		//Enable Timer2 in RCC
	TIM2->PSC = 480 - 1;
	TIM2->ARR = 100 - 1;

	TIM2->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;	//TIM2->CH1
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;	//TIM2->CH2
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;	//TIM2->CH3
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;	//TIM2->CH4


	TIM2->CCER |= TIM_CCER_CC1E;	//Enable output in capture/control register 1
	TIM2->CCER |= TIM_CCER_CC2E;	//Enable output in capture/control register 2
	TIM2->CCER |= TIM_CCER_CC3E;
	TIM2->CCER |= TIM_CCER_CC4E;

	TIM2->CR1 |= TIM_CR1_CEN;		//Enable timer counter

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	return;
}

void delay(float val)
{
	int i, max = (int)(val * 50000);
	for(i = 0; i < max; i++);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (current_state == MANUAL){
		manual_control_flag = 1;
	}
	else{
		receive_flag = 1;
		message_index++;
		message_len++;
	}
}

void TIM3_IRQHandler()
{
	if((TIM3->SR & TIM_SR_UIF) != 0)
	{
		TIM3->SR &= ~1;
		return;
	}

	if(TIM3->SR & TIM_SR_CC1IF)
	{
		TIM2->CCR1 = 100;
		int __attribute((unused)) useless;
		useless = TIM2->CCR1;
		//return;
	}
	if(TIM3->SR & TIM_SR_CC2IF)
	{
		TIM2->CCR2 = 100;
		int __attribute((unused)) useless;
		useless = TIM2->CCR2;
	}
	if(TIM3->SR & TIM_SR_CC3IF)		//Blinds open
	{
		while((GPIOB->IDR & (1 << 0)) != 0)
		{
			delay(2);
			if(TIM2->CCR3 < 100)
			{
				TIM2->CCR3 += 10;
			}
		}
		int __attribute((unused)) useless;
		useless = TIM2->CCR3;
	}
	if(TIM3->SR & TIM_SR_CC4IF)		//Blinds Close
	{
		while((GPIOB->IDR & (1 << 1)) != 0)
		{
			delay(2);
			if(TIM2->CCR3 > 0)
			{
				TIM2->CCR3 -= 10;
			}
		}
		int __attribute((unused)) useless;
		useless = TIM2->CCR3;
	}

	while((GPIOA->IDR & (1 << 6)) != 0);		//Wait for button to stop being pressed
	while((GPIOA->IDR & (1 << 7)) != 0);
	while((GPIOB->IDR & (1 << 0)) != 0);
	while((GPIOB->IDR & (1 << 1)) != 0);

	int __attribute((unused)) useless;
	useless = TIM3->CCR1;
	useless = TIM3->CCR2;
	useless = TIM3->CCR3;
	useless = TIM3->CCR4;
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	return;
}

void Button_OpenClose_Init(void)
{
	/*Previously used:
			PC6 to manually operate blinds up	(TIM3->CH1)
			PC7 to manually operate blinds down (TIM3->CH2)
			PC8 to manually operate blinds open (TIM3->CH3)
			PC9 to manually operate blinds close (TIM3->CH4)
		*/

		/*Currently used:
		 	 PA6 to manually operate blinds up (TIM3->CH1)
		 	 PA7 to manually operate blinds down (TIM3->CH2)
		 	 PB0 to manually operate blinds open (TIM3->CH3)
		 	 PB1 to manually operate blinds close (TIM3->CH4)


		 */

		//Timer
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

		GPIOB->MODER &= ~(3 << (2 * 0));	//Clear PB0
		GPIOB->MODER &= ~(3 << (2 * 1));	//Clear PB1

		GPIOB->MODER |= (2 << (2 * 0));		//Set PB0 to AF
		GPIOB->MODER |= (2 << (2 * 1));		//Set PB1 to AF

		GPIOB->AFR[0] &= ~(0xF << (4 * 0));	//Clear AFR[0]
		GPIOB->AFR[0] &= ~(0xF << (4 * 1));	//Clear AFR[1]

		GPIOB->AFR[0] |= (1 << (4 * 0));	//Set AFR[6] to AF1
		GPIOB->AFR[0] |= (1 << (4 * 1));	//Set AFR[7] to AF1

		GPIOB->PUPDR &= ~(3 << (2 * 0));	//Clear PUPDR 6
		GPIOB->PUPDR &= ~(3 << (2 * 1));	//Clear PUPDR 7

		GPIOB->PUPDR |= (2 << (2 * 0));		//Set Pull-Down
		GPIOB->PUPDR |= (2 << (2 * 1));		//Set Pull-Down

		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;	//Enable Timer3 in RCC

		TIM3->PSC = 1 - 1;
		TIM3->ARR = 0xFFFFFFFF;

		TIM3->CCMR2 &= ~TIM_CCMR2_CC3S;
		TIM3->CCMR2 &= ~TIM_CCMR2_CC4S;

		TIM3->CCMR2 |= TIM_CCMR2_CC3S_0;
		TIM3->CCMR2 |= TIM_CCMR2_CC4S_0;

		TIM3->CCER &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
		TIM3->CCER &= ~(TIM_CCER_CC4P | TIM_CCER_CC4NP);

		TIM3->CCER |= TIM_CCER_CC3E;
		TIM3->CCER |= TIM_CCER_CC4E;

		TIM3->DIER |= TIM_DIER_CC3IE;
		TIM3->DIER |= TIM_DIER_CC4IE;

		TIM3->CCMR2 |= TIM_CCMR2_IC3F_3 | TIM_CCMR2_IC3F_2 | TIM_CCMR2_IC3F_1 | TIM_CCMR2_IC3F_0;
		TIM3->CCMR2 |= TIM_CCMR2_IC4F_3 | TIM_CCMR2_IC4F_2 | TIM_CCMR2_IC4F_1 | TIM_CCMR2_IC4F_0;

}

void Button_UpDown_Init(void)
{
	/*Previously used:
		PC6 to manually operate blinds up	(TIM3->CH1)
		PC7 to manually operate blinds down (TIM3->CH2)
		PC8 to manually operate blinds open (TIM3->CH3)
		PC9 to manually operate blinds close (TIM3->CH4)
	*/

	/*Currently used:
	 	 PA6 to manually operate blinds up (TIM3->CH1)
	 	 PA7 to manually operate blinds down (TIM3->CH2)
	 	 PB0 to manually operate blinds open (TIM3->CH3)
	 	 PB1 to manually operate blinds close (TIM3->CH4)


	 */

	//Timer
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER &= ~(3 << (2 * 6));	//Clear PA6
	GPIOA->MODER &= ~(3 << (2 * 7));	//Clear PA7

	GPIOA->MODER |= (2 << (2 * 6));		//Set PA6 to AF
	GPIOA->MODER |= (2 << (2 * 7));		//Set PA7 to AF

	GPIOA->AFR[0] &= ~(0xF << (4 * 6));	//Clear AFR[6]
	GPIOA->AFR[0] &= ~(0xF << (4 * 7));	//Clear AFR[7]

	GPIOA->AFR[0] |= (1 << (4 * 6));	//Set AFR[6] to AF1
	GPIOA->AFR[0] |= (1 << (4 * 7));	//Set AFR[7] to AF1

	GPIOA->PUPDR &= ~(3 << (2 * 6));	//Clear PUPDR 6
	GPIOA->PUPDR &= ~(3 << (2 * 7));	//Clear PUPDR 7

	GPIOA->PUPDR |= (2 << (2 * 6));		//Set Pull-Down
	GPIOA->PUPDR |= (2 << (2 * 7));		//Set Pull-Down

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;	//Enable Timer3 in RCC

	TIM3->PSC = 1 - 1;
	TIM3->ARR = 0xFFFFFFFF;

	TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM3->CCMR1 &= ~TIM_CCMR1_CC2S;

	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;
	TIM3->CCMR1 |= TIM_CCMR1_CC2S_0;

	TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
	TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2NP);

	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER |= TIM_CCER_CC2E;

	TIM3->DIER |= TIM_DIER_CC1IE;
	TIM3->DIER |= TIM_DIER_CC2IE;

	TIM3->CR1 |= (2 << 8);	//Set clock div4

	NVIC->ISER[0] = 1 << TIM3_IRQn;

	TIM3->CCMR1 |= TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0;
	TIM3->CCMR1 |= TIM_CCMR1_IC2F_3 | TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_0;

}

void convert_time(struct time * t, float additional_time){
	int seconds = (int) additional_time;
	t->minute = t->minute + seconds/60;
	if (t->minute >= 60){
		t->minute = t->minute % 60;
		t->hour = t->hour + 1;
	}

}

int check_time(struct time time_cur, struct time comp_time){
	if (time_cur.hour >= comp_time.hour){
		if (time_cur.hour == comp_time.hour){
			if (time_cur.minute >= comp_time.minute){
				return 1;
			}
		}
	}
	return 0;
}


void OpenBlinds(void)
{
	  while(TIM2->CCR3 < 100)
	  {
		  delay(3);
		  TIM2->CCR3 += 10;
	  }
}

void CloseBlinds(void)
{
	  while(TIM2->CCR3 > 0)
	  {
		  delay(3);
		  TIM2->CCR3 -= 10;
	  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	current_state = START;
	message_index = 0;
	receive_flag = 0;
	message_len = 0;
	configured_flag = 0;
	curr_bat = -1;
	curr_pos = -1;
	curr_temp = -1;
	Bright_Outside = -1;
	time_cur.hour = -1;
	time_cur.minute = -1;
	close_time.hour = -1;
	close_time.minute = -1;
	open_time.hour = -1;
	open_time.minute = -1;
	bright_flag = -1;
	dark_flag = -1;
	done_config = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //If you want to reset ESP8266
  esp8266_reset(&huart1);


  if (esp8266_check_wifi_connection(&huart1)){
	  if (esp8266_setupTCP(&huart1)){
		  current_state = IDLE;
		  done_config = 1;
	  }else{
		  current_state = ERROR_STATE;
	  }
	  configured_flag = 1;
	  milli = HAL_GetTick();
  }
  else{
	  esp8266_reset(&huart1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  TIM2->CCR1 = 0;
	  TIM2->CCR2 = 0;

	  //Initiate system
	  while (current_state == START){
		  if (esp8266_init(&huart1)){
			  if (esp8266_setupTCP(&huart1)){
				  current_state = IDLE;
				  break;
			  }
		  }
		  current_state = ERROR_STATE;
	  }

	  //State where the system does nothing
	  while(current_state == IDLE){
		  HAL_UART_Receive_IT(&huart1, &message[message_index], 1);
		  curr_milli = HAL_GetTick();

		  if (receive_flag == 1){
			  current_state = RECEIVE;
			  receive_flag = 0;
		  }
		  else if (done_config){
			  //more than a minute has passed
			  if (curr_milli - milli > 60000){
				  //Update current time
				  if (time_cur.hour != -1){
					  convert_time(&time_cur, curr_milli - milli);
				  }
				  //Check if it is bright outside
				  Bright_Outside = Get_Luminosity(ADC_Read(4));
				  //Get current temperature
				  curr_temp = Convert_Fahrenheit(Get_Temperature(ADC_Read(10)));

				  //Automatic Slat and Slat open Configurations
				  //Time is highest priority
				  if (open_time.hour != -1 && close_time.hour != -1){
					  if (check_time(time_cur, open_time)){
						  //Open Blinds
						  OpenBlinds();
					  }
					  else if (check_time(time_cur, close_time)){
						  //Close Blinds
						  CloseBlinds();
					  }
				  }
				  else if (bright_flag != -1 && dark_flag != -1){
					  if (Bright_Outside && close_temp != -1 && open_temp != -1){
						  if (curr_temp >= open_temp){
							  //If it is warmer than opening temperature
							  //and if it is bright outside
							  OpenBlinds();
						  }
						  if (curr_temp >= close_temp){
							  //If is hotter than closing temperature
							  //and if it is bright outside
							  CloseBlinds();
						  }
					  }
					  else {
						  if (bright_flag && Bright_Outside){
							  //If BRIGHT outside...
							  //User configures it to OPEN when it is BRIGHT
							  OpenBlinds();
						  }
						  else if (bright_flag && Bright_Outside){
							  //If BRIGHT outside...
							  //User configures it to CLOSE when it is BRIGHT
							  CloseBlinds();
						  }
						  if (dark_flag && !Bright_Outside){
							  //If it DARK outside...
							  //User configures it to OPEN when it is DARK
							  OpenBlinds();
						  }
						  else if (dark_flag && !Bright_Outside){
							  //If it is DARK outside...
							  //User configures it to CLOSE when it is DARK
							  OpenBlinds();
						  }
					  }
				  }
				  milli = HAL_GetTick();
			  }
		  }
	  }

	  //State where the system connects to WiFi
	  while(current_state == CONNECT_WIFI){
		  HAL_UART_AbortReceive_IT(&huart1);
		  if (esp8266_sendmsg(&huart1, "OK\r\n", 4)){
			  if (esp8266_connectWifi(&huart1, wifi_name, wifi_password)){
				  if (esp8266_setupTCP(&huart1)){
					  current_state = IDLE;
					  done_config = 1;
					  milli = HAL_GetTick();
					  break;
				  }
			  }
		  }
		  current_state = ERROR_STATE;
	  }

	 //This is the state where you get luminosity, battery and temperature data
	 while (current_state == GET_CUR_DATA){
		 char message[25] ="REF";

		 //Read all necessary data
		 curr_temp = Convert_Fahrenheit(Get_Temperature(ADC_Read(10)));
		 curr_pos = TIM2->CCR3;
		 curr_bat = Get_Battery_Percentage(ADC_Read(11));

		 //Send data to android application
		 esp8266_send_current_data(&huart1, message, curr_temp, curr_bat, curr_pos, 3);
	 }


	  //This is the state where manual control is performed
	 while (current_state == MANUAL){
		HAL_UART_Receive_IT(&huart1, (uint8_t*) manual_control, 15);

		if (manual_control_flag){
		  if (strstr(manual_control, (char*) "SP") != NULL){
			  //TODO: Do necessary stuff to raise stop any manual action below

		  }
		  if (strstr(manual_control, (char*) "QT") != NULL){
			  //TODO: Do necessary stuff to quit out of manual control below
			  if (esp8266_sendmsg(&huart1, "K\n", 2)){
				  current_state = IDLE;
				  manual_control_flag = 0;
				  break;
			  }
		  }
		  if (strstr(manual_control, (char*) "UP") != NULL){
			  //TODO: Do necessary stuff to raise the blinds up below
		  }
		  if (strstr(manual_control, (char*) "DN") != NULL){
			  //TODO: Do necessary stuff to lower the blinds down below

		  }
		  if (strstr(manual_control, (char*) "PU") != NULL){
			  //TODO: Do necessary stuff to pitch up the blinds up below (controlling the slats)

		  }
		  if (strstr(manual_control, (char*) "CL") != NULL){
			  //TODO: Do necessary stuff to pitch down the blinds down below (controlling the slats)

		  }
		  HAL_UART_AbortReceive_IT(&huart1);
		  manual_control_flag = 0;
		}

		current_state = MANUAL;
	 }

	 //State where the system goes when in error
	 while (current_state == ERROR_STATE){
	 		  esp8266_reset(&huart1);
	 		  if (esp8266_check_wifi_connection(&huart1)){
	 			  if (esp8266_setupTCP(&huart1)){
	 				  current_state = IDLE;
	 				  done_config = 1;
	 			  }else{
	 				  current_state = ERROR_STATE;
	 			  }
	 			  configured_flag = 1;
	 			  milli = HAL_GetTick();
	 		  }
	 		  else{
	 			  done_config = 0;
	 			  current_state = START;
	 		  }
	 	 }

	  //State where the system is receiving message
	  while (current_state == RECEIVE){
		  HAL_UART_AbortReceive_IT(&huart1);
		  if (HAL_UART_Receive(&huart1, &message[message_index], 1, 100) == HAL_OK){
			  message_len++;
			  message_index++;
		  }
		  if (message[message_index-1] == '\n'){
			  current_state = IDLE;
			  manipulate_string((char *) message, message_len);

			  if (configured_flag == 0 && strstr((char*) message, (char *) "CONNECT") != NULL){
				  current_state = IDLE;
			  }
			  else if(wifi_credential_search((char*) message, wifi_name, wifi_password, message_len)){
				  current_state = CONNECT_WIFI;
			  }
			  else if (strstr((char*) message, (char *) "TEMP_CONFIG") != NULL){
				  //Acknowledge the message
				  esp8266_sendmsg(&huart1, "K\n", 2);
			  }
			  else if (strstr((char *) message, (char *) "TEMP_CLOSE") != NULL){
				  //Get integer value of closing temperature from message
				  close_temp = get_temperature((char *) message, message_len);
				  esp8266_sendmsg(&huart1, "CLOSE_OK\n", 9);
			  }
			  else if (strstr((char *) message, (char *) "TEMP_OPEN") != NULL){
				  //Get integer value of opening temperature from message
				  open_temp = get_temperature((char *) message, message_len);
				  esp8266_sendmsg(&huart1, "OPEN_OK\n", 9);
			  }
			  else if (strstr((char *) message, (char *) "LIGHT_CONFIG") != NULL){
				  //Acknowledge the message;
				  esp8266_sendmsg(&huart1, "K\n", 2);
			  }
			  else if (strstr((char *) message, (char *) "BRIGHT") != NULL){
				  //Check condition for when it is bright
				  //1 - OPEN
				  //0 - CLOSE
				  bright_flag = 0;
				  if (strstr((char*) message, (char *) "OPEN")){
					  bright_flag = 1;
				  }
		  		  esp8266_sendmsg(&huart1, "BRI_OK\n", 7);
			  }
			  else if (strstr((char *) message, (char *) "DARK") != NULL){
				  //Check condition for when it is dark
				  //1 - OPEN
				  //0 - CLOSE
				  dark_flag = 0;
				  if (strstr((char*) message, (char *) "OPEN")){
					  dark_flag = 1;
				  }
		  		  esp8266_sendmsg(&huart1, "DAR_OK\n", 7);
			  }
			  else if (strstr((char *) message, (char *) "TIME_CONFIG") != NULL){
				  esp8266_sendmsg(&huart1, "K\n", 2);
			  }
			  else if (strstr((char *) message, (char *) "TM_OP") != NULL){
				  //Extract Opening time from message
				  //open_text format: <hour>:<minute><AM or PM>
				  extract_time ((char*)message, message_len, time_open);
				  //Convert string time to time struct
				  if (time_open[5] == 'P'){
					  get_time_struct(&open_time, (char*) message,1);
				  }
				  else{
					  get_time_struct(&open_time, (char*) message,0);
				  }
		  		  esp8266_sendmsg(&huart1, "OP_OK\n", 6);
			  }
			  else if (strstr((char *) message, (char *) "TM_CL") != NULL){
				  //Extract Opening time from message
				  //open_text format: <hour>:<minute><AM or PM>
				  extract_time ((char*) message, message_len, time_close);
				  //Convert string time to time struct
				  if (time_close[5] == 'P'){
					  get_time_struct(&close_time, (char*) message,1);
				  }
				  else{
					  get_time_struct(&close_time, (char*) message,0);
				  }
		  		  esp8266_sendmsg(&huart1, "CL_OK\n", 6);
			  }

			  else if (strstr((char*) message, (char *) "MANUAL") != NULL){
				  esp8266_sendmsg(&huart1, "K\n", 2);
			  }

			  else if (strstr((char*) message, (char *) "CUR_TIME") != NULL){
				  //Extract military time from this function
				  extract_time ((char *) message, message_len, cur_time);
				  //Convert string time to time struct
				  get_time_struct(&time_cur, (char*) message,0);
				  current_state = GET_CUR_DATA;
			  }


			  HAL_UART_AbortReceive(&huart1);
			  memset(message, 0, message_len);
			  message_index = 0;
			  message_len = 0;
		  }
	  }
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
