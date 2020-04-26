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
char stap_ip[20] = "";
int configured_flag;
int close_temp;
int open_temp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	receive_flag = 1;
	message_index++;
	message_len++;

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
	staip_size = 0;

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
  //esp8266_reset(&huart1);


  if (esp8266_check_wifi_connection(&huart1)){
	  esp8266_setupTCP(&huart1);
	  current_state = IDLE;
	  configured_flag = 1;
  }
  else{
	  esp8266_reset(&huart1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
		  if (receive_flag == 1){
			  current_state = RECEIVE;
			  receive_flag = 0;
		  }
	  }

	  //State where the system connects to WiFi
	  while(current_state == CONNECT_WIFI){
		  HAL_UART_AbortReceive_IT(&huart1);
		  if (esp8266_sendmsg(&huart1, "OK\r\n", 4)){
			  if (esp8266_connectWifi(&huart1, wifi_name, wifi_password)){
				  if (esp8266_setupTCP(&huart1)){
					  current_state = IDLE;
					  break;
				  }
			  }
		  }
		  current_state = ERROR_STATE;
	  }

	  while (current_state == GET_CUR_TEMP){
		  char message[15] = "TEMP";

		  HAL_UART_AbortReceive_IT(&huart1);

		  //Do necessary stuff to get temperature bellow

		  //Send data
		  esp8266_send_current_data(&huart1, (char *) message, 60, 4);
		  current_state = IDLE;
	  }


	  while (current_state == GET_CUR_POS){
		  char message[15] = "POS";

		  HAL_UART_AbortReceive_IT(&huart1);

		  //Do necessary stuff to get blind position bellow

		  //Send data
		  esp8266_send_current_data(&huart1, (char *) message, 2, 3);
		  current_state = IDLE;
	  }

	  while (current_state == GET_CUR_BAT){
		  char message[15] = "BAT";

		  HAL_UART_AbortReceive_IT(&huart1);

		  //Do necessary stuff to get battery data bellow

		  //Send data
		  esp8266_send_current_data(&huart1, (char *) message, 87, 3);
		  current_state = IDLE;
	  }

	  while (current_state == TEMP_CONFIG){
		  HAL_UART_AbortReceive_IT(&huart1);

		  //Do any necessary stuff to initiate temperature configuration stuff

		  current_state = IDLE;
	  }

	  while (current_state == TEMP_CLOSE_CONFIG){
		  HAL_UART_AbortReceive_IT(&huart1);

		  //Do any necessary stuff to configure closing temperature stuff

		  //Acknowledge that you are done configuring closing temperature stuff
		  esp8266_sendmsg(&huart1, "CLOSE_OK\n", 9);

		  current_state = IDLE;
	  }

	  while (current_state == TEMP_OPEN_CONFIG){
		  HAL_UART_AbortReceive_IT(&huart1);

		  //Do any necessary stuff to configure opening temperature stuff

		  //Acknowledge that you are done configuring closing temperature stuff
		  esp8266_sendmsg(&huart1, "OPEN_OK\n", 8);

		  current_state = IDLE;
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
			  else if (strstr((char *) message, (char *) "RQ_TEMP")!= NULL){
				  current_state = GET_CUR_TEMP;
			  }

			  else if (strstr((char *) message, (char *) "RQ_POS")!= NULL){
				  current_state = GET_CUR_POS;
			  }

			  else if (strstr((char *) message, (char *) "RQ_BAT")!= NULL){
				  current_state = GET_CUR_BAT;
			  }
			  else if (strstr((char*) message, (char *) "TEMP_CONFIG") != NULL){
				  //First acknowledge the message
				  if (esp8266_sendmsg(&huart1, "K\n", 2)){
					  current_state = TEMP_CONFIG;
				  }
			  }
			  else if (strstr((char *) message, (char *) "TEMP_CLOSE") != NULL){
				  current_state = TEMP_CLOSE_CONFIG;
				  close_temp = get_temperature((char *) message, message_len);
			  }
			  else if (strstr((char *) message, (char *) "TEMP_OPEN") != NULL){
				  current_state = TEMP_OPEN_CONFIG;
				  open_temp = get_temperature((char *) message, message_len);

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
