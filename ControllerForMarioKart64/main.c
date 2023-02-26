/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "iks01a3_motion_sensors.h"
#include "math.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Initialization of global flag to handle blue push-button through interrupt
volatile uint8_t button_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void getCleanData(int cleanData[], int32_t xAxisReading[], int32_t yAxisReading[], int32_t zAxisReading[], int32_t xAxisReadingGyro[]);
void dataShiftRight(int32_t xAxisReading[],int32_t yAxisReading[],int32_t zAxisReading[], int32_t xAxisReadingGyro[]);
float getXYorientation(int32_t cleanData[]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	// Vectors to store data from sensor and filter them
	int32_t xAxisReading[6] = {0};
	int32_t yAxisReading[6] = {0};
	int32_t zAxisReading[6] = {0};
    int32_t xAxisReadingGyro[6] = {0};


	// Flag activated when game is in pause
	uint8_t pause_flag = 0;

	// Flag activated to avoid conflicting commands
	uint8_t jump_flag = 0;

	// Vector storing X,Y,Z acceleration and X angular rate filtered data
	int cleanData[4] = {0};


	// Variable to store the XY orientation of our board
	float orientation;

	// Series of variables to send commands to python script
	char command[5] = {0};
	char special[5] = {0};
	char jump[5] = {0};
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize and enable the accelerometer in X-MEMS
  IKS01A3_MOTION_SENSOR_Init(1,MOTION_ACCELERO);
  IKS01A3_MOTION_SENSOR_Enable(1,MOTION_ACCELERO);
  // Initialize and enable the gyroscope in X-MEMS
  IKS01A3_MOTION_SENSOR_Init(0,MOTION_GYRO);
  IKS01A3_MOTION_SENSOR_Enable(0,MOTION_GYRO);

  // Initialize these two strings as they will not be modified inside the main
  strcpy(jump, "e");
  strcpy(special,"r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   // Acquisition and filtering of sensor data stored in cleanData
	   getCleanData(cleanData, xAxisReading, yAxisReading, zAxisReading, xAxisReadingGyro);


	   // Computation of orientation on XY plane of out board
	   orientation = getXYorientation(cleanData);

	   // Neutral value, this will not lead to any action in the game (unmodified only in case of malfunctioning)
	   strcpy(command,"0");

	   /* COMMANDS COMPUTATION */

	     // In case the board is laying on the table and program is not yet in pause, pause command is sent
	  	 if (cleanData[2] > 970 && pause_flag == 0){
	  		strcpy(command,"p");
	  		pause_flag = 1;
	  	 }

	  	 // In case game is in pause and board is no longer on the table, game is restarted
	  	 if (cleanData[2] < 900  && pause_flag == 1){
	  		 strcpy(command,"p");
	  		 pause_flag = 0;
	  	 }

	  	 // If while playing board is turned fast about x axis toward player's chest, jump command is instantly sent
	  	 if (cleanData[3] > 100000 && pause_flag == 0){
	  		 jump_flag = 1;
	  		 HAL_UART_Transmit_IT(&huart2,jump,strlen(jump));
	  		 HAL_Delay(50);
	  	 }

	  	 // If while playing blue button is pressed, command to use objects is immediately sent
	  	 if (button_flag == 1 && pause_flag == 0){
	  		 button_flag = 0;
	  		 HAL_UART_Transmit_IT(&huart2,special,strlen(special));
	  		 HAL_Delay(50);
	  	 }

	  	 // If while playing board is turned left, command to turn left is sent.
	  	 // If the orientation is between [-10,-40] degrees will be send in python the command
	  	 // to steer softly, instead if it is grater then -40 degrees it will be send the command
	  	 // to steer hardly
	  	 /* (If jump command is sent, the orientation is such that also an
	  	  * unwanted turn left command is sent, jump_flag avoids this*/
	  	 if (orientation<=-10 && pause_flag == 0 && jump_flag == 0){
	  		 if(orientation>=-40 && orientation<-10){
	  			strcpy(command,"l");
	  		 }
	  		 else if (orientation<-40) {
	  		   strcpy(command,"f");
	  		 }
	  	 }

	  	 // If while playing board is turned right, command to turn right is sent.
	  	 // If the orientation is between [10,40] degrees will be send in python the command
	  	 // to steer softly, instead if it is grater then 40 degrees it will be send the command
	  	 // to steer hardly
	  	 else if(orientation>=10 && pause_flag == 0){
	  		 if (orientation>10 && orientation<=40){
	  			strcpy(command,"k");
	  		 }
	  		 else if (orientation>40) {
	  		   strcpy(command,"h");
	  		 }
	  	 }

	  	 // If while playing board is not turned, command to accelerate is sent
	  	 else if(orientation < 10 && orientation > -10 && pause_flag == 0){
	  		 strcpy(command,"z");
	  	 }


	  	 // Sending commands through serial port
		 HAL_UART_Transmit_IT(&huart2,command,strlen(command));

		 // Flag is reset
		 jump_flag = 0;

		 HAL_Delay(50);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void getCleanData(int cleanData[], int32_t xAxisReading[], int32_t yAxisReading[], int32_t zAxisReading[], int32_t xAxisReadingGyro[]){
	/*Implementation of Moving average filter */

	// Initialization of the struct to store sensor measurements
	IKS01A3_MOTION_SENSOR_Axes_t axes;
	IKS01A3_MOTION_SENSOR_Axes_t axesGyro;

	// Initialization of check of good acquisition
	int8_t returnStatus = 0;
	int8_t returnStatusGyro = 0;

	 // Start of data acquisition and filtering
	 // Accelerometer measures in mg (1000 mg = 9.81 m/s^2), Gyroscope measures in millidegree/s


	// Reading from sensors using IKS01A3 library
	 returnStatus = IKS01A3_MOTION_SENSOR_GetAxes(1,MOTION_ACCELERO,&axes);	// I am acquiring data from the accelerometer in 3 directions
	 returnStatusGyro = IKS01A3_MOTION_SENSOR_GetAxes(0,MOTION_GYRO,&axesGyro);// I am acquiring data from the gyroscope in 3 directions

	 // An update of previous measures is performed only if both sensor readings were successful
	 if(returnStatus == BSP_ERROR_NONE && returnStatusGyro == BSP_ERROR_NONE){

		  // Acquisition of new measurement
		  xAxisReading[0] = axes.x;
		  yAxisReading[0] = axes.y;
		  zAxisReading[0] = axes.z;
		  xAxisReadingGyro[0] = axesGyro.x;

		 // Implementation of the recursive moving average filter (window of 5 points)
		  cleanData[0] = cleanData[0] - xAxisReading[5]/5 + xAxisReading[0]/5;
		  cleanData[1] = cleanData[1] - yAxisReading[5]/5 + yAxisReading[0]/5;
		  cleanData[2] = cleanData[2] - zAxisReading[5]/5 + zAxisReading[0]/5;
		  cleanData[3] = cleanData[3] - xAxisReadingGyro[5]/5 + xAxisReadingGyro[0]/5;

		  // The old value is cancelled and the new value is now ready to be acquired
		  dataShiftRight(xAxisReading, yAxisReading, zAxisReading, xAxisReadingGyro);

	    }
}


void dataShiftRight(int32_t xAxisReading[],int32_t yAxisReading[],int32_t zAxisReading[], int32_t xAxisReadingGyro[]){
	/* This function will perform the right shifting of data contained inside four unidimensional arrays
	 * of dimension 6. The shifting will not involve the first element which is left unmodified	 */

	// Counter initialization
	int8_t i;

	// Actual data shifting
	for(i=5; i>0; i--){
		xAxisReading[i] = xAxisReading[i-1];
		yAxisReading[i] = yAxisReading[i-1];
		zAxisReading[i] = zAxisReading[i-1];
		xAxisReadingGyro[i] = xAxisReadingGyro[i-1];
	}
}

float getXYorientation(int32_t cleanData[]){
	/* This function computes the XY orientation of the board starting from
	 * Acceleration filtered data of X axis and Y axis: 0° corresponds to board positioned
	 * horizontally with MEMS board on the left side when facing someone's chest 	 */

	// Output variable initialization
	 float angle;

	 // Orientation computaiton in radiants
	 angle=atan2(cleanData[1],cleanData[0]);

	 // Orientation conversion in degrees
	 angle=angle*180/3.1415;

	 return angle;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Interrupt created to handle blue push-button (B1_Pin), in case it is pressed
	 * the global flag is set to 1 and an action will be initiated in the main*/

	if(GPIO_Pin == B1_Pin)
	{
		button_flag = 1;
	}
	else{
		__NOP ();
	}
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
