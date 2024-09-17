/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define Ts 20 //Sampling Period(ms) -> Ts = 1/(Fs*1000) = 20ms
#define G 0.0003053 //Constant to be multiplied with gyro readings -> G = 1/Fs/G_R
#define A_R 4096.0 //Accelerometer scale factor. Set during initialization. Consult datasheet section 4.4 for more info.
#define G_R 65.5 //Gyroscope scale factor. Set during initialization. Consult datasheet section 4.5 for more info.
#define M_PI   3.14159265358979323846264338327950288

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz,t;
uint16_t maxPwm =100;
uint8_t minPwm = 5;

char buffer[32]={0};
uint8_t count = 0;

float setpoint = 90.00;
float error = 0;
float lastError = 0;

float Kp = 1.0;
float Ki = 1.2;
float Kd = 1.3;

float P;
float I;
float D;
float pitch, roll;
float offset_gyroX;
float offset_gyroY;
float offset_acc_pitch;
float offset_acc_roll;
uint32_t I2C_TIMEOUT = 50;

uint32_t get_ms_tick(void) {
	return HAL_GetTick();
}

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
		//
		pitch=0;
		   roll=0;
	   offset_gyroX=0;
	    offset_gyroY=0;
     offset_acc_pitch=0;
     offset_acc_roll=0;
	}
}
void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}


void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
}

void mpu6050_update() {
	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();
}

void mpu6050_calc_pitch_roll(float *pitch, float *roll) {
	//bool that keeps track of whether or not this is the first iteration
	int first_run = 1;

	//total pitch/roll calculations
	static float total_pitch = 0;
	static float total_roll = 0;

	//get gyro's current x/y values, and subtract an offset
	float gyro_x = Gx- offset_gyroX;
	float gyro_y = Gy - offset_gyroY;

	//integrate gyro rates into the total pitch/roll calculation. G = 1/G_R/Fs
	total_pitch += gyro_x * (G);
	total_roll += gyro_y * (G);

	//calculate pitch/roll using accelerometer readings alone, minus an offset
	float acc_pitch = (180 * atan2(Ay / A_R, Az / A_R)
			/ M_PI) - (offset_acc_pitch) + 90;
	float acc_roll = (180 * atan2(Ax / A_R, Az / A_R)
			/ M_PI) - (offset_acc_roll);

	//if this is the first iteration, simply set pitch/roll
	//to accelerometer's pitch/roll
	if (first_run) {
		total_pitch = acc_pitch;
		total_roll = acc_roll;

		first_run = 0;
	}
	//else apply complimentary filter using gyro/accelerometer pitch/roll
	//calculations
	else {
		total_pitch = total_pitch * .8 + acc_pitch * .2;
		total_roll = total_roll * .8 + acc_roll * .2;
	}

	//dereference pitch and roll pointers, set literal value to calculated
	//pitch and roll angles
	*pitch = total_pitch;
	*roll = total_roll;
}
void LeftW(int Forward, int Back){
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Forward);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,Back);
}
void RightW(int Forward, int Back){
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Forward);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Back);
}

void mpu6050_calc_acc_pitch_roll( float *pitch, float *roll) {
	float acc_x, acc_y, acc_z;
	float acc_pitch, acc_roll;

	//normalized accelerometer readings. Constructed by taking
	//raw accelerometer readings and diving by accelerometer scaling
	//factor.
	acc_x = Ax / A_R;
	acc_y = Ay / A_R;
	acc_z = Az / A_R;

	//calculated pitch and roll angles
	acc_pitch = 180 * atan2(acc_y, acc_z) / M_PI;
	acc_roll = 180 * atan2(acc_x, acc_z) / M_PI;

	//dereference pitch and roll pointers, set literal value to calculated
	//pitch and roll angles
	*pitch = acc_pitch;
	*roll = acc_roll;
}

void mpu6050_calibrate() {
	//number of samples to take
	int num_samples = 50;

	//local variables used for calculating offsets.
	float offset_gyroX_test = 0;
	float offset_gyroY_test  = 0;
	float offset_accPitch_test  = 0;
	float offset_accRoll_test  = 0;

	uint32_t now = get_ms_tick();
	for (int i = 0; i < num_samples; i++) {
		//variables that will hold accelerometer pitch/roll values
		//for each iteration.
		float acc_pitch_test  = 0;
		float acc_roll_test  = 0;

		//update sensor readings
		mpu6050_update();
		//calculate pitch/roll using only accelerometer
		mpu6050_calc_acc_pitch_roll( &acc_pitch_test , &acc_roll_test);

		//increment offset values using calculated values
		offset_accPitch_test  += acc_pitch_test ;
		offset_accRoll_test  += acc_roll_test ;
		offset_gyroX_test  += Gx;
		offset_gyroY_test  += Gy;

		//wait till sampling period has elapsed, and repeat
		while (get_ms_tick() - now < Ts) {
		}
		now = get_ms_tick();
	}

	//take the average of the total readings. Store that value
	//in the mpu6050 typedef
	offset_gyroX = offset_gyroX / (float) num_samples;
	offset_gyroY = offset_gyroY / (float) num_samples;
	offset_acc_pitch = offset_accPitch_test / (float) num_samples;
	offset_acc_roll = offset_accRoll_test / (float) num_samples;
}
void uprintf(char *str)
{
		HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

void PID() {
	//update IMU, and calculate pitch/roll angles
	mpu6050_update();
	//mpu6050_calc_pitch_roll(&mpu6050, &pitch, &roll);
	mpu6050_calc_pitch_roll( &pitch,&roll);
	/********************* PID CALCULATION *************************/

	//calculate error
	error = setpoint - pitch;

	//calculate Proportional term
	P = Kp * error;

	//calculate Integral term. Account for wind-up
	I += (Ki * error);
	if (I > maxPwm)
		I = maxPwm;
	else if (I < minPwm)
		I = minPwm;

	////calculate Derivative term
	D = -Kd * (error - lastError);

	//total PID value
	float pid_pwm = P + I + D;

	//max sure pwm is bound between allowed min/max thresholds
	uint8_t out_pwm;
	pid_pwm *= 10;
	if (pid_pwm > maxPwm)
		out_pwm = maxPwm;
	else if (pid_pwm < minPwm)
		out_pwm = minPwm;
	else
		out_pwm = (uint8_t) pid_pwm;

	lastError = error;
	
	/******************** SET MOTOR PWM & DIRECTION *********************/
	//out_pwm *= 10;
	if (pid_pwm > 350) pid_pwm = 350;
	else if (pid_pwm <250) pid_pwm = 250;
	sprintf(buffer, "%f pid_pwm: %f\n", error,pid_pwm);
	uprintf(buffer);
	if (error >0)
	{
		LeftW(300,0);
		RightW(300,0);
	}
	else
	{
		LeftW(0,300);
		RightW(0,300);
	}
	//sprintf(buffer, "Error: %f\n, out_pwm: %d\n",error,out_pwm);
	//uprintf(buffer);
	//Set H-Bridge direction 
	//doi chieu quay ( can xai thi xai)
	/*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,
			(error > 0.0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,
			(error > 0.0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,
			(error > 0.0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,
			(error > 0.0) ? GPIO_PIN_RESET : GPIO_PIN_SET);*/

	//update PWM to H-Bridge. PWM controlled by TIMER1
	// chon toc do quay
	/*htim1.Instance->CCR1 = out_pwm;
	htim1.Instance->CCR4 = out_pwm;*/

	/*******************************************************************/

}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//lcd_init();
		
		
	MPU6050_Init();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		PID();
		HAL_Delay(80);
		
	/*	MPU6050_Read_Accel();
	  MPU6050_Read_Gyro();
		 HAL_Delay(200);*/
		 
		 
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 899;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 899;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
