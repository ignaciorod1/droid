
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t spiTx[3], spiRx[6];
uint8_t modeReg;
uint8_t i2cRx[8], i2cBuff[7], i2cBuffm[7];
int i = 0;
float acc[3], mag[3], gyro[3], acc_deg[3] = {0,0,0}, gyro_deg[3] = {0,0,0}, deg[3] = {0,0,0};
float offsetMag[3];
float total_acc;
float Ka, Kg;	// gains of the complementary filter

uint8_t mg[9];

struct Stepper {
	GPIO_TypeDef *dir_letter;
	uint16_t dir_number;
	GPIO_PinState dir;
	uint32_t uStepping;	//this controls the pins of microstepping  for the A4988 resolution
	uint32_t vel;		//velocity in steps	
	TIM_TypeDef *timer;	//the timer attached 
}left, right;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void setVel(struct Stepper *stepper, uint32_t vel){
		stepper->timer->ARR = vel;
		stepper->timer->CCR1 = vel/2;
}

void setDir(struct Stepper *stepper, int dir){
	
	if(dir == 1)	{
		stepper->dir = GPIO_PIN_SET;
	}
	else if (dir == 0){ 
		stepper->dir = GPIO_PIN_RESET;
	}

	HAL_GPIO_WritePin(stepper->dir_letter, stepper->dir_number, stepper->dir);
}

void turnOnGyro(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

void turnOffGyro(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void getGyro(){
	uint16_t data[3];
		
  // send data
	turnOnGyro();	
	spiTx[0] = GYRO_AXES_DATA_REG | 0xC0;
	HAL_SPI_Transmit(&hspi1, spiTx, 1, 10); 
	// read values
	HAL_SPI_Receive(&hspi1, spiRx, 6, 10);
	turnOffGyro();
	HAL_Delay(10);
	
	data[0] = (spiRx[1] << 8) | spiRx[0];
	data[1] = (spiRx[3] << 8) | spiRx[2];
	data[2] = (spiRx[5] << 8) | spiRx[4];
	
	
	for(int i = 0; i < 3; i++){	//C2 data processing				
		if(data[i] >= 0x8000){
			gyro[i] = - ( data[i] ^ 0xFFFF) + 1;
		}
		else	gyro[i] =  data[i];
		
		gyro[i] = gyro[i] * GYRO_SENS;
	}
}

// transmit data = 0x32
// receive data = 0x33
void testAcc(){
		if(HAL_I2C_IsDeviceReady(&hi2c1, 0x33, 1, 10) == HAL_OK)
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		
		
		if(HAL_I2C_IsDeviceReady(&hi2c1, MAG_ADD_READ, 1, 10) == HAL_OK)
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		
		
		if(HAL_I2C_IsDeviceReady(&hi2c1, 0x54, 1, 10) == HAL_OK)
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		

}


void getAcc(){
	
	i2cBuff[0] = 0x28 | 0x80;	//sending the register address
	HAL_I2C_Master_Transmit(&hi2c1, 0x33, i2cBuff, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, 0x32, &i2cBuff[1], 6, 10);
	
	int16_t data[3];
	data[0] = (( i2cBuff[2] << 8 ) | i2cBuff[1]) >> 4;
	data[1] = (( i2cBuff[4] << 8 ) | i2cBuff[3]) >> 4;
	data[2] = (( i2cBuff[6] << 8 ) | i2cBuff[5]) >> 4;
	
	for(int i = 0; i < 3; i++){
		if ( data[i] > 0x7FF){	//	negative number
			acc[i] = - (data[i] ^ 0xFFF) + 1;
		}
		else	acc[i] = data[i];
		
		acc[i] /= 1024;
		
		total_acc = sqrt(acc[0] *acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
	}	

}

void getMagnetometer(){
	
	i2cBuffm[0] = 0x03;	//sending the register address
	HAL_I2C_Master_Transmit(&hi2c1, MAG_ADD_WRITE, i2cBuffm, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, MAG_ADD_READ, &i2cBuffm[1], 6, 10);
	
	HAL_I2C_Master_Transmit(&hi2c1, 0x54, i2cBuffm, 7, 10);

	int16_t data[3];
	data[0] = ( i2cBuffm[1] << 8 ) | i2cBuffm[2];
	data[1] = ( i2cBuffm[3] << 8 ) | i2cBuffm[4];
	data[2] = ( i2cBuffm[5] << 8 ) | i2cBuffm[6];
	
	for(int i = 0; i < 3; i++){
		if ( data[i] > 0x7FF){	//	negative number
			mag[i] = - (data[i] ^ 0xFFF) + 1;
		}
		else	mag[i] = data[i];
	}	
		
		mag[0] /= MAG_SENS_XY;
		mag[1] /= MAG_SENS_XY;
		mag[2] /= MAG_SENS_Z;
}


void calib_magnetometer(){
	for(int i = 0; i < 3; i++){
		mag[i] = offsetMag[i];
	}
}


void gyro_int_values(){
	for( int i = 0; i < 3; i++){
		gyro_deg[i] += gyro[i] / (float)(1.0/95.0);
	}
}

void acc2angle(){

	acc_deg[0] = asin(acc[1] / total_acc) * 57.2958;		// ROLL
	acc_deg[1] = asin(-acc[0] /total_acc) * 57.2958;		// PITCH
}	

void angles(){
	
	for(int i = 0; i < 2; i++){
		deg[i] = Kg*(deg[i] + gyro_deg[i] * (1/95)) + Ka*acc_deg[i]; 
	}
}
	
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	left.dir_letter = GPIOD;
	left.dir_number = GPIO_PIN_8;
	left.timer = TIM1;
	left.uStepping = 0;
	left.vel = 0;
	left.dir = GPIO_PIN_SET;

	right.dir_letter = GPIOD;
	right.dir_number = GPIO_PIN_10;
	right.timer = TIM4;
	right.uStepping = 0;
	right.vel = 0;
	right.dir = GPIO_PIN_SET;
	
	Kg = 0.95;
	Ka = 1 - Kg;
	
	offsetMag[0] = -0.085;
	offsetMag[1] = -0.065;
	offsetMag[2] = -0.325;
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_1);  
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_Delay(1000);
	
	setVel(&left, 300 );
	setDir(&left, 1);
	
	setVel(&right, 500);
  setDir(&right, 0);
	
	spiTx[0] = 0x20;
	spiTx[1] = 0x0F;		//  95 Hz of output data rate and 12.5 Hz cutoff low freq
	
	turnOnGyro();
	HAL_SPI_Transmit(&hspi1, spiTx, 2, 50);	
  turnOffGyro();
	
	turnOnGyro();	
	// adding 1 as the MSB to enable read
	spiTx[0] = GYRO_CTRL_REG1 | 0x80;
	// transmiting the new data to the register
	HAL_SPI_Transmit(&hspi1, spiTx, 1, 50);
	HAL_SPI_Receive(&hspi1, spiRx, 1, 50);
	turnOffGyro();
	
	
	spiTx[0] = GYRO_SENS_REG;
	spiTx[1] = GYRO_SENS_FS;
	turnOnGyro();
	HAL_SPI_Transmit(&hspi1, spiTx, 2, 20);
	turnOffGyro();
	
	// i2c test
	testAcc();
	
	
	//  ********************* ACCELEROMETER ***********************
	
	// WRITE DATA
	i2cBuff[0] = 0x20 | 0x80;	// register 0. the |0x80 puts a '1' as the MSB and therefore enables multiple byte reading.
	i2cBuff[1] = 0x47;	// data to put in the register
	HAL_I2C_Master_Transmit(&hi2c1, 0x33, i2cBuff, 2, 10);	
	HAL_Delay(50);
	
	i2cBuff[0] = 0x23 | 0x80;	// register 0. the |0x80 puts a '1' as the MSB and therefore enables multiple byte reading.
	i2cBuff[1] = 0x0;	// data to put in the register
	HAL_I2C_Master_Transmit(&hi2c1, 0x33, i2cBuff, 2, 10);	
	HAL_Delay(50);

	// READ DATA
	i2cBuff[0] = 0x20;
	HAL_I2C_Master_Transmit(&hi2c1, 0x33, i2cBuff, 1, 10);
	i2cBuff[1] = 0x00;	// empty the variable
	HAL_I2C_Master_Receive(&hi2c1, 0x32,  &i2cRx[1], 1, 10);
	
	// *********************** MAGNETOMETER ************************
	
	i2cBuff[0] = 0;
	i2cBuff[1] = 0x14;
	HAL_I2C_Master_Transmit(&hi2c1, MAG_ADD_WRITE, i2cBuff, 2, 10);
	HAL_Delay(50);
	
	i2cBuff[0] = 0x01;
	i2cBuff[1] = 0x30;
	HAL_I2C_Master_Transmit(&hi2c1, MAG_ADD_WRITE, i2cBuff, 2, 10);
	HAL_Delay(50);
	
	i2cBuff[0] = 0x02;
	i2cBuff[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, MAG_ADD_WRITE, i2cBuff, 2, 10);
	HAL_Delay(50);
	
	i2cBuff[0] = 0x0A;
	i2cRx[2] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, MAG_ADD_WRITE, i2cBuff, 2, 10);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, MAG_ADD_READ, &i2cRx[2], 3, 10);
	
	// *********************** ARDUINO *****************
	
	i2cBuff[0] = 0x0F;
	i2cBuff[1] = 0x14;
	HAL_I2C_Master_Transmit(&hi2c1, 0, i2cBuff, 1, 10);
	HAL_Delay(50);

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	getGyro();
	getAcc();
  getMagnetometer();
  gyro_int_values();
	acc2angle();
	angles();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIR_2_Pin|DIR_Pin|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_2_Pin DIR_Pin PD13 PD14 
                           PD15 */
  GPIO_InitStruct.Pin = DIR_2_Pin|DIR_Pin|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
