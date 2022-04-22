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
#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{

	double pre_input;
	double pre_output;
	double pre_time;
	double Wc;


} LowPass;

typedef struct{

	double kp,ki,kd;
	double tau;		// derivative low-pass filter time constant
	double max,min;	// output limit
	double dt;		// sample time
	double integral,pre_error,diff,pre_mea;

} PID;

// UART protocol
typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail; //RXHeadUseDMA

} UARTStucrture;

UARTStucrture UART2 =
{ 0 };

typedef enum{
	state_idle,
	state_start,
	state_mode,
	state_n_station,
	state_data_frame,
	state_check_sum,
	state_wait_for_ack1_1,
	state_wait_for_ack1_2,
}uart_state;

int inputchar = -1;


typedef enum{
	state_move_idle,
	state_tar_plan,
	state_wait_des,
	state_wait_5sec,
	state_check_left_stations
}moving_state;





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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint64_t _micro = 0;


int encoder_value = 0;
double encoder_velocity = 0,encoder_velocity_rpm = 0;
uint64_t time_stamp = 0,time_stamp2 = 0,time_stamp_5sec = 0;


LowPass lowpass_filters[10] = {0};
double Wc_arr[15] = {0.00001,0.00002,0.00003,0.00004,0.00005,0.00006,0.00007,0.00008,0.00009,0.0001} ;
double lowpass_output[15] = {0};
double kalman_output = 0;

PID pids[2] = {0};
double pid_pwm_output = 0,setpoint = 0;

double paths[1000] = {0};
int paths_ind = 0,path_n_cnt = 0;



int station_ind = 0;
int stations_postion[10] = {0,45,180,35,270,70,150,200,30,300};
uint8_t mcu_connect = 0,goals[512] = {0},go_now = 0,current_station = 0,enable_endeffector = 0,enable_sethome = 0;
uint16_t n_goal = 0;
double max_velocity = 0,set_position = 0,current_position = 1.5634;


moving_state move_state = state_move_idle;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

uint64_t micros();

int unwraping_update();
double velocity_update(int cur_pos);
double low_pass_process(LowPass *lowpass,double input);
double kalman_filter_update(double U);
double pid_update(PID *pid,double setpoint,double mea);
double ppms_to_rpm(double input);
void targectory_cal(double *datas,int *n,int start_pos,int stop_pos,double dt);
void encoder_lowpass_update();
void uart_update();
void moving_state_update();
void end_effector_update();



//UART protocol
void UARTInit(UARTStucrture *uart);

void UARTResetStart(UARTStucrture *uart);

uint32_t UARTGetRxHead(UARTStucrture *uart);

int16_t UARTReadChar(UARTStucrture *uart);

void UARTTxDumpBuffer(UARTStucrture *uart);

void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);

void uart_protocal(int16_t input,UARTStucrture *uart);


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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // start micro
  HAL_TIM_Base_Start_IT(&htim5);

  // start PWM
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);


  // start QEI
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);


  //init lowpass_filter
  for(int i = 0;i < 10;i++){
	  lowpass_filters[i].Wc = Wc_arr[i];
  }

  //init pid
  pids[0].dt = pids[1].dt = 0.02;
  pids[0].min = pids[1].min = -10000;
  pids[0].max = pids[1].max = 10000;
  pids[0].tau = pids[1].tau = 0.02;


  pids[0].kp = 500;
  pids[0].ki = 600;
  pids[0].kd = 10;




  //PID position control
  pids[1].kp = 500;
  pids[1].ki = 1000;
  pids[1].kd = 1;



  //UART protocol
  UART2.huart = &huart2;
  UART2.RxLen = 255;
  UART2.TxLen = 255;
  UARTInit(&UART2);
  UARTResetStart(&UART2);


  enable_endeffector = 1;

  targectory_cal(paths, &path_n_cnt, 0, 180, 0.02);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  moving_state_update();

	  encoder_lowpass_update();

	  end_effector_update();

	  //pid control , system dead-time = 0.16 sec = 6.25 Hz 165000
	  if(micros() - time_stamp2 > 20000){ // 6.06Hz
		  time_stamp2 = micros();


		  setpoint = paths_ind < path_n_cnt ? paths[paths_ind++]/6:0;
		  pid_pwm_output = pid_update(&pids[0], setpoint, encoder_velocity_rpm);

		  /*
		  if(go_now == 1 && abs((int)(setpoint - (double)encoder_value/(12*64*4-1)*360)) < 3){
		  	  setpoint = stations_postion[goals[station_ind++]];
			  if(station_ind >= n_goal){go_now = station_ind = 0;}
			}
			pid_pwm_output = pid_update(&pids[1], setpoint, (double)encoder_value/(12*64*4-1)*360);
		   */


		  if(pid_pwm_output > 0){
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pid_pwm_output);
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
		  }
		  else{
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,-pid_pwm_output);
		  }
	  }


	  uart_update();



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = (12*64*4) -1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

}

/* USER CODE BEGIN 4 */

static int pre_enc_cnt = 0;
static int k = 0;

#define half_enc_cnt (12*64*4)/2
#define enc_cnt 12*64*4
int unwraping_update(){

	static int cur_enc_cnt = 0;

	cur_enc_cnt = TIM1->CNT;


	if(abs((int)cur_enc_cnt - pre_enc_cnt) > half_enc_cnt){
		k+= (int)cur_enc_cnt - pre_enc_cnt > 0 ? -1 : 1 ;
	}

	pre_enc_cnt = cur_enc_cnt;
	return enc_cnt * k + cur_enc_cnt;
}

double velocity_update(int cur_pos){
	static int pre_pos = 0;
	static int pre_time = 0;
	static int cur_time = 0;

	cur_time = micros();

	double velo = (double)(cur_pos - pre_pos)/(cur_time - pre_time);

	pre_pos = cur_pos;
	pre_time = cur_time;

	return velo;

}

double ppms_to_rpm(double input){
	return input*60e6/(enc_cnt);
}

double low_pass_process(LowPass *lowpass,double input){

	double cur_time = micros();
	double delta_t = cur_time - lowpass->pre_time;
	double k = (lowpass->Wc*delta_t)/(2+ lowpass->Wc*delta_t);

	double output = (1 - 2*k)*(lowpass->pre_output) +k*(input+lowpass->pre_input);

	lowpass->pre_time = cur_time;
	lowpass->pre_input = input;
	lowpass->pre_output = output;

	return output;
}

static double R = 40; //noise covariance
static double H = 1; //measurement map scalar
static double Q = 10; //initial estimate covariance
static double P = 0 ; //initial error covariance
static double U_hat = 0; //initial estimate state
static double K = 0; //initial Kalman gain

double kalman_filter_update(double U){
	//being
	K = P*H/(H*P*H+R); //updata kalman gain
	U_hat = U_hat + (U-H*U_hat); //update estimated

	//update error covariance
	P = (1-K*H)*P+Q;

	return U_hat;
}

double pid_update(PID *pid,double setpoint,double mea){

	//Error
	double error = setpoint - mea;

	//Proportional
	double proportional = pid->kp * error;

	//Integral
	pid->integral = pid->integral + 0.5f * pid->ki * pid->dt * (error + pid->pre_error);

	//calculate integral anti wind up
	double max_i,min_i;

	max_i = pid->max > proportional ? pid->max - proportional : 0;
	min_i = pid->min < proportional ? pid->min - proportional : 0;


	//integral clamp
	if(pid->integral > max_i){
		pid->integral = max_i;
	}
	else if(pid->integral < min_i){
		pid->integral = min_i;
	}

	//Derivative
	pid->diff = -(2.0f * pid->kd * (mea - pid->pre_mea) + (2.0f * pid->tau - pid->dt) * pid->diff) / (2.0f * pid->tau + pid->dt);

	double output = proportional + pid->integral + pid->diff;

	if(output > pid->max){
		output = pid->max;
	}
	else if(output < pid->min){
		output = pid->min;
	}

	pid->pre_error = error;
	pid->pre_mea = mea;

	return output;

}


void targectory_cal(double *datas,int *n,int start_pos,int stop_pos,double dt){

    double v_max = 10*6;            // degree per sec
    double a_max = 0.5 * 57.296;    // degree per sec^2

    int dis = (stop_pos - start_pos +360)%360;
    int inverse = 0;
    if ((start_pos - stop_pos +360)%360 < dis){
        inverse = 1;
        dis = (start_pos - stop_pos +360)%360;
    }
    start_pos = 0;
    stop_pos = dis;

    double tf = 5;
    double a0 = 0;
    double a1 = 0;
    double a2 = 0;
    double a3 = 10*((double)dis)/(tf*tf*tf);
    double a4 = -15*((double)dis)/(tf*tf*tf*tf);
    double a5 = 6*((double)dis)/(tf*tf*tf*tf*tf);

    *n = (int)(tf/dt);

    double t = 0;
    for(int i =0;i < *n ; i++){
        datas[i] = a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
        t+=dt;
    }

    if(inverse){
        for(int i = 0;i<*n;i++){
            datas[i] *= -1;
        }
    }

}










uint64_t micros(){
	return _micro + TIM5->CNT;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim == &htim5){
		_micro += ((uint64_t)1<<32)-1;
	}

}






//UART protocol

void UARTInit(UARTStucrture *uart)
{
	//dynamic memory allocate
	uart->RxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.RxLen);
	uart->TxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.TxLen);
	uart->RxTail = 0;
	uart->TxTail = 0;
	uart->TxHead = 0;

}

void UARTResetStart(UARTStucrture *uart)
{
	HAL_UART_Receive_DMA(uart->huart, uart->RxBuffer, uart->RxLen);
}
uint32_t UARTGetRxHead(UARTStucrture *uart)
{
	return uart->RxLen - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);
}
int16_t UARTReadChar(UARTStucrture *uart)
{
	int16_t Result = -1; // -1 Mean no new data

	//check Buffer Position
	if (uart->RxTail != UARTGetRxHead(uart))
	{
		//get data from buffer
		Result = uart->RxBuffer[uart->RxTail];
		uart->RxTail = (uart->RxTail + 1) % uart->RxLen;

	}
	return Result;

}
void UARTTxDumpBuffer(UARTStucrture *uart)
{
	static uint8_t MultiProcessBlocker = 0;

	if (uart->huart->gState == HAL_UART_STATE_READY && !MultiProcessBlocker)
	{
		MultiProcessBlocker = 1;

		if (uart->TxHead != uart->TxTail)
		{
			//find len of data in buffer (Circular buffer but do in one way)
			uint16_t sentingLen =
					uart->TxHead > uart->TxTail ?
							uart->TxHead - uart->TxTail :
							uart->TxLen - uart->TxTail;

			//sent data via DMA
			HAL_UART_Transmit_DMA(uart->huart, &(uart->TxBuffer[uart->TxTail]),
					sentingLen);
			//move tail to new position
			uart->TxTail = (uart->TxTail + sentingLen) % uart->TxLen;

		}
		MultiProcessBlocker = 0;
	}
}
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len)
{
	//check data len is more than buffur?
	uint16_t lenAddBuffer = (len <= uart->TxLen) ? len : uart->TxLen;
	// find number of data before end of ring buffer
	uint16_t numberOfdataCanCopy =
			lenAddBuffer <= uart->TxLen - uart->TxHead ?
					lenAddBuffer : uart->TxLen - uart->TxHead;
	//copy data to the buffer
	memcpy(&(uart->TxBuffer[uart->TxHead]), pData, numberOfdataCanCopy);

	//Move Head to new position

	uart->TxHead = (uart->TxHead + lenAddBuffer) % uart->TxLen;
	//Check that we copy all data That We can?
	if (lenAddBuffer != numberOfdataCanCopy)
	{
		memcpy(uart->TxBuffer, &(pData[numberOfdataCanCopy]),
				lenAddBuffer - numberOfdataCanCopy);
	}
	UARTTxDumpBuffer(uart);

}



static uart_state state = state_idle;
static uint8_t sum = 0,debug = 0;
static uint8_t datas[256] = {0},data_ind = 0,n_data = 0;
static uint8_t mode = 0;

void uart_protocal(int16_t input,UARTStucrture *uart){

	debug = input;

	switch (state) {
		case state_idle:
			sum = data_ind = 0;
			if(input >= 0b10010001 && input <= 0b10011110){
				mode = input & 0b1111;
				sum += input;
				switch (mode){
					case 1:n_data = 2;state = state_data_frame;break;
					case 2:state = state_check_sum;break;
					case 3:state = state_check_sum;break;
					case 4:n_data = 2;state = state_data_frame;break;
					case 5:n_data = 2;state = state_data_frame;break;
					case 6:n_data = 2;state = state_data_frame;break;
					case 7:state = state_n_station;break;
					case 8:state = state_check_sum;break;
					case 9:state = state_check_sum;break;
					case 10:state = state_check_sum;break;
					case 11:state = state_check_sum;break;
					case 12:state = state_check_sum;break;
					case 13:state = state_check_sum;break;
					case 14:state = state_check_sum;break;
				}
			}
			else{
				sum = n_data = data_ind = mode = 0;
			}
			break;
		case state_n_station:

			//n_data = (input+1)/2 & 0xFF; //data sheet version

			n_data = input; //UI version


			sum+= input;
			state = state_data_frame;
			break;
		case state_data_frame:
			if(data_ind < n_data){
				datas[data_ind] = input;
				sum += datas[data_ind++];
			}
			if(data_ind == n_data){
				state = state_check_sum;
			}
			break;
		case state_check_sum:
			if(input == (uint8_t) ~sum){
				switch(mode){
					case 1:{
						uint8_t temp[] = { (0b1001<<4) | mode , datas[0] , datas[1] , (uint8_t)input};
						UARTTxWrite(&UART2, temp, 4);
						state = state_idle;
						break;
					}
					case 2:{
						mcu_connect = 1;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 3:{
						mcu_connect = 0;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 4:{
						max_velocity = datas[1];
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 5:{
						set_position = (double)((uint16_t)(datas[0]<<8) + datas[1])*1e-4;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 6:{
						goals[0] = datas[1];
						n_goal = 1;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 7:{
						n_goal = 0;
						for(int i = 0;i < n_data;i++){

							/* data sheet version
							goals[2*i] = datas[i] & 0b1111;
							goals[2*i+1] = datas[i]>>4;
							n_goal += 2;
							*/

							//UI version
							goals[i] = datas[i];
							n_goal++;
							//UI version
						}
						if(goals[n_goal-1] == 0){
							n_goal--;
						}
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 8:{
						go_now = 1;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 9:{
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						uint8_t temp2[] = {0b10011001,current_station,~(0b10011001+current_station) & 0xFF};
						UARTTxWrite(&UART2, temp2, 3);
						state = state_wait_for_ack1_1;
						break;
					}
					case 10:{
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						uint16_t pos = (uint16_t)(current_position*1e4);
						uint8_t temp2[] = {0b10011010,pos >> 8,pos & 0xFF, ~(0b10011001+(pos >> 8)+ (pos & 0xFF)) & 0xFF};
						UARTTxWrite(&UART2, temp2, 4);
						state = state_wait_for_ack1_1;
						break;
					}
					case 11:{
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						uint8_t temp2[] = {0b10011011,(uint8_t)max_velocity,~(0b10011011+(uint8_t)max_velocity) & 0xFF};
						UARTTxWrite(&UART2, temp2, 3);
						state = state_wait_for_ack1_1;
						break;
					}
					case 12:{
						enable_endeffector = 1;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 13:{
						enable_endeffector = 0;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
					case 14:{
						enable_sethome = 1;
						uint8_t temp[] = {0x58,0b01110101};
						UARTTxWrite(&UART2, temp, 2);
						state = state_idle;
						break;
					}
				}
			}
			else{
				//error check sum
			}
			break;
		case state_wait_for_ack1_1:{if(input == 0x58){state = state_wait_for_ack1_2;}break;}
		case state_wait_for_ack1_2:{if(input == 0b01110101){state = state_idle;}break;}
	}

}


void encoder_lowpass_update(){

	 encoder_value = unwraping_update();

	  // read encoder with low-pass
	  if(micros() - time_stamp > 1000){ // 1kHz
		  time_stamp = micros();

		  encoder_velocity = velocity_update(encoder_value);

		  for(int i = 0;i < 10;i++){
			  lowpass_output[i] = low_pass_process(&lowpass_filters[i], encoder_velocity);
		  }

		  encoder_velocity_rpm = ppms_to_rpm(lowpass_output[1]);
		  //kalman_output = kalman_filter_update(encoder_velocity*300);
	  }

}


void uart_update(){
	//Uart protocol
	int16_t inputChar = UARTReadChar(&UART2);
	if(inputChar != -1){
	  /*char temp[32];
	  sprintf(temp, "%d", inputChar);
	  UARTTxWrite(&UART2, (uint8_t*) temp, strlen(temp));*/
	  uart_protocal(inputChar, &UART2);
	}
	UARTTxDumpBuffer(&UART2);
}


void moving_state_update(){

	switch (move_state) {
		case state_move_idle:{if(go_now){move_state = state_tar_plan;} break;}
		case state_tar_plan:{ targectory_cal(paths, &path_n_cnt, (double)(TIM1->CNT)/(12*64*4 -1)*360,stations_postion[goals[station_ind++]] , 0.02); paths_ind = 0; move_state = state_wait_des; break;}
		case state_wait_des:{if(paths_ind >= path_n_cnt){time_stamp_5sec = micros(); enable_endeffector = 1; move_state = state_wait_5sec;} break;}
		case state_wait_5sec:{if(micros() - time_stamp_5sec >= 5e6){move_state = state_check_left_stations;} break;}
		case state_check_left_stations:{if(station_ind >= n_goal){go_now = station_ind = 0;uint8_t temp[] = {70,110}; UARTTxWrite(&UART2, temp, 2); state = state_idle;} move_state = state_move_idle; break;}
		default:break;
	}

}


#define end_effector_address 0x23
void end_effector_update(){
	if(enable_endeffector){
		uint8_t temp = 0x45;
		HAL_I2C_Master_Transmit(&hi2c1, end_effector_address << 1 , &temp, 1, 1000);
		enable_endeffector = 0;
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
