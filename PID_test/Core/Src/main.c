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



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint64_t _micro = 0;


int encoder_value = 0;
double encoder_velocity = 0,encoder_velocity_rpm = 0;
uint64_t time_stamp = 0,time_stamp2 = 0;


LowPass lowpass_filters[10] = {0};
double Wc_arr[15] = {0.00001,0.00002,0.00003,0.00004,0.00005,0.00006,0.00007,0.00008,0.00009,0.0001} ;
double lowpass_output[15] = {0};
double kalman_output = 0;

PID pids[2] = {0};
double pid_pwm_output = 0,setpoint = 0;

double paths[1000] = {0};
int paths_ind = 0,path_n_cnt = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

uint64_t micros();

int unwraping_update();
double velocity_update(int cur_pos);
double low_pass_process(LowPass *lowpass,double input);
double kalman_filter_update(double U);
double pid_update(PID *pid,double setpoint,double mea);
double ppms_to_rpm(double input);
void targectory_cal(double *datas,int *n,int start_pos,int stop_pos,double dt);


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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
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


  targectory_cal(paths, &path_n_cnt, 0, 10, 0.02);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


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

	  //pid control , system dead-time = 0.16 sec = 6.25 Hz 165000
	  if(micros() - time_stamp2 > 20000){ // 6.06Hz
	  		  time_stamp2 = micros();


	  		  setpoint = paths_ind < path_n_cnt ? paths[paths_ind++]/6:0;
	  		  pid_pwm_output = pid_update(&pids[0], setpoint, encoder_velocity_rpm);


	  		  if(pid_pwm_output > 0){
	  			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pid_pwm_output);
	  			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	  		  }
	  		  else{
	  			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	  			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,-pid_pwm_output);
	  		  }


	  	  }




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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

    if(dis < (v_max*v_max)/a_max){
        double ta = sqrt(dis/a_max);
        double T = ta*2;
        double tf = T;
        double t = 0;
        *n = (int)(T/dt);
        for(int i = 0;i <*n ;i++){
            if(t <= ta){
                datas[i] = a_max*t;
            }
            else{
                datas[i] = a_max*(tf-t);
            }
            t += dt;
        }
    }
    else{
        double T = (dis*a_max + (v_max*v_max))/(a_max*v_max);
        double ta = v_max/a_max;
        double tf = T;
        double t = 0;
        *n = (int)(T/dt);
        for(int i = 0;i < *n;i++){
            if(t <= ta){
                datas[i] = a_max*t;
            }
            else if(t <= tf-ta){
                datas[i] = a_max*ta;
            }
            else{
                datas[i] = a_max*(tf-t);
            }
            t += dt;
        }
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
