/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t curtim;
    uint32_t pastim;
    uint32_t tim;
    float speed;
    uint8_t direction;
    float pid_out_date;
}motor;
typedef struct {//pid算法结构????
    float Kp; // 比例系数
    float Ki; // 积分系数
    float Kd; // 微分系数
    float target; // 目标????
    float error; // 当前误差
    float last_error; // 上一次误????
    float integral; // 积分
    float derivative; // 微分
    float output; // PID 输出
    float max_output;//????大输????
    float min_output;//????小输????
} PID_TypeDef;
typedef struct{
    float x;
    float y;
} bluetoothdata;
PID_TypeDef lm;//左轮子pid
PID_TypeDef rm;//右轮子pid
motor lift_motor;
motor right_motor;
uint8_t receivedate[11];
bluetoothdata receive;
float numl;
float numr;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float PID_Calculate(PID_TypeDef *pid, float actual);
void motor_init();
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float target);
void Set_Motor_PWM(TIM_HandleTypeDef *htim, uint32_t channel, float pwm);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//函数实现
void UART_Send_Float(UART_HandleTypeDef *huart, float value)
{
    char buffer[32];
    int length;
    length = sprintf(buffer, "%.6f", value);
    HAL_UART_Transmit(huart, (uint8_t*)buffer, length,100);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim==&htim1){
        uint32_t capture1=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
        uint32_t capture2=HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
        lift_motor.curtim=capture1;
        right_motor.curtim=capture2;
        //左轮????
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)==0){
            if(lift_motor.curtim>=lift_motor.pastim){
                lift_motor.tim=lift_motor.curtim-lift_motor.pastim;
            } else{
                //lift_motor.tim=(lift_motor.curtim+0xFFFF)-lift_motor.pastim;
            }
            lift_motor.direction=0;
        }
        else{
            if(lift_motor.curtim<=lift_motor.pastim){
                lift_motor.tim =lift_motor.pastim-lift_motor.curtim;
            }else {
                //lift_motor.tim = (lift_motor.pastim+0xFFFF)-lift_motor.curtim;
            }
            lift_motor.direction=1;
        }
        float kl;
        if(lift_motor.direction==0) {
            kl = (float) lift_motor.tim;
        }
        else{
            kl = -(float) lift_motor.tim;
        }
        //右轮????
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)==0){
            if(right_motor.curtim>=right_motor.pastim){
                right_motor.tim=right_motor.curtim-right_motor.pastim;
            } else{
                //right_motor.tim=(right_motor.curtim+0xFFFF)-right_motor.pastim;
            }
            right_motor.direction=0;
        }
        else{
            if(right_motor.curtim<=right_motor.pastim){
                right_motor.tim =right_motor.pastim-right_motor.curtim;
            }else {
               // right_motor.tim = (right_motor.pastim+0xFFFF)-right_motor.curtim;
            }
            right_motor.direction=1;
        }
        float kr;
        if(right_motor.direction==0) {
            kr = (float) right_motor.tim;
        }
        else{
            kr = -(float) right_motor.tim;
        }
        //计算速度
        lift_motor.speed=60*(kl/700)/0.01;
        lift_motor.pastim=lift_motor.curtim;
        right_motor.speed=60*(kr/700)/0.01;
        right_motor.pastim=right_motor.curtim;
        lift_motor.pid_out_date= PID_Calculate(&lm,lift_motor.speed);
        right_motor.pid_out_date= PID_Calculate(&rm,right_motor.speed);
        //Set_Motor_PWM(&htim1,TIM_CHANNEL_1,lift_motor.pid_out_date);
        //Set_Motor_PWM(&htim1,TIM_CHANNEL_4,right_motor.pid_out_date);
    }
}
void motor_init(){//对左右轮参数进行初始????
    lift_motor.curtim=0;
    lift_motor.pastim=0;
    lift_motor.tim=0;
    lift_motor.direction=0;
    lift_motor.speed=0;
    right_motor.curtim=0;
    right_motor.pastim=0;
    right_motor.tim=0;
    right_motor.direction=0;
    right_motor.speed=0;
}
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float target)//pid初始化函????
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = target;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
    pid->max_output=200;
    pid->min_output=-200;
}
float PID_Calculate(PID_TypeDef *pid, float actual)//pid计算函数
{
    // 计算误差
    pid->error = pid->target - actual;

    // 计算积分
    pid->integral += pid->error;
    if(pid->integral>pid->max_output)pid->integral=pid->max_output;
    if(pid->integral<pid->min_output)pid->integral=pid->min_output;

    // 计算微分
    pid->derivative = pid->error - pid->last_error;

    // 计算 PID 输出
    pid->output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * pid->derivative;

    //输出限幅
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < pid->min_output) pid->output = pid->min_output;

    // 更新上一次误????
    pid->last_error = pid->error;

    return pid->output;
}
// 设置电机PWM输出（带方向控制????
void Set_Motor_PWM(TIM_HandleTypeDef *htim, uint32_t channel, float pwm) {
    // 限制PWM范围
    //if (pwm > 400.0f) pwm = 400.0f;
    //else if (pwm < -400.0f) pwm = -400.0f;
if(channel==TIM_CHANNEL_4) {
    // 设置方向
    if (pwm >= 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);   // 反转
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET); // 正转
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        pwm = -pwm;  // 取绝对???
    }
    // 计算PWM占空比（假设ARR=999，对????100%占空比）
    uint32_t pulse = (uint32_t) (10000 * pwm / 60); //????大???是700，pwm/700就是百分比，然后再导????65535的分辨率中输????
    //Send_uint32_t_in_char(&huart2,pulse);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}
else{
    // 设置方向
    if (pwm >= 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);   // 反转
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); // 正转
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        pwm = -pwm;  // 取绝对???
    }
    // 计算PWM占空比（假设ARR=999，对????100%占空比）
    uint32_t pulse = (uint32_t) (10000 * pwm / 60); //????大???是700，pwm/700就是百分比，然后再导????65535的分辨率中输????
    //Send_uint32_t_in_char(&huart2,pulse);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}
}
uint8_t calculate_checksum(const uint8_t* data,uint8_t length){
    uint8_t sum=0;
    for(uint8_t i=0;i<length;i++){
        sum+=data[i];
    }
    return sum;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(receivedate[0]==0xA5&&receivedate[10]==0x5A){
            if(receivedate[9]== calculate_checksum(&receivedate[1],8)) {
                memcpy(&receive.x, &receivedate[1], 4);
                memcpy(&receive.y, &receivedate[5], 4);
                HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
            }
        }//这一部分是蓝牙的通讯协议
   // HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
    HAL_UART_Receive_DMA(&huart1, receivedate, sizeof(receivedate));//开启DMA接收中断

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    motor_init();
    //HAL_TIM_Base_Start_IT(&htim1);//启动定时器
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    PID_Init(&lm,0.4,0.3,0.2,64);
    PID_Init(&rm,0.4,0.3,0.2,0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_UART_Receive_DMA(&huart1, receivedate, sizeof(receivedate));//开启DMA接收中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);

      float nl,nr;
      nl=receive.y+receive.x;
      nr=receive.y-receive.x;
     if(nl>140){
         nl=140;
     }
     if(nr>140){
         nr=140;
     }
      float lift,right;
      lift=10000*((nl)/140);
      right=10000*((nr)/140);
      if(lift<0){
          HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);   // 反转
          HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
          lift=-lift;
      }
      else{
          HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);   // 正转
          HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);

      }
      if(right<0){
          HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);   // 反转
          HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
          right=-right;
      }
      else{
          HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);   // 正转
          HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

      }
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, lift);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, right);
      ///下面这一段是往复循环移动的小车的代码
//      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);   // 反转
//      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);   // 反转
//      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 5200);
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4500);
//      HAL_Delay(3000);
//      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);   // 反转
//      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);   // 反转
//      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//      HAL_Delay(500);
//      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);   // 反转
//      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);   // 反转
//      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 5200);
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4500);
//      HAL_Delay(3000);
//      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);   // 反转
//      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);   // 反转
//      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//      HAL_Delay(500);


//      numl=receive.x-receive.y;
//      numr=receive.x+receive.y;
//      if(numl>140){
//          numl=140;
//      }
//      else if(numl<-140){
//          numl=-140;
//      }
//      if(numr>140){
//          numr=140;
//      }
//      else if(numr<-140){
//          numr=-140;
//      }
      //PID_Init(&lm,0.7,0,0,-numl);
      //PID_Init(&rm,0.7,0,0,-numr);
//      float pl,pr;
//      pl=-(numl/140)*65535;
//      pr=-(numr/140)*65535;
//      Set_Motor_PWM(&htim1,TIM_CHANNEL_1,pl);
//      Set_Motor_PWM(&htim1,TIM_CHANNEL_4,pr);
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
