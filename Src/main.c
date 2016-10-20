/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "MC_const.h"
#include "MC_type.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
unsigned int iAccumulator = 0;
volatile unsigned int iFreqCode = 0;
volatile uint8_t cBotState = SET;
volatile uint8_t cRampTurnOnOff = 0;
Volt_Components Stat_Volt_alfa_beta;        /*Valpha & Vbeta, RevPark transformations of Vq & Vd*/
uint8_t iTableAddr = 0; 
//static s16 hSin_Theta,*/ hCos_Theta;
const int16_t hSin_Cos_Table[256] = SIN_COS_TABLE;
extern volatile unsigned int iRampTick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void RampTurnOn(void);
void RampTurnOff(void);
void GPIO_Set_Led3(unsigned char On);
void GPIO_Set_Led4(unsigned char On);
void GPIO_Set_Sync(unsigned char On);
unsigned char GPIO_GetBtn();
uint32_t Adc_Read(uint8_t channel);
void SVPWM_IcsCalcDutyCycles (Volt_Components Stat_Volt_Input);
void UpdatePWM();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define LED3_GPIO_PORT          GPIOA	
#define LED3_GPIO_PIN           GPIO_PIN_5

#define LED4_GPIO_PORT          GPIOB	
#define LED4_GPIO_PIN           GPIO_PIN_13

#define BTN_GPIO_PORT          GPIOC	
#define BTN_GPIO_PIN           GPIO_PIN_13

#define SQRT_3		1.732051
#define T		(PWM_PERIOD * 4)
#define T_SQRT3         (uint16_t)(T * SQRT_3)

#define SECTOR_1	(uint32_t)1
#define SECTOR_2	(uint32_t)2
#define SECTOR_3	(uint32_t)3
#define SECTOR_4	(uint32_t)4
#define SECTOR_5	(uint32_t)5
#define SECTOR_6	(uint32_t)6
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
//  TIM_Cmd(TIM1, ENABLE);
//  TIM_CtrlPWMOutputs( TIM1, ENABLE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
    iFreqCode = Adc_Read(13)*5243;
//    if(cBotState == RESET)
//    {
//      if(cRampTurnOnOff == TURN_ON)
//      {
//        RampTurnOn();
//      }
//      else if (cRampTurnOnOff == TURN_OFF)
//      {
//        RampTurnOff();
//      }
//      cBotState = SET;
//    }
//    if (cRampTurnOnOff == TURN_POT)
//    {
//      iFreqCode = Adc_Read(13)*5243;
//    } 
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/10000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(-1, 0, 0);
}

/* USER CODE BEGIN 4 */
void GPIO_Set_Led3(unsigned char On)
{
  if (On)
    LED3_GPIO_PORT->BSRR = LED3_GPIO_PIN; 
  else
    LED3_GPIO_PORT->BRR = LED3_GPIO_PIN;
}

void GPIO_Set_Led4(unsigned char On)
{
  if (On)
    LED4_GPIO_PORT->BSRR = LED4_GPIO_PIN; 
  else
    LED4_GPIO_PORT->BRR = LED4_GPIO_PIN;
}

void GPIO_Set_Sync(unsigned char On)
{
  if (On)
    GPIOB->BSRR = GPIO_PIN_10; 
  else
    GPIOB->BRR = GPIO_PIN_10;
}

unsigned char GPIO_GetBtn()
{
  return HAL_GPIO_ReadPin(BTN_GPIO_PORT, BTN_GPIO_PIN);
}

uint32_t adc_read;
uint32_t Adc_Read(uint8_t channel)
{
  //uint8_t channel = 0;
  //uint8_t channel = 10;
  //ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5);
  // Start the conversion
//  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  HAL_ADC_Start(&hadc1);
  // Wait until conversion completion
  //while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  HAL_ADC_PollForConversion(&hadc1,1000);
  // Get the conversion value
//  return ADC_GetConversionValue(ADC1);
  adc_read = HAL_ADC_GetValue(&hadc1);
  return adc_read;
}

void RampTurnOn(void)
{
  unsigned int iFreq;
  iFreq = iFreqCode/5243; //get the current frequency
  while (iFreq < 4095)
  {
    if(!iRampTick)   
    {
      iFreq +=163; //2Hz upgrade in final frequency
      iFreqCode = iFreq*5243;
      iRampTick = 1000;
    }   
  }
 
}

void RampTurnOff(void)
{
  unsigned int iFreq;
  iFreq = iFreqCode/5243; //get the current frequency
  while (iFreq > 165 )
  {
    if(!iRampTick)   
    {
      iFreq -=163; //2Hz decrement in the final frequency
      iFreqCode = iFreq*5243;
      iRampTick = 1000;
    }   
  }
  iFreqCode = 0;
}


/*******************************************************************************
* Function Name  : SVPWM_IcsCalcDutyCycles
* Description    : Computes duty cycle values corresponding to the input value
and configures 
* Input          : Stat_Volt_alfa_beta
* Output         : None
* Return         : None
*******************************************************************************/

void SVPWM_IcsCalcDutyCycles (Volt_Components Stat_Volt_Input)
{
  uint8_t bSector;
  int32_t wX, wY, wZ, wUAlpha, wUBeta;
  uint16_t  hTimePhA=0, hTimePhB=0, hTimePhC=0;
  
  wUAlpha = Stat_Volt_Input.qV_Component1 * T_SQRT3 ;
  wUBeta = -(Stat_Volt_Input.qV_Component2 * T);
  
  wX = wUBeta;
  wY = (wUBeta + wUAlpha)/2;
  wZ = (wUBeta - wUAlpha)/2;
  
  // Sector calculation from wX, wY, wZ
  if (wY<0)
  {
    if (wZ<0)
    {
      bSector = SECTOR_5;
    }
    else // wZ >= 0
      if (wX<=0)
      {
        bSector = SECTOR_4;
      }
      else // wX > 0
      {
        bSector = SECTOR_3;
      }
  }
  else // wY > 0
  {
    if (wZ>=0)
    {
      bSector = SECTOR_2;
    }
    else // wZ < 0
      if (wX<=0)
      {  
        bSector = SECTOR_6;
      }
      else // wX > 0
      {
        bSector = SECTOR_1;
      }
  }
  
  /* Duty cycles computation */
  
  switch(bSector)
  {  
  case SECTOR_1:
  case SECTOR_4:
    hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
    hTimePhB = hTimePhA + wZ/131072;
    hTimePhC = hTimePhB - wX/131072;                                       
    break;
  case SECTOR_2:
  case SECTOR_5:  
    hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
    hTimePhB = hTimePhA + wZ/131072;
    hTimePhC = hTimePhA - wY/131072;
    break;
    
  case SECTOR_3:
  case SECTOR_6:
    hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
    hTimePhC = hTimePhA - wY/131072;
    hTimePhB = hTimePhC + wX/131072;
    break;
  default:
    break;
  }
  
  /* Load compare registers values */
  
  TIM1->CCR1 = hTimePhA;
  TIM1->CCR2 = hTimePhB;
  TIM1->CCR3 = hTimePhC;
}


void UpdatePWM()
{
  iAccumulator = iAccumulator + iFreqCode;
  
  iTableAddr = iAccumulator >> 24;
  Stat_Volt_alfa_beta.qV_Component1 = hSin_Cos_Table[(iTableAddr+OFFSET)&0xFF] / 2;
  Stat_Volt_alfa_beta.qV_Component2 = hSin_Cos_Table[iTableAddr] / 2;
  SVPWM_IcsCalcDutyCycles(Stat_Volt_alfa_beta);
  
  if (Stat_Volt_alfa_beta.qV_Component2 & 0x8000) 
  { 
    GPIO_Set_Led4(1); 
    GPIO_Set_Sync(1); 
  } 
  else 
  { 
    GPIO_Set_Led4(0); 
    GPIO_Set_Sync(0); 
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
