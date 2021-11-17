/**************************************************************************
* File Name             PWMCmd.c
* Description           
*				          
* Date				          Name(s)						          Action
* October 5, 2021		    Jaskaran K. & David C.			First Implementation
***************************************************************************/

/**************************************************************************
---------------------------- LIBRARY DEFINITIONS --------------------------
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>

#include "common.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_rcc_ex.h"
#include "stm32f4xx_hal_pwr_ex.h"
#include "stm32f4xx_hal_cortex.h"

/**************************************************************************
--------------------------- PRECOMPILER DEFINITIONS -----------------------
***************************************************************************/
#define MICROSECONDS 0
#define MILISECONDS 1

#define MAX_TIMERS 16
#define PROFILE_SAMPLES 20
/**************************************************************************
------------------------------- VARIABLE TYPES ----------------------------
***************************************************************************/
typedef struct {
  uint16_t *profile;
  uint32_t channelNum;
  uint8_t status;
}PWM_CHANNEL;

/**************************************************************************
---------------------------- GLOBAL VARIABLES --------------------------
***************************************************************************/
TIM_HandleTypeDef tim1;  // Timer Handler 
const uint16_t period = 10000;
uint16_t counter = 0;

uint16_t constantPlot[] = {32768, 32768, 32768, 32768, 32768, 32768, 32768,
                          32768, 32768, 32768, 32768, 32768, 32768, 32768, 32768,
                          32768, 32768, 32768, 32768, 32768};
uint16_t linearPlot[] = {0, 7281.7, 14563, 21845, 29127, 36408, 43690, 50972,
                         58253, 65535, 65535, 58253, 50972, 43690, 36408, 
                         29127, 21845, 14563, 7281.7 ,0};
uint16_t sinePlot[] = {32768, 43408, 52895, 60200, 64533, 65424, 62776, 56876,
                       48364, 38161, 27375, 17172, 8659.8, 2759.9, 1119.2, 1002.7,
                       5335.7, 12641, 22128, 32768};
uint16_t parabolaPlot[] = {0, 13070, 24688, 34855, 43569, 50830, 56640, 
                           60996, 63901, 65353, 65353, 63901, 60996, 56640, 
                           50830, 43569, 34855, 24688, 13070, 0};

PWM_CHANNEL ch1 = {constantPlot, TIM_CHANNEL_1, 0};
PWM_CHANNEL ch2 = {constantPlot, TIM_CHANNEL_2, 0};
PWM_CHANNEL ch3 = {constantPlot, TIM_CHANNEL_3, 0};
PWM_CHANNEL *ch1Ptr = &ch1;
PWM_CHANNEL *ch2Ptr = &ch2;
PWM_CHANNEL *ch3Ptr = &ch3;

/**************************************************************************
------------------------ OWN FUNCTION DEFINITIONS -------------------------
***************************************************************************/

/*--------------------------------------------------------------------------
*	Name:			    TimerInit
*	Description:	
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
ParserReturnVal_t PWMInit()
{ 
  // Set up the timer
  __HAL_RCC_TIM1_CLK_ENABLE();
  TIM_OC_InitTypeDef sConfig;
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  tim1.Instance = TIM1;
  tim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 100000 - 1;
  tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim1.Init.Period = period;
  tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim1.Init.RepetitionCounter = 0;
  tim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&tim1) != HAL_OK)
  {
    printf("Error 1 initializing the timer\n");
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&tim1, &sClockSourceConfig) != HAL_OK)
  {
    printf("Error 2 initializing the timer\n");
  }
  
  HAL_NVIC_SetPriority((IRQn_Type) TIM1_UP_TIM10_IRQn, (uint32_t) 0, (uint32_t) 1);
  HAL_NVIC_EnableIRQ((IRQn_Type) TIM1_UP_TIM10_IRQn);
  // Start timer
  HAL_TIM_Base_Start_IT(&tim1);

  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.Pulse = period/2;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState =TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_3);

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* Configure the PWM output pins */
  GPIO_InitStruct.Pin = (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  // Adjust profile arrays for period
  for(int i = 0; i < PROFILE_SAMPLES; i++)
  {
    constantPlot[i] = (constantPlot[i] * period) / 65535;
    linearPlot[i] = (linearPlot[i] * period) / 65535;
    sinePlot[i] = (sinePlot[i] * period) / 65535;
    parabolaPlot[i] = (parabolaPlot[i] * period) / 65535;
    //printf("%d, ", sinePlot[i]);
  }
  //HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_3);

  return CmdReturnOk;
}
// MACRO: Add new command to help menu
ADD_CMD("pwminit", PWMInit,"\t\tInitializes PWM.")

/*--------------------------------------------------------------------------
*	Name:			    TimerInit
*	Description:	
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
ParserReturnVal_t PWMSweep()
{ 
  uint16_t tempChannel;
  uint16_t tempProfile;
  if(fetch_uint16_arg(&tempChannel)){
    printf("Please enter the channel number\n");
  }
  else{
    if(fetch_uint16_arg(&tempProfile)){
      printf("Please enter the type of increment you want\n");
    }
    else{
      switch(tempChannel){
        
        case 1:{
          // Initialize channel 1 of  PWM
          ch1Ptr->status = 1;
          HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
          switch(tempProfile){
            case 0:{
              ch1Ptr->profile = constantPlot;
              break;
            }
            case 1:{
              ch1Ptr->profile = linearPlot;
              break;
            }
            case 2:{
              ch1Ptr->profile = parabolaPlot;
              break;
            }
            case 3:{
              ch1Ptr->profile = sinePlot;
              break;
            }
            default:{
              printf("Please enter a profile between 0 and 3\n");
              break;
            }
          }
          break;
        }

        case 2:{
          // Initialize channel 2 of  PWM
          ch2Ptr->status = 1;
          HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_2);
          switch(tempProfile){
            case 0:{
              ch2Ptr->profile = constantPlot;
              break;
            }
            case 1:{
              ch2Ptr->profile = linearPlot;
              break;
            }
            case 2:{
              ch2Ptr->profile = parabolaPlot;
              break;
            }
            case 3:{
              ch2Ptr->profile = sinePlot;
              break;
            }
            default:{
              printf("Please enter a profile between 0 and 3\n");
              break;
            }
          }
          break;
        }
        case 3:{
          // Initialize channel 3 of  PWM
          ch3Ptr->status = 1;
          HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_3);
          switch(tempProfile){
            case 0:{
              ch3Ptr->profile = constantPlot;
              break;
            }
            case 1:{
              ch3Ptr->profile = linearPlot;
              break;
            }
            case 2:{
              ch3Ptr->profile = parabolaPlot;
              break;
            }
            case 3:{
              ch3Ptr->profile = sinePlot;
              break;
            }
            default:{
              printf("Please enter a profile between 0 and 3\n");
              break;
            }
          }
          break;
        }
        default:{
          printf("Please enter a channel between 1 and 3\n");
          break;
        }
      }
    }
  }

  return CmdReturnOk;
} 
// MACRO: Add new command to help menu
ADD_CMD("pwmsweep", PWMSweep,"\t\tSet type of duty cycle sweep.")

/*--------------------------------------------------------------------------
*	Name:			    TimerInit
*	Description:	
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  // This will call for "HAL_TIM_PeriodElapsedCallback()" on timer update 
  HAL_TIM_IRQHandler(&tim1);
}
/*--------------------------------------------------------------------------
*	Name:			    TimerInit
*	Description:	
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
void incrementDC(TIM_HandleTypeDef *htim, PWM_CHANNEL *chx)
{ 
  if(chx->status)
  {
    __HAL_TIM_SET_COMPARE(htim, (chx->channelNum), (chx->profile[counter]));
    //__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, parabolaPlot[counter]);
  }
}
/*--------------------------------------------------------------------------
*	Name:			    TimerInit
*	Description:	
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  incrementDC(htim, ch1Ptr);
  incrementDC(htim, ch2Ptr);
  incrementDC(htim, ch3Ptr);

  if(++counter >= PROFILE_SAMPLES)
  {
    counter = 0;
  }
}

