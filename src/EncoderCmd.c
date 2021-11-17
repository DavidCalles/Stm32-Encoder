/**************************************************************************
* File Name             Encoder.c
* Description           
*				          
* Date				          Name(s)						          Action
* November 17, 2021		  David C.			First Implementation
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
#define DC_MOTOR_IN1_PIN GPIO_PIN_0
#define DC_MOTOR_IN2_PIN GPIO_PIN_1
#define DC_MOTOR_ENABLE_PIN GPIO_PIN_8

#define DC_MOTOR_PORT GPIOA

#define TOLERANCE 200

/**************************************************************************
------------------------------- VARIABLE TYPES ----------------------------
***************************************************************************/


/**************************************************************************
---------------------------- GLOBAL VARIABLES --------------------------
***************************************************************************/
TIM_HandleTypeDef tim3;

const uint16_t periodEncoder = 0xFFFF;
const uint16_t period = 100;

volatile int32_t desiredPosition;
volatile int32_t currentPosition;

TIM_HandleTypeDef tim1;  // Timer Handler 

uint16_t pins[] = {DC_MOTOR_IN1_PIN, DC_MOTOR_IN2_PIN, DC_MOTOR_ENABLE_PIN};
GPIO_TypeDef* ports[] = {DC_MOTOR_PORT, DC_MOTOR_PORT, DC_MOTOR_PORT};


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

ParserReturnVal_t encoderInit()
{
  HAL_StatusTypeDef rc;
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* initialize GPIO pins PA6 and PA7 */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Initialize direction pins
  GPIO_InitTypeDef My_GPIO_InitStructA = {0};
  GPIO_InitTypeDef My_GPIO_InitStructB = {0};
  GPIO_InitTypeDef My_GPIO_InitStructC = {0};  

  for(int i = 0; i< 3; i++){
    if(ports[i] == GPIOA){
      My_GPIO_InitStructA.Pin |= (pins[i]);
    }
    if(ports[i] == GPIOB){
      My_GPIO_InitStructB.Pin |= (pins[i]);
    }
    if(ports[i] == GPIOC){
      My_GPIO_InitStructC.Pin |= (pins[i]);
    }
  }

  if(My_GPIO_InitStructA.Pin != 0){
    My_GPIO_InitStructA.Mode = GPIO_MODE_OUTPUT_PP;
    My_GPIO_InitStructA.Pull = GPIO_NOPULL;
    My_GPIO_InitStructA.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &My_GPIO_InitStructA);
  }
  

  if(My_GPIO_InitStructB.Pin != 0){
    My_GPIO_InitStructB.Mode = GPIO_MODE_OUTPUT_PP;
    My_GPIO_InitStructB.Pull = GPIO_NOPULL;
    My_GPIO_InitStructB.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &My_GPIO_InitStructB);
  }

  if(My_GPIO_InitStructC.Pin != 0){
    My_GPIO_InitStructC.Mode = GPIO_MODE_OUTPUT_PP;
    My_GPIO_InitStructC.Pull = GPIO_NOPULL;
    My_GPIO_InitStructC.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &My_GPIO_InitStructC);
  }
  /* Configure these timer pins mode using 
  HAL_GPIO_Init() */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* Configure these timer pins
    mode using HAL_GPIO_Init() */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
  // Set up the timer for PWM
  __HAL_RCC_TIM1_CLK_ENABLE();
  //TIM_OC_InitTypeDef sConfig;
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
  

  __HAL_RCC_TIM3_CLK_ENABLE();
  tim3.Instance = TIM3;
  tim3.Init.Prescaler = 0;
  tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim3.Init.Period = periodEncoder;
  tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim3.Init.RepetitionCounter = 0;
  rc = HAL_TIM_Base_Init(&tim3);
  if (rc != HAL_OK)
  {
      printf("Failed to initialize Timer 3 Base, “ ”rc=%u\n", rc);
      return CmdReturnBadParameter1;
  }

  TIM_Encoder_InitTypeDef encoderConfig;
  encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  encoderConfig.IC1Polarity = 0;
  encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC1Prescaler = 0;
  encoderConfig.IC1Filter = 3;
  encoderConfig.IC2Polarity = 0;
  encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC2Prescaler = 0;
  encoderConfig.IC2Filter = 3;
  rc = HAL_TIM_Encoder_Init(&tim3, &encoderConfig);
  if (rc != HAL_OK)
  {
      printf("Failed to initialize Timer 3 Encoder, "
              "rc=%u\n",
              rc);
      return CmdReturnBadParameter1;
  }
  rc = HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_1);
  if (rc != HAL_OK)
  {
      printf("Failed to start Timer 3 Encoder, "
              "rc=%u\n",
              rc);
      return CmdReturnBadParameter1;
  }
  rc = HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_2);
  if (rc != HAL_OK)
  {
      printf("Failed to start Timer 3 Encoder, "
              "rc=%u\n",
              rc);
      return CmdReturnBadParameter1;
  }
  //HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_ENABLE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN2_PIN, GPIO_PIN_RESET);

  return CmdReturnOk;
}
// MACRO: Add new command to help menu
ADD_CMD("encoderinit", encoderInit, "\t\tInitializes timer 3 as encoder mode.")
/*--------------------------------------------------------------------------
*	Name:			    TimerInit
*	Description:	
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
ParserReturnVal_t Encoder()
{ 
  uint16_t currentEnc = __HAL_TIM_GET_COUNTER(&tim3); 
  
  printf("%d\n", currentEnc);

  return CmdReturnOk;
}
// MACRO: Add new command to help menu
ADD_CMD("encoder", Encoder, "\t\tGet current encoder value")

/*--------------------------------------------------------------------------
*	Name:			    TimerInit
*	Description:	
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
ParserReturnVal_t SetPosition()
{ 
  int32_t posx;

  if(fetch_int32_arg(&posx)){
    printf("No argument."
    "Please enter (0) for Brake, (1) for Forward, or (2) for reverse.\n");
  }
  else{
    if(posx > 0){
      HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN1_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN2_PIN, GPIO_PIN_RESET);
      desiredPosition = posx;
    }
    else{
      HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN2_PIN, GPIO_PIN_SET);
      desiredPosition = -posx;
    }
  }

  return CmdReturnOk;
}
// MACRO: Add new command to help menu
ADD_CMD("setposition", SetPosition, "\t\tGet current encoder value")
/*--------------------------------------------------------------------------
*	Name:			    TIM1_UP_TIM10_IRQHandler
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
*	Name:			    HAL_TIM_PeriodElapsedCallback
*	Description:	
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

  currentPosition = (int32_t)__HAL_TIM_GET_COUNTER(&tim3); 

  if((currentPosition > (desiredPosition+TOLERANCE)) || (currentPosition < (desiredPosition-TOLERANCE))){
    HAL_GPIO_TogglePin(DC_MOTOR_PORT, DC_MOTOR_ENABLE_PIN);
  }
  else{
    HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_ENABLE_PIN, GPIO_PIN_RESET);
  }
  
}

