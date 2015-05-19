/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @date    16/05/2015 23:21:20
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"

/* External variables --------------------------------------------------------*/

extern ADC_HandleTypeDef hadc;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim6;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles TIM6 global interrupt and DAC underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
  HAL_TIM_IRQHandler(&htim6);
}

/**
* @brief This function handles EXTI Line 4 to 15 interrupt.
*/
void EXTI4_15_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/**
* @brief This function handles ADC1, COMP1 and COMP2 global interrupts.
*/
void ADC1_COMP_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(ADC1_COMP_IRQn);
  HAL_ADC_IRQHandler(&hadc);
}

/**
* @brief This function handles SPI1 global interrupt.
*/
void SPI1_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
  HAL_SPI_IRQHandler(&hspi1);
}

/**
* @brief This function handles EXTI Line 0 and Line 1 interrupt.
*/
void EXTI0_1_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
