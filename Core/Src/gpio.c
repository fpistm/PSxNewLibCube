/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

}

/* USER CODE BEGIN 2 */
bool enable_gpio_clock(GPIO_TypeDef *port)
{
  bool ret = true;
  switch ((uint32_t)port) {
    case (uint32_t)GPIOA_BASE:
      __HAL_RCC_GPIOA_CLK_ENABLE();
      break;
    case (uint32_t)GPIOB_BASE:
      __HAL_RCC_GPIOB_CLK_ENABLE();
      break;
#if defined GPIOC_BASE
    case (uint32_t)GPIOC_BASE:
      __HAL_RCC_GPIOC_CLK_ENABLE();
      break;
#endif
#if defined GPIOD_BASE
    case (uint32_t)GPIOD_BASE:
      __HAL_RCC_GPIOD_CLK_ENABLE();
      break;
#endif
#if defined GPIOE_BASE
    case (uint32_t)GPIOE_BASE:
      __HAL_RCC_GPIOE_CLK_ENABLE();
      break;
#endif
#if defined GPIOF_BASE
    case (uint32_t)GPIOF_BASE:
      __HAL_RCC_GPIOF_CLK_ENABLE();
      break;
#endif
#if defined GPIOG_BASE
    case (uint32_t)GPIOG_BASE:
#if defined(PWR_CR2_IOSV) || defined(PWR_SVMCR_IO2VMEN)
      // Enable VDDIO2 supply for 14 I/Os (Port G[15:2])
      HAL_PWREx_EnableVddIO2();
#endif
      __HAL_RCC_GPIOG_CLK_ENABLE();
      break;
#endif
#if defined GPIOH_BASE
    case (uint32_t)GPIOH_BASE:
      __HAL_RCC_GPIOH_CLK_ENABLE();
      break;
#endif
#if defined GPIOI_BASE
    case (uint32_t)GPIOI_BASE:
      __HAL_RCC_GPIOI_CLK_ENABLE();
      break;
#endif
#if defined GPIOJ_BASE
    case (uint32_t)GPIOJ_BASE:
      __HAL_RCC_GPIOJ_CLK_ENABLE();
      break;
#endif
#if defined GPIOK_BASE
    case (uint32_t)GPIOK_BASE:
      __HAL_RCC_GPIOK_CLK_ENABLE();
      break;
#endif
#if defined GPIOZ_BASE
    case (uint32_t)GPIOZ_BASE:
      __HAL_RCC_GPIOZ_CLK_ENABLE();
      break;
#endif
    default:
      // wrong port number
      ret = false;
      break;
  }
  return ret;
}

void configure_gpio(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t pull)
{
  if (enable_gpio_clock(port)) {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    LL_GPIO_Init(port, &GPIO_InitStruct);
  }
}
/* USER CODE END 2 */
