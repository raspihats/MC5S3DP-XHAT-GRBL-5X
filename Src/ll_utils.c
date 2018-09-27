/*
 * ll_utils.c
 *
 *  Created on: 17 Sep 2018
 *      Author: fcos
 */

#include "ll_utils.h"


void LL_GPIO_WriteOutputPin(GPIO_TypeDef* gpio, const uint32_t pin, const uint32_t value) {
  if(value > 0) {
    LL_GPIO_SetOutputPin(gpio, pin);
  }
  else {
    LL_GPIO_ResetOutputPin(gpio, pin);
  }
}

