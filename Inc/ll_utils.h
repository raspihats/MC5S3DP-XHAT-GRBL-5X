/*
 * ll_utils.h
 *
 *  Created on: 17 Sep 2018
 *      Author: fcos
 */

#ifndef LL_UTILS_H_
#define LL_UTILS_H_

#include "stm32f4xx_ll_gpio.h"

void LL_GPIO_WriteOutputPin(GPIO_TypeDef* gpio, const uint32_t pin, const uint32_t value);

#endif /* LL_UTILS_H_ */
