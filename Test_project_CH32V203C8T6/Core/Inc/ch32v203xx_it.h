/*
 * ch32v203xx_it.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Oleg Volkov
 */

#ifndef INC_CH32V203XX_IT_H_
#define INC_CH32V203XX_IT_H_

#include "main.h"
#include "ch32v20x_RVMSIS.h"


__attribute__((interrupt("machine"))) void SysTick_Handler(void);
__attribute__((interrupt("machine"))) void TIM1_UP_IRQHandler(void);
__attribute__((interrupt("machine"))) void TIM3_IRQHandler(void);
__attribute__((interrupt("machine"))) void TIM4_IRQHandler(void);
__attribute__((interrupt("machine"))) void EXTI0_IRQHandler(void);
__attribute__((interrupt("machine"))) void ADC1_2_IRQHandler(void);
__attribute__((interrupt("machine"))) void DMA1_Channel1_IRQHandler(void);
__attribute__((interrupt("machine"))) void USART1_IRQHandler(void);
__attribute__((interrupt("machine"))) void USART2_IRQHandler(void);
__attribute__((interrupt("machine"))) void USART3_IRQHandler(void);


#endif /* INC_CH32V203XX_IT_H_ */
