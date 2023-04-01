/*
 * ch32v20x_registers.h
 *
 *  Created on: Mar 30, 2023
 *      Author: Oleg Volkov
 */

#ifndef USER_CH32V20X_REGISTERS_H_
#define USER_CH32V20X_REGISTERS_H_

#include "main.h"

#define __WEAK   __attribute__((weak))
#define __INTERRUPT __attribute__ ((interrupt("WCH-Interrupt-fast")))

__INTERRUPT void SysTick_Handler(void);
__INTERRUPT void TIM3_IRQHandler(void);
__INTERRUPT void EXTI0_IRQHandler(void);
__INTERRUPT void ADC1_2_IRQHandler(void);
__INTERRUPT void DMA1_Channel1_IRQHandler(void);

void RVMSIS_Debug_init(void);
void RVMSIS_RCC_SystemClock_144MHz(void);
void RVMSIS_SysTick_Timer_init(void);
void Delay_ms(uint32_t Milliseconds);
void RVMSIS_PC13_OUTPUT_Push_Pull_init(void);
void RVMSIS_Blink_PC13(uint32_t ms);
void RVMSIS_PA8_MCO_init(void);
void RVMSIS_EXTI0_init(void);
void RVMSIS_TIM3_init(void);
void RVMSIS_TIM3_PWM_CHANNEL1_init(void);
void RVMSIS_TIM3_PWM_CHANNEL2_init(void);
void RVMSIS_ADC_DMA_init(void);

#endif /* USER_CH32V20X_REGISTERS_H_ */
