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
#define __INTERRUPTF __attribute__ ((interrupt("WCH-Interrupt-fast")))


  //§³§ä§â§å§Ü§ä§å§â§Ñ §á§à USART
    struct USART_name {
        uint8_t tx_buffer[20]; //§¢§å§æ§Ö§â §á§à§Õ §Ó§í§ç§à§Õ§ñ§ë§Ú§Ö §Õ§Ñ§ß§ß§í§Ö
        uint8_t rx_buffer[20]; //§¢§å§æ§Ö§â §á§à§Õ §Ó§ç§à§Õ§ñ§ë§Ú§Ö §Õ§Ñ§ß§ß§í§Ö
        uint16_t rx_counter; //§³§é§Ö§ä§é§Ú§Ü §á§â§Ú§ç§à§Õ§ñ§ë§Ú§ç §Õ§Ñ§ß§ß§í§ç §ä§Ú§á§Ñ uint8_t §á§à USART
        uint16_t rx_len; //§¬§à§Ý§Ú§é§Ö§ã§ä§Ó§à §á§â§Ú§ß§ñ§ä§í§ç §Ò§Ñ§Û§ä §á§à§ã§Ý§Ö §ã§â§Ñ§Ò§à§ä§Ü§Ú §æ§Ý§Ñ§Ô§Ñ IDLE
    };

__INTERRUPTF void SysTick_Handler(void);
__INTERRUPTF void TIM3_IRQHandler(void);
__INTERRUPTF void EXTI0_IRQHandler(void);
__INTERRUPTF void ADC1_2_IRQHandler(void);
__INTERRUPTF void DMA1_Channel1_IRQHandler(void);
__INTERRUPTF void USART1_IRQHandler(void);


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
void RVMSIS_USART1_Init(void);
bool RVMSIS_USART_Transmit(USART_TypeDef* USART, uint8_t* data, uint16_t Size, uint32_t Timeout_ms);

#endif /* USER_CH32V20X_REGISTERS_H_ */
