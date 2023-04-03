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


  //Структура по USART
    struct USART_name {
        uint8_t tx_buffer[20]; //Буфер под выходящие данные
        uint8_t rx_buffer[20]; //Буфер под входящие данные
        uint16_t rx_counter; //Счетчик приходящих данных типа uint8_t по USART
        uint16_t rx_len; //Количество принятых байт после сработки флага IDLE
    };

__INTERRUPTF void SysTick_Handler(void);
__INTERRUPTF void TIM3_IRQHandler(void);
__INTERRUPTF void EXTI0_IRQHandler(void);
__INTERRUPTF void ADC1_2_IRQHandler(void);
__INTERRUPTF void DMA1_Channel1_IRQHandler(void);
__INTERRUPTF void USART1_IRQHandler(void);
__INTERRUPTF void USART2_IRQHandler(void);


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
void RVMSIS_I2C_Reset(void);
void RVMSIS_I2C1_Init(void);
bool RVMSIS_I2C_Adress_Device_Scan(I2C_TypeDef* I2C, uint8_t Adress_Device, uint32_t Timeout_ms);
bool RVMSIS_I2C_Data_Transmit(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_I2C_Data_Receive(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_I2C_MemWrite(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_I2C_MemRead(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
void RVMSIS_SPI1_init(void);
bool RVMSIS_SPI_Data_Transmit_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_SPI_Data_Transmit_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_SPI_Data_Receive_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_SPI_Data_Receive_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms);

#endif /* USER_CH32V20X_REGISTERS_H_ */
