/*
 * ch32v203xx_it.c
 *
 *  Created on: Aug 17, 2023
 *      Author: Oleg Volkov
 */

#include "ch32v203xx_it.h"
#include "SoftwareTimer.h"
#include "Buttons.h"

extern volatile uint32_t SysTimer_ms; //§±§Ö§â§Ö§Þ§Ö§ß§ß§Ñ§ñ, §Ñ§ß§Ñ§Ý§à§Ô§Ú§é§ß§Ñ§ñ HAL_GetTick()
extern volatile uint32_t Delay_counter_ms; //§³§é§Ö§ä§é§Ú§Ü §Õ§Ý§ñ §æ§å§ß§Ü§è§Ú§Ú Delay_ms
extern volatile uint32_t Timeout_counter_ms; //§±§Ö§â§Ö§Þ§Ö§ß§ß§Ñ§ñ §Õ§Ý§ñ §ä§Ñ§Û§Þ§Ñ§å§ä§Ñ §æ§å§ß§Ü§è§Ú§Û

extern struct USART_name husart1; //§°§Ò§ì§ñ§Ó§Ý§ñ§Ö§Þ §ã§ä§â§å§Ü§ä§å§â§å §á§à USART.(§ã§Þ. ch32v203x_RVMSIS.h)
extern struct USART_name husart2; //§°§Ò§ì§ñ§Ó§Ý§ñ§Ö§Þ §ã§ä§â§å§Ü§ä§å§â§å §á§à USART.(§ã§Þ. ch32v203x_RVMSIS.h)
extern struct USART_name husart3; //§°§Ò§ì§ñ§Ó§Ý§ñ§Ö§Þ §ã§ä§â§å§Ü§ä§å§â§å §á§à USART.(§ã§Þ. ch32v203x_RVMSIS.h)

/*
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §æ§Ý§Ñ§Ô§å CNTIF
 ******************************************************************************
 */

extern bool Task1;
extern uint32_t I2C1_Polling_Timer; //§´§Ñ§Û§Þ§Ö§â §Õ§Ý§ñ §à§á§â§à§ã§Ñ HDC1080
extern bool Task2;
extern uint32_t GMG12864_Timer; //§´§Ñ§Û§Þ§Ö§â §Õ§Ý§ñ §â§Ñ§Ò§à§ä§í §Õ§Ú§ã§á§Ý§Ö§ñ

extern uint32_t TIMER_wait_before_write_flash; //§°§Ø§Ú§Õ§Ñ§Ö§Þ §á§Ö§â§Ö§Õ §Ù§Ñ§á§Ú§ã§î§ð §Õ§Ñ§ß§ß§í§ç §Ó§à §æ§Ý§Ö§ê

void SysTick_Handler(void) {
    SysTick->SR &= ~(1 << 0);
    SysTimer_ms++;

    if (Delay_counter_ms) {
        Delay_counter_ms--;
    }
    if (Timeout_counter_ms) {
        Timeout_counter_ms--;
    }

    if (!Software_timer_check((uint32_t*) &I2C1_Polling_Timer)) {
        Task1 = true;
    }
    if (!Software_timer_check((uint32_t*) &GMG12864_Timer)) {
        Task2 = true;
    }

    if (TIMER_wait_before_write_flash){
        TIMER_wait_before_write_flash--;
    }
}

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à USART1
 ******************************************************************************
 */

void USART1_IRQHandler(void) {
    if (READ_BIT(USART1->STATR, USART_STATR_RXNE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§ê§Ý§Ú §Õ§Ñ§ß§ß§í§Ö §á§à USART
        if (husart1.rx_counter < USART_MAX_LEN_RX_BUFFER) { //§¦§ã§Ý§Ú §Ò§Ñ§Û§ä §á§â§Ú§Ý§Ö§ä§Ö§Ý§à §Þ§Ö§ß§î§ê§Ö, §é§Ö§Þ §â§Ñ§Ù§Þ§Ö§â §Ò§å§æ§Ö§â§Ñ
            husart1.rx_buffer[husart1.rx_counter] = USART1->DATAR; //§³§é§Ú§ä§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö §Ó §ã§à§à§ä§Ó§Ö§ä§ã§ä§Ó§å§ð§ë§å§ð §ñ§é§Ö§Û§Ü§å §Ó rx_buffer
            husart1.rx_counter++; //§µ§Ó§Ö§Ý§Ú§é§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ß§ñ§ä§í§ç §Ò§Ñ§Û§ä §ß§Ñ 1
        } else {
            husart1.rx_counter = 0; //§¦§ã§Ý§Ú §Ò§à§Ý§î§ê§Ö - §ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü.
        }
    }
    if (READ_BIT(USART1->STATR, USART_STATR_IDLE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§Ý§Ö§ä§Ö§Ý §æ§Ý§Ñ§Ô IDLE
        USART1->DATAR; //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô IDLE
        husart1.rx_len = husart1.rx_counter; //§µ§Ù§ß§Ñ§Ö§Þ, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §á§à§Ý§å§é§Ú§Ý§Ú
        husart1.rx_counter = 0; //§ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ç§à§Õ§ñ§ë§Ú§ç §Õ§Ñ§ß§ß§í§ç
    }
}

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à USART2
 ******************************************************************************
 */

void USART2_IRQHandler(void) {
    if (READ_BIT(USART2->STATR, USART_STATR_RXNE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§ê§Ý§Ú §Õ§Ñ§ß§ß§í§Ö §á§à USART
        if (husart2.rx_counter < USART_MAX_LEN_RX_BUFFER) { //§¦§ã§Ý§Ú §Ò§Ñ§Û§ä §á§â§Ú§Ý§Ö§ä§Ö§Ý§à §Þ§Ö§ß§î§ê§Ö, §é§Ö§Þ §â§Ñ§Ù§Þ§Ö§â §Ò§å§æ§Ö§â§Ñ
            husart2.rx_buffer[husart2.rx_counter] = USART2->DATAR; //§³§é§Ú§ä§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö §Ó §ã§à§à§ä§Ó§Ö§ä§ã§ä§Ó§å§ð§ë§å§ð §ñ§é§Ö§Û§Ü§å §Ó rx_buffer
            husart2.rx_counter++; //§µ§Ó§Ö§Ý§Ú§é§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ß§ñ§ä§í§ç §Ò§Ñ§Û§ä §ß§Ñ 1
        } else {
            husart2.rx_counter = 0; //§¦§ã§Ý§Ú §Ò§à§Ý§î§ê§Ö - §ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü.
        }
    }
    if (READ_BIT(USART2->STATR, USART_STATR_IDLE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§Ý§Ö§ä§Ö§Ý §æ§Ý§Ñ§Ô IDLE
        USART2->DATAR; //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô IDLE
        husart2.rx_len = husart2.rx_counter; //§µ§Ù§ß§Ñ§Ö§Þ, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §á§à§Ý§å§é§Ú§Ý§Ú
        husart2.rx_counter = 0; //§ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ç§à§Õ§ñ§ë§Ú§ç §Õ§Ñ§ß§ß§í§ç
    }
}

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à USART3
 ******************************************************************************
 */

void USART3_IRQHandler(void) {
    if (READ_BIT(USART3->STATR, USART_STATR_RXNE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§ê§Ý§Ú §Õ§Ñ§ß§ß§í§Ö §á§à USART
        if (husart3.rx_counter < USART_MAX_LEN_RX_BUFFER) { //§¦§ã§Ý§Ú §Ò§Ñ§Û§ä §á§â§Ú§Ý§Ö§ä§Ö§Ý§à §Þ§Ö§ß§î§ê§Ö, §é§Ö§Þ §â§Ñ§Ù§Þ§Ö§â §Ò§å§æ§Ö§â§Ñ
            husart3.rx_buffer[husart3.rx_counter] = USART3->DATAR; //§³§é§Ú§ä§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö §Ó §ã§à§à§ä§Ó§Ö§ä§ã§ä§Ó§å§ð§ë§å§ð §ñ§é§Ö§Û§Ü§å §Ó rx_buffer
            husart3.rx_counter++; //§µ§Ó§Ö§Ý§Ú§é§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ß§ñ§ä§í§ç §Ò§Ñ§Û§ä §ß§Ñ 1
        } else {
            husart3.rx_counter = 0; //§¦§ã§Ý§Ú §Ò§à§Ý§î§ê§Ö - §ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü.
        }
    }
    if (READ_BIT(USART3->STATR, USART_STATR_IDLE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§Ý§Ö§ä§Ö§Ý §æ§Ý§Ñ§Ô IDLE
        USART3->DATAR; //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô IDLE
        husart3.rx_len = husart3.rx_counter; //§µ§Ù§ß§Ñ§Ö§Þ, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §á§à§Ý§å§é§Ú§Ý§Ú
        husart3.rx_counter = 0; //§ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ç§à§Õ§ñ§ë§Ú§ç §Õ§Ñ§ß§ß§í§ç
    }
}

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à TIM3
 ******************************************************************************
 */
extern Buttons Button0;
extern Buttons Button1;
extern Buttons Button2;
extern Buttons Button3;

void TIM3_IRQHandler(void) {
    if (READ_BIT(TIM3->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM3->INTFR, TIM_UIF); //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ
    }
      ButtonAction((uint8_t*) &Button0.StateOld, (uint8_t*) &Button0.StateNew, (uint8_t*) &Button0.TimeHold, BUTTON0_STATE, (void*) Button0_Callback);
      ButtonAction((uint8_t*) &Button1.StateOld, (uint8_t*) &Button1.StateNew, (uint8_t*) &Button1.TimeHold, BUTTON1_STATE, (void*) Button1_Callback);
      ButtonAction((uint8_t*) &Button2.StateOld, (uint8_t*) &Button2.StateNew, (uint8_t*) &Button2.TimeHold, BUTTON2_STATE, (void*) Button2_Callback);
      ButtonAction((uint8_t*) &Button3.StateOld, (uint8_t*) &Button3.StateNew, (uint8_t*) &Button3.TimeHold, BUTTON3_STATE, (void*) Button3_Callback);
}
