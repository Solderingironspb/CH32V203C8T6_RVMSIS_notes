/*
 * ch32v203xx_it.c
 *
 *  Created on: Aug 17, 2023
 *      Author: Oleg Volkov
 */

#include "ch32v203xx_it.h"
#include "SoftwareTimer.h"
#include "Buttons.h"

extern volatile uint32_t SysTimer_ms; //���֧�֧ާ֧ߧߧѧ�, �ѧߧѧݧ�ԧڧ�ߧѧ� HAL_GetTick()
extern volatile uint32_t Delay_counter_ms; //����֧��ڧ� �էݧ� ���ߧܧ�ڧ� Delay_ms
extern volatile uint32_t Timeout_counter_ms; //���֧�֧ާ֧ߧߧѧ� �էݧ� ��ѧۧާѧ��� ���ߧܧ�ڧ�

extern struct USART_name husart1; //���ҧ��ӧݧ�֧� �����ܧ���� ��� USART.(���. ch32v203x_RVMSIS.h)
extern struct USART_name husart2; //���ҧ��ӧݧ�֧� �����ܧ���� ��� USART.(���. ch32v203x_RVMSIS.h)
extern struct USART_name husart3; //���ҧ��ӧݧ�֧� �����ܧ���� ��� USART.(���. ch32v203x_RVMSIS.h)

/*
 ******************************************************************************
 *  @breif ����֧��ӧѧߧڧ� ��� ��ݧѧԧ� CNTIF
 ******************************************************************************
 */

extern bool Task1;
extern uint32_t I2C1_Polling_Timer; //���ѧۧާ֧� �էݧ� ������� HDC1080
extern bool Task2;
extern uint32_t GMG12864_Timer; //���ѧۧާ֧� �էݧ� ��ѧҧ��� �էڧ��ݧ֧�

extern uint32_t TIMER_wait_before_write_flash; //���اڧէѧ֧� ��֧�֧� �٧ѧ�ڧ��� �էѧߧߧ�� �ӧ� ��ݧ֧�

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
 *  @breif ����֧��ӧѧߧڧ� ��� USART1
 ******************************************************************************
 */

void USART1_IRQHandler(void) {
    if (READ_BIT(USART1->STATR, USART_STATR_RXNE)) {
        //����ݧ� ���ڧ�ݧ� �էѧߧߧ�� ��� USART
        if (husart1.rx_counter < USART_MAX_LEN_RX_BUFFER) { //����ݧ� �ҧѧۧ� ���ڧݧ֧�֧ݧ� �ާ֧ߧ���, ��֧� ��ѧ٧ާ֧� �ҧ��֧��
            husart1.rx_buffer[husart1.rx_counter] = USART1->DATAR; //����ڧ�ѧ֧� �էѧߧߧ�� �� �����ӧ֧���ӧ����� ���֧ۧܧ� �� rx_buffer
            husart1.rx_counter++; //���ӧ֧ݧڧ�ڧ� ���֧��ڧ� ���ڧߧ���� �ҧѧۧ� �ߧ� 1
        } else {
            husart1.rx_counter = 0; //����ݧ� �ҧ�ݧ��� - ��ҧ���ڧ� ���֧��ڧ�.
        }
    }
    if (READ_BIT(USART1->STATR, USART_STATR_IDLE)) {
        //����ݧ� ���ڧݧ֧�֧� ��ݧѧ� IDLE
        USART1->DATAR; //���ҧ���ڧ� ��ݧѧ� IDLE
        husart1.rx_len = husart1.rx_counter; //���٧ߧѧ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� ���ݧ��ڧݧ�
        husart1.rx_counter = 0; //��ҧ���ڧ� ���֧��ڧ� ���ڧ��է��ڧ� �էѧߧߧ��
    }
}

/**
 ******************************************************************************
 *  @breif ����֧��ӧѧߧڧ� ��� USART2
 ******************************************************************************
 */

void USART2_IRQHandler(void) {
    if (READ_BIT(USART2->STATR, USART_STATR_RXNE)) {
        //����ݧ� ���ڧ�ݧ� �էѧߧߧ�� ��� USART
        if (husart2.rx_counter < USART_MAX_LEN_RX_BUFFER) { //����ݧ� �ҧѧۧ� ���ڧݧ֧�֧ݧ� �ާ֧ߧ���, ��֧� ��ѧ٧ާ֧� �ҧ��֧��
            husart2.rx_buffer[husart2.rx_counter] = USART2->DATAR; //����ڧ�ѧ֧� �էѧߧߧ�� �� �����ӧ֧���ӧ����� ���֧ۧܧ� �� rx_buffer
            husart2.rx_counter++; //���ӧ֧ݧڧ�ڧ� ���֧��ڧ� ���ڧߧ���� �ҧѧۧ� �ߧ� 1
        } else {
            husart2.rx_counter = 0; //����ݧ� �ҧ�ݧ��� - ��ҧ���ڧ� ���֧��ڧ�.
        }
    }
    if (READ_BIT(USART2->STATR, USART_STATR_IDLE)) {
        //����ݧ� ���ڧݧ֧�֧� ��ݧѧ� IDLE
        USART2->DATAR; //���ҧ���ڧ� ��ݧѧ� IDLE
        husart2.rx_len = husart2.rx_counter; //���٧ߧѧ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� ���ݧ��ڧݧ�
        husart2.rx_counter = 0; //��ҧ���ڧ� ���֧��ڧ� ���ڧ��է��ڧ� �էѧߧߧ��
    }
}

/**
 ******************************************************************************
 *  @breif ����֧��ӧѧߧڧ� ��� USART3
 ******************************************************************************
 */

void USART3_IRQHandler(void) {
    if (READ_BIT(USART3->STATR, USART_STATR_RXNE)) {
        //����ݧ� ���ڧ�ݧ� �էѧߧߧ�� ��� USART
        if (husart3.rx_counter < USART_MAX_LEN_RX_BUFFER) { //����ݧ� �ҧѧۧ� ���ڧݧ֧�֧ݧ� �ާ֧ߧ���, ��֧� ��ѧ٧ާ֧� �ҧ��֧��
            husart3.rx_buffer[husart3.rx_counter] = USART3->DATAR; //����ڧ�ѧ֧� �էѧߧߧ�� �� �����ӧ֧���ӧ����� ���֧ۧܧ� �� rx_buffer
            husart3.rx_counter++; //���ӧ֧ݧڧ�ڧ� ���֧��ڧ� ���ڧߧ���� �ҧѧۧ� �ߧ� 1
        } else {
            husart3.rx_counter = 0; //����ݧ� �ҧ�ݧ��� - ��ҧ���ڧ� ���֧��ڧ�.
        }
    }
    if (READ_BIT(USART3->STATR, USART_STATR_IDLE)) {
        //����ݧ� ���ڧݧ֧�֧� ��ݧѧ� IDLE
        USART3->DATAR; //���ҧ���ڧ� ��ݧѧ� IDLE
        husart3.rx_len = husart3.rx_counter; //���٧ߧѧ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� ���ݧ��ڧݧ�
        husart3.rx_counter = 0; //��ҧ���ڧ� ���֧��ڧ� ���ڧ��է��ڧ� �էѧߧߧ��
    }
}

/**
 ******************************************************************************
 *  @breif ����֧��ӧѧߧڧ� ��� TIM3
 ******************************************************************************
 */
extern Buttons Button0;
extern Buttons Button1;
extern Buttons Button2;
extern Buttons Button3;

void TIM3_IRQHandler(void) {
    if (READ_BIT(TIM3->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM3->INTFR, TIM_UIF); //���ҧ���ڧ� ��ݧѧ� ���֧��ӧѧߧڧ�
    }
      ButtonAction((uint8_t*) &Button0.StateOld, (uint8_t*) &Button0.StateNew, (uint8_t*) &Button0.TimeHold, BUTTON0_STATE, (void*) Button0_Callback);
      ButtonAction((uint8_t*) &Button1.StateOld, (uint8_t*) &Button1.StateNew, (uint8_t*) &Button1.TimeHold, BUTTON1_STATE, (void*) Button1_Callback);
      ButtonAction((uint8_t*) &Button2.StateOld, (uint8_t*) &Button2.StateNew, (uint8_t*) &Button2.TimeHold, BUTTON2_STATE, (void*) Button2_Callback);
      ButtonAction((uint8_t*) &Button3.StateOld, (uint8_t*) &Button3.StateNew, (uint8_t*) &Button3.TimeHold, BUTTON3_STATE, (void*) Button3_Callback);
}
