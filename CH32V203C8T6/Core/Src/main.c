#include "main.h"
#include "ch32v20x_RVMSIS.h"
volatile uint32_t TIM_Counter;
volatile uint32_t EXTI0_Counter;
//extern volatile uint16_t ADC_RAW_Data[2]; //Массив, куда будем кидать данные с АЦП
extern struct USART_name husart1; //Объявляем структуру по USART.(см. ch32v203x_RVMSIS.h)
uint8_t Arr[6] = {4,8,15,16,23,42};


void USART1_IRQHandler(void) {
   if (READ_BIT(USART1->STATR, USART_STATR_RXNE)) {
       //Если пришли данные по USART
       husart1.rx_buffer[husart1.rx_counter] = USART1->DATAR; //Считаем данные в соответствующую ячейку в rx_buffer
       husart1.rx_counter++; //Увеличим счетчик принятых байт на 1
   }
   if (READ_BIT(USART1->STATR, USART_STATR_IDLE)) {
       //Если прилетел флаг IDLE
       USART1->DATAR; //Сбросим флаг IDLE
       husart1.rx_len = husart1.rx_counter; //Узнаем, сколько байт получили
       husart1.rx_counter = 0; //сбросим счетчик приходящих данных
   }
}


void DMA1_Channel1_IRQHandler(void) {
    if (READ_BIT(DMA1->INTFR, DMA_TCIF1)) {
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //Сбросим глобальный флаг.
        /*Здесь можно писать код*/

    }
    else if (READ_BIT(DMA1->INTFR, DMA_TEIF1)) {
        /*Здесь можно сделать какой-то обработчик ошибок*/
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //Сбросим глобальный флаг.
    }
}

void TIM3_IRQHandler(void) {
    if (READ_BIT(TIM3->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM3->INTFR, TIM_UIF); //Сбросим флаг прерывания
        TIM_Counter++;
    }
}

void EXTI0_IRQHandler(void) {
    SET_BIT(EXTI->INTFR, EXTI_INTF_INTF0); //Команда выхода из прерывания
    EXTI0_Counter++;

}

int main(void){
    RVMSIS_Debug_init(); //Настройка дебага
    RVMSIS_RCC_SystemClock_144MHz(); //Настройка системной частоты
    RVMSIS_SysTick_Timer_init(); //Настройка системного таймера
    RVMSIS_PC13_OUTPUT_Push_Pull_init(); //Настройка PC13 в режим Push-pull
    RVMSIS_PA8_MCO_init(); //Выведем на ножку PA8 144 МГц
    RVMSIS_TIM3_init(); //Настроим таймер на 1 КГц
    RVMSIS_TIM3_PWM_CHANNEL1_init(); //Настроим ножку PA6 на выход ШИМ
    RVMSIS_TIM3_PWM_CHANNEL2_init(); //Настроим ножку PA7 на выход ШИМ(инверсный)
    RVMSIS_EXTI0_init(); //Настроим ножку PB0, как EXTI0 по восходящему фронту
    RVMSIS_ADC_DMA_init(); //Настроим ADC1+DMA Ch1 на ножках PA0 и PA1
    RVMSIS_USART1_Init(); //Настроим USART1 в режиме UART 9600 8N1. PA9 - Tx, PA10 - Rx

    while(1) {
        GPIOC->BSHR = GPIO_BSHR_BS13;
        Delay_ms(1000);
        GPIOC->BSHR = GPIO_BSHR_BR13;
        Delay_ms(1000);
        RVMSIS_USART_Transmit(USART1, Arr, 6, 100);


    }
}
