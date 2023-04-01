#include "main.h"
#include "ch32v20x_RVMSIS.h"
volatile uint32_t TIM_Counter;
volatile uint32_t EXTI0_Counter;
//extern volatile uint16_t ADC_RAW_Data[2]; //Массив, куда будем кидать данные с АЦП


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

int main(void) {
    RVMSIS_Debug_init(); //Настройка дебага
    RVMSIS_RCC_SystemClock_144MHz(); //Настройка системной частоты
    RVMSIS_SysTick_Timer_init(); //Настройка системного таймера
    RVMSIS_PC13_OUTPUT_Push_Pull_init(); //Настройка PC13 в режим Push-pull
    RVMSIS_PA8_MCO_init(); //Выведем на ножку PA8 144 МГц
    RVMSIS_TIM3_init(); //Настроим таймер на 1 КГц
    RVMSIS_TIM3_PWM_CHANNEL1_init(); //Настроим ножку PA6 на выход ШИМ
    RVMSIS_TIM3_PWM_CHANNEL2_init(); //Настроим ножку PA7 на выход ШИМ(инверсный)
    RVMSIS_EXTI0_init(); //Настроим ножку PB0, как EXTI0 по восходящему фронту
    RVMSIS_ADC_DMA_init();

    while(1) {
        GPIOC->BSHR = GPIO_BSHR_BS13;
        Delay_ms(1000);
        GPIOC->BSHR = GPIO_BSHR_BR13;
        Delay_ms(1000);

    }
}
