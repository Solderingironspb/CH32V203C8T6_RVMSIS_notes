/**
 ******************************************************************************
 *  @file ch32v20x_RVMSIS.h
 *  @brief RVMSIS на примере МК CH32V203C8T6
 *  @author Волков Олег
 *  @date 31.03.2023
 *
 ******************************************************************************
 * @attention
 *
 *  Библиотека помогает разобраться с библиотекой RVMSIS на примере
 *  МК CH32V203C8T6
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/STM32F103C8T6_CMSIS_notes
 *  Группа ВК: https://vk.com/solderingiron.stm32
 *  Работал по документации: http://www.wch-ic.com/products/CH32V203.html?
 *
 ******************************************************************************
 */
#include "ch32v20x_RVMSIS.h"

/*================================= НАСТРОЙКА DEBUG ============================================*/
/**
 ***************************************************************************************
 *  @breif Debug port mapping
 ***************************************************************************************
 */
void RVMSIS_Debug_init(void) {
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Запустим тактирование порта A
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //Запустим тактирование альтернативных функций
    MODIFY_REG(AFIO->PCFR1, AFIO_PCFR1_SWJ_CFG, 0b000 << AFIO_PCFR1_SWJ_CFG_Pos); //Serial wire

    /**
     *  При выборе Serial wire:
     *  PA13 /JTMS/SWDIO
     *  PA14 /JTCK/SWCLK.
     *  PA15, PB3 и PB4 свободны
     */
    /*Заблокируем доступ для редактирования конфигурации PA13 и PA14*/

    GPIOA->LCKR = GPIO_LCKK | GPIO_LCK13 | GPIO_LCK14;
    GPIOA->LCKR = GPIO_LCK13 | GPIO_LCK14;
    GPIOA->LCKR = GPIO_LCKK | GPIO_LCK13 | GPIO_LCK14;
    GPIOA->LCKR;
}

/*============================== НАСТРОЙКА RCC =======================================*/
/**
 ***************************************************************************************
 *  @breif Настройка МК CH32V203C8T6 на частоту 144MHz от внешнего кварцевого резонатора
 *  P.S. можете кидать в меня камнями, но я считаю, что от 144МГц тут пользы, как
 *  от козла молока) I2C1 и I2C2 на APB1. Для них максимум 36 МГц.
 *  Скорость FLASH максимум 60 МГц, а делитель там есть только на 2, что опять
 *  не дает нам выставить 144 МГц. А если мы хотим работать с ADC, то опять же, на APB2
 *  мы не можем выставить 144 МГц, т.к. максимальный делитель 8(144/8 = 18МГц.)
 *  А максимум для АЦП - 14 МГц...Поэтому для универсальной работы всего, частоты задирать
 *  выше 72 МГц не будем.
 *  Внешний кварцевый резонатор на 8 MHz
 *  ADC настроен на 12MHz
 *  USB настроен на 48MHz
 *  MCO подключен к HSE и тактируется от 8MHz
 *
 ***************************************************************************************
 */
void RVMSIS_RCC_SystemClock_144MHz(void) {
    SET_BIT(RCC->CTLR, RCC_HSION); //Запустим внутренний RC генератор на 8 МГц
    while (READ_BIT(RCC->CTLR, RCC_HSIRDY) == 0);
    //Дождемся поднятия флага о готовности
    CLEAR_BIT(RCC->CTLR, RCC_HSEBYP);//Просто сбросим этот бит в 0(Хотя изначально он и так должен быть в 0).
    SET_BIT(RCC->CTLR, RCC_HSEON); //Запустим внешний кварцевый резонатор. Он у нас на 8 MHz.
    while (READ_BIT(RCC->CTLR, RCC_HSERDY) == 0);
    //Дождемся поднятия флага о готовности
    SET_BIT(RCC->CTLR, RCC_CSSON);//Включим CSS
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b01 << RCC_SW_Pos); //Выберем HSE в качестве System Clock(PLL лучше пока не выбирать, он у нас отключен)
    CLEAR_BIT(RCC->CTLR, RCC_PLLON); //Выключим PLL
    MODIFY_REG(RCC->CFGR0, RCC_HPRE, RCC_HPRE_DIV2); //AHB prescaler /2
    //Note: FLASH access clock frequency cannot
    //be more than 60 MHz.
    CLEAR_BIT(FLASH->CTLR, 1 << 25U); //0: FLASH access clock frequency = SYSCLK/2.
    MODIFY_REG(RCC->CFGR0, RCC_PPRE1, RCC_PPRE1_DIV2); //APB1 Prescaler /2,  72/2 = 36 MHz.(Иначе у нас I2C будет не настроить...)
    MODIFY_REG(RCC->CFGR0, RCC_PPRE2, RCC_PPRE2_DIV1); //APB2 Prescaler /1. Будет 72MHz.
    MODIFY_REG(RCC->CFGR0, RCC_ADCPRE, RCC_ADCPRE_DIV6); //ADC Prescaler /6, чтоб было 12MHz, т.к. максимальная частота тут 14 MHz
    CLEAR_BIT(RCC->CFGR0, RCC_PLLXTPRE); //0: HSE clock not divided.
    SET_BIT(RCC->CFGR0, RCC_PLLSRC); //В качестве входного сигнала для PLL выберем HSE
    MODIFY_REG(RCC->CFGR0, RCC_PLLMULL, 0b1111 << RCC_PLLMULL_Pos); //т.к. кварц у нас 8Mhz, а нам нужно 144MHz, то в PLL нужно сделать умножение на 18. 8MHz * 18 = 144MHz.
    MODIFY_REG(RCC->CFGR0, RCC_USBPRE, 0b10 << RCC_USBPRE_Pos); //10: Divided by 3 (when PLLCLK=144MHz); 144/3 = 48 МГц
    MODIFY_REG(RCC->CFGR0, RCC_CFGR0_MCO, RCC_CFGR0_MCO_HSE); //В качестве тактирования для MCO выбрал HSE. Будет 8 MHz.
    SET_BIT(RCC->CTLR, RCC_PLLON); //Запустим PLL

    //Т.к. PLL уже запущен, выберем его в качестве System Clock:
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b10 << RCC_SW_Pos);//Выберем PLL в качестве System Clock
    while (READ_BIT(RCC->CTLR, RCC_PLLRDY) == 0);
    //Дожидемся поднятия флага включения PLL

}

/*========================= НАСТРОЙКА СИСТЕМНОГО ТАЙМЕРА ==============================*/

/**
 ***************************************************************************************
 *  @breif Настройка SysTick на микросекунды
 *  На этом таймере мы настроим Delay и аналог HAL_GetTick()
 ***************************************************************************************
 */
void RVMSIS_SysTick_Timer_init(void) {
    SysTick->CTLR &= ~(1 << 0); //Выключим таймер для проведения настроек.
    SysTick->CTLR |= (1 << 1); //1: Enable counter interrupts.
    SysTick->CTLR &= ~(1 << 2); //0: HCLK for time base.144/8 =18
    SysTick->CTLR |= (1 << 3); //1: Re-counting from 0 after counting up to the comparison value, and re-counting from the comparison value after counting down to 0
    SysTick->CTLR |= (1 << 4); //0: Counting up.
    SysTick->CTLR |= (1 << 5); //1: Updated to 0 on up counts, updated to the comparison value on down counts.
    SysTick->CMP = 8999; ////Настроим прерывание на частоту в 1 кГц(т.е. сработка будет каждую мс) 18000000 / 18000 = 1000Гц
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->CTLR |= (1 << 0); //Запустим таймер.
}

/**
 ***************************************************************************************
 *  @breif Настройка Delay и аналог HAL_GetTick()
 ***************************************************************************************
 */
volatile uint32_t SysTimer_ms = 0; //Переменная, аналогичная HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0; //Счетчик для функции Delay_ms
volatile uint32_t Timeout_counter_ms = 0; //Переменная для таймаута функций

/**
 ******************************************************************************
 *  @breif Прерывание по флагу CNTIF
 ******************************************************************************
 */
void SysTick_Handler(void) {
    SysTick->SR &= ~(1 << 0);
    SysTimer_ms++;

    if (Delay_counter_ms) {
        Delay_counter_ms--;
    }
    if (Timeout_counter_ms) {
        Timeout_counter_ms--;
    }
}

/**
 ******************************************************************************
 *  @breif Delay_ms
 *  @param   uint32_t Milliseconds - Длина задержки в миллисекундах
 ******************************************************************************
 */
void Delay_ms(uint32_t Milliseconds) {
    Delay_counter_ms = Milliseconds;
    while (Delay_counter_ms != 0);
}

/*============================== НАСТРОЙКА GPIO =======================================*/

/**
 ***************************************************************************************
 *  @breif Инициализация PIN PC13 на выход в режиме Push-Pull с максимальной скоростью 50 MHz
 *  Перед настройкой (GPIOs and AFIOs) нужно включить тактирование порта.
 ***************************************************************************************
 */
void RVMSIS_PC13_OUTPUT_Push_Pull_init(void) {
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //Запустим тактирование порта C
    MODIFY_REG(GPIOC->CFGHR, GPIO_CFGHR_MODE13, 0b10 << GPIO_CFGHR_MODE13_Pos); //Настройка GPIOC порта 13 на выход со максимальной скоростью в 50 MHz
    MODIFY_REG(GPIOC->CFGHR, GPIO_CFGHR_CNF13, 0b00 << GPIO_CFGHR_CNF13_Pos); //Настройка GPIOC порта 13 на выход в режиме Push-Pull

}

//Служебная функция
static void RVMSIS_GPIO_MODE_Set(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, uint8_t Reg, uint8_t Data) {
    uint8_t Mode = 0;
    switch (Reg) {
    case(0):
        Mode = GPIO_Pin * 4;
        MODIFY_REG(GPIO->CFGLR, (0x3UL << Mode), Data << Mode);
        break;
    case(1):
        GPIO_Pin = GPIO_Pin - 8;
        Mode = GPIO_Pin * 4;
        MODIFY_REG(GPIO->CFGHR, (0x3UL << Mode), Data << Mode);
        break;
    }
}

//Служебная функция
static void RVMSIS_GPIO_SPEED_Set(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, uint8_t Speed) {
    uint8_t Reg = 0;
    if (GPIO_Pin < 8) {
        Reg = 0;
    } else {
        Reg = 1;
    }
    //MODE
    if (Speed == GPIO_SPEED_RESERVED) {
        RVMSIS_GPIO_MODE_Set(GPIO, GPIO_Pin, Reg, 0b00);
    } else if (Speed == GPIO_SPEED_10_MHZ) {
        RVMSIS_GPIO_MODE_Set(GPIO, GPIO_Pin, Reg, 0b01);
    } else if (Speed == GPIO_SPEED_2_MHZ) {
        RVMSIS_GPIO_MODE_Set(GPIO, GPIO_Pin, Reg, 0b10);
    } else if (Speed == GPIO_SPEED_50_MHZ) {
        RVMSIS_GPIO_MODE_Set(GPIO, GPIO_Pin, Reg, 0b11);
    }
}

//Служебная функция
static void RVMSIS_GPIO_CNF_Set(GPIO_TypeDef *GPIO, uint8_t Reg, uint8_t Mode, uint8_t* CNF_Pos) {
    switch (Reg) {
    case (0):
        MODIFY_REG(GPIO->CFGLR, (0x3UL << *CNF_Pos), Mode << *CNF_Pos);
        break;
    case(1):
        MODIFY_REG(GPIO->CFGHR, (0x3UL << *CNF_Pos), Mode << *CNF_Pos);
    }
}

//Служебная функция
static void RVMSIS_GPIO_Reg_Set(GPIO_TypeDef *GPIO, uint8_t* GPIO_Pin, uint8_t Configuration_mode, uint8_t Type, uint8_t Reg, uint8_t* CNF_Pos) {
    switch (Configuration_mode) {
    case(GPIO_GENERAL_PURPOSE_OUTPUT):
        switch (Type) {
        case (GPIO_OUTPUT_PUSH_PULL):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b00, *(&CNF_Pos));
            break;
        case(GPIO_OUTPUT_OPEN_DRAIN):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b01, *(&CNF_Pos));
            break;
        }
        break;
    case(GPIO_ALTERNATIVE_FUNCTION_OUTPUT):
        switch (Type) {
        case (GPIO_OUTPUT_PUSH_PULL):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b10, *(&CNF_Pos));
            break;
        case(GPIO_OUTPUT_OPEN_DRAIN):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b11, *(&CNF_Pos));
            break;
        }
        break;
    case(GPIO_INPUT):
        switch (Type) {
        case(GPIO_INPUT_ANALOG):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b00, *(&CNF_Pos));
            break;
        case(GPIO_INPUT_FLOATING):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b01, *(&CNF_Pos));
            break;
        case(GPIO_INPUT_PULL_DOWN):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b10, *(&CNF_Pos));
            CLEAR_BIT(GPIO->OUTDR, (0x1UL << *GPIO_Pin));
            break;
        case (GPIO_INPUT_PULL_UP):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b10, *(&CNF_Pos));
            SET_BIT(GPIO->OUTDR, (0x1UL << *GPIO_Pin));
            break;
        }
        break;
    }
}


/**
 ***************************************************************************************
 *  @breif Быстрая конфигурация GPIO
 *  Reference Manual/см. п.9.2 GPIO registers (стр. 171)
 *  Перед настройкой (GPIOs and AFIOs) нужно включить тактирование порта.
 *  @param  *GPIO - Порт GPIO(A, B, C, D, E)
 *  @param  GPIO_Pin - номер пина 0-15
 *  @param  Congiguration_mode: GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_ALTERNATIVE_FUNCTION_OUTPUT, GPIO_INPUT
 *  @param  Type: GPIO_OUTPUT_PUSH_PULL,
 *                GPIO_OUTPUT_OPEN_DRAIN,
 *                GPIO_INPUT_ANALOG,
 *                GPIO_INPUT_FLOATING,
 *                GPIO_INPUT_PULL_DOWN,
 *                GPIO_INPUT_PULL_UP
 *  @param  Speed: GPIO_SPEED_RESERVED,
 *                 GPIO_SPEED_10_MHZ,
 *                 GPIO_SPEED_2_MHZ,
 *                 GPIO_SPEED_50_MHZ
 ***************************************************************************************
 */

void RVMSIS_GPIO_init(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, uint8_t Configuration_mode, uint8_t Type, uint8_t Speed) {
    uint8_t CNF_Pos = 0;
    if (GPIO == GPIOA) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Запуск тактирования порта А
    } else if (GPIO == GPIOB) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOB); //Запуск тактирования порта B
    } else if (GPIO == GPIOC) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //Запуск тактирования порта C
    } else if (GPIO == GPIOD) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOD); //Запуск тактирования порта D
    } else if (GPIO == GPIOE) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOE); //Запуск тактирования порта E
    }


    RVMSIS_GPIO_SPEED_Set(GPIO, GPIO_Pin, Speed);

    if (GPIO_Pin < 8) {
        CNF_Pos = (GPIO_Pin * 4) + 2;
        RVMSIS_GPIO_Reg_Set(GPIO, (uint8_t*)&GPIO_Pin, Configuration_mode, Type, 0, &CNF_Pos);
    } else {
        GPIO_Pin = GPIO_Pin - 8;
        CNF_Pos = (GPIO_Pin * 4) + 2;
        RVMSIS_GPIO_Reg_Set(GPIO, (uint8_t*)&GPIO_Pin, Configuration_mode, Type, 1, &CNF_Pos);
    }
}


/**
 ***************************************************************************************
 *  @breif Blink PIN PC13 на выход в режиме Push-Pull
 ***************************************************************************************
 */
void RVMSIS_Blink_PC13(uint32_t ms) {
    GPIOC->BSHR = GPIO_BSHR_BR13;
    Delay_ms(ms);
    GPIOC->BSHR = GPIO_BSHR_BS13;
    Delay_ms(ms);
}

/**
 ***************************************************************************************
 *  @breif Настройка MCO c выходом на ножку PA8
 *  Перед настройкой (GPIOs and AFIOs) нужно включить тактирование порта.
 ***************************************************************************************
 */
void RVMSIS_PA8_MCO_init(void) {
    //Тактирование MCO должно быть настроено в регистре RCC
    MODIFY_REG(RCC->CFGR0, RCC_CFGR0_MCO, RCC_CFGR0_MCO_SYSCLK); //В качестве тактирования для MCO выбрал HSE. Будет 8 MHz.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Запустим тактирование порта A
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_MODE8, 0b11 << GPIO_CFGHR_MODE8_Pos); //Настройка GPIOA порта 8 на выход со максимальной скоростью в 50 MHz
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_CNF8, 0b10 << GPIO_CFGHR_CNF8_Pos); //Настройка GPIOA порта 8, как альтернативная функция, в режиме Push-Pull
}

/*================================ РЕЖИМ EXTI =======================================*/

/**
 ***************************************************************************************
 *  @breif Режим EXTI.
 ***************************************************************************************
 */
void RVMSIS_EXTI0_init(void) {
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //Запуск тактирования альтернативных функций
    MODIFY_REG(AFIO->EXTICR[0], AFIO_EXTICR1_EXTI0, AFIO_EXTICR1_EXTI0_PB); //AFIO_EXTICR1, EXTI0, выбран порт B.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOB); //Включим тактирование порта B
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_CNF0, 0b10 << GPIO_CFGLR_CNF0_Pos); //Настроим ножку PB0 в режим Input with pull-up / pull-down
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_MODE0, 0b00 << GPIO_CFGLR_MODE0_Pos); //Настройка в режим Input
    GPIOB->BSHR = GPIO_BSHR_BR0; //Подтяжка к земле
    SET_BIT(EXTI->INTENR, EXTI_INTENR_MR0); //Включаем прерывание EXTI0 по входному сигналу
    SET_BIT(EXTI->RTENR, EXTI_RTENR_TR0); //Реагирование по фронту вкл.
    SET_BIT(EXTI->FTENR, EXTI_FTENR_TR0); //Реагирование по спаду вкл.
    //SET_BIT(EXTI->SWIEVR, EXTI_SWIEVR_SWIEVR0);//Это софтварное включение прерывания
    //SET_BIT(EXTI->INTFR, EXTI_INTF_INTF0); //Команда выхода из прерывания
    NVIC_EnableIRQ(EXTI0_IRQn);

}

__WEAK void EXTI0_IRQHandler(void) {

    SET_BIT(EXTI->INTFR, EXTI_INTF_INTF0); //Команда выхода из прерывания

}

/*================================ Таймеры на примере TIM3 =======================================*/

void RVMSIS_TIM3_init(void) {
    /*Включим тактирование таймера (страница 48)*/
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_TIM3); //Запуск тактирования таймера 3
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //Запуск тактирования альтернативных функций

    CLEAR_BIT(TIM3->CTLR1, TIM_UDIS); //Генерировать событие Update
    CLEAR_BIT(TIM3->CTLR1, TIM_URS); //Генерировать прерывание
    CLEAR_BIT(TIM3->CTLR1, TIM_OPM); //One pulse mode off(Счетчик не останавливается при обновлении)
    CLEAR_BIT(TIM3->CTLR1, TIM_DIR); //Считаем вниз
    MODIFY_REG(TIM3->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos); //Выравнивание по краю
    SET_BIT(TIM3->CTLR1, TIM_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM3->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos); //Предделение выключено

    /*Настройка прерываний (Страница 409)*/
    SET_BIT(TIM3->DMAINTENR, TIM_UIE); //Update interrupt enable

    TIM3->PSC = 1 - 1;
    TIM3->ATRLR = 6000 - 1;

    NVIC_EnableIRQ(TIM3_IRQn); //Разрешить прерывания по таймеру 3
    SET_BIT(TIM3->CTLR1, TIM_CEN); //Запуск таймера
}

__WEAK void TIM3_IRQHandler(void) {
    if (READ_BIT(TIM3->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM3->INTFR, TIM_UIF); //Сбросим флаг прерывания
    }
}

void RVMSIS_TIM3_PWM_CHANNEL1_init(void) {
    /*Настройка ножки PA6 под ШИМ*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Включим тактирование порта А
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF6, 0b10 << GPIO_CFGLR_CNF6_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE6, 0b11 << GPIO_CFGLR_MODE6_Pos);

    /*Настройка шим(Канал 1)*/
    MODIFY_REG(TIM3->CHCTLR1, TIM_CC1S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC1FE); //Fast mode disable
    SET_BIT(TIM3->CHCTLR1, TIM_OC1PE); //Preload enable
    MODIFY_REG(TIM3->CHCTLR1, TIM_OC1M, 0b110 << TIM_OC1M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC1CE); //OC1Ref is not affected by the ETRF input

    /*Запуск ШИМ*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM3->CCER, TIM_CC1E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM3->CCER, TIM_CC1P); //OC1 active high.

    TIM3->CH1CVR = 0;
}

void RVMSIS_TIM3_PWM_CHANNEL2_init(void) {
    /*Настройка ножки PA7 под ШИМ*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Включим тактирование порта А
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF7, 0b10 << GPIO_CFGLR_CNF7_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE7, 0b11 << GPIO_CFGLR_MODE7_Pos);

    /*Настройка шим(Канал 2)*/
    /*Настройка шим(Канал 1)*/
    MODIFY_REG(TIM3->CHCTLR1, TIM_CC2S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC2FE); //Fast mode disable
    SET_BIT(TIM3->CHCTLR1, TIM_OC2PE); //Preload enable
    MODIFY_REG(TIM3->CHCTLR1, TIM_OC2M, 0b110 << TIM_OC2M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC2CE); //OC1Ref is not affected by the ETRF input

    /*Запуск ШИМ*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM3->CCER, TIM_CC2E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM3->CCER, TIM_CC2P); //OC1 active high.

    TIM3->CH2CVR = 5;
}

/*================================= НАСТРОЙКА ADC ============================================*/

/**
 ***************************************************************************************
 *  @breif Analog-to-digital converter (ADC)
 ***************************************************************************************
 */

volatile uint16_t ADC_RAW_Data[2] = { 0, }; //Массив, куда будем кидать данные с АЦП

void RVMSIS_ADC_DMA_init(void) {
    //Chapter 11 Direct Memory Access Control (DMA)
    SET_BIT(RCC->AHBPCENR, RCC_AHBPeriph_DMA1); //Включение тактирования DMA1
    DMA1_Channel1->PADDR = (uint32_t) &(ADC1->RDATAR); //Задаем адрес периферийного устройства
    DMA1_Channel1->MADDR = (uint32_t) ADC_RAW_Data; //Задаем адрес в памяти, куда будем кидать данные.
    DMA1_Channel1->CNTR = 2; //Настроим количество данных для передачи. После каждого периферийного события это значение будет уменьшаться.
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PL, 0b00 << DMA_CFGR1_PL_Pos); //Зададим приоритет канала на высокий
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_DIR); //Чтение будем осуществлять с периферии
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_CIRC); //Настроим DMA в Circular mode
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PSIZE, 0b01 << DMA_CFGR1_PSIZE_Pos); //Размер данных периферийного устройства 16 бит
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_MSIZE, 0b01 << DMA_CFGR1_MSIZE_Pos); //Размер данных в памяти 16 бит
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TCIE); //Включим прерывание по полной передаче
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_HTIE); //Отключим прерывание по половинной передаче
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TEIE); //Включим прерывание по ошибке передачи.
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_MINC); //Включим инкрементирование памяти
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_EN); //DMA ON
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_ADC1); //Включение тактирования ADC1.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Включение тактирования порта А.

    /*Настройка ножек PA0 и PA1 на аналоговый вход*/
    /*Pin PA0 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF0, 0b00 << GPIO_CFGLR_CNF0_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE0, 0b00 << GPIO_CFGLR_MODE0_Pos);

    /*Pin PA1 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b00 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b00 << GPIO_CFGLR_MODE1_Pos);

    //Прерывание по АЦП: регулярные каналы (вкл/выкл)
    CLEAR_BIT(ADC1->CTLR1, ADC_EOCIE);//EOC interrupt enabled/disabled. An interrupt is generated when the EOC bit is set

    //Прерывание по АЦП: analog watchdog (вкл/выкл)
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDIE);//Analog watchdog interrupt disabled

    //Прерывание по АЦП: инжектированные каналы (вкл/выкл)
    CLEAR_BIT(ADC1->CTLR1, ADC_JEOCIE);//JEOC interrupt disabled

    SET_BIT(ADC1->CTLR1, ADC_SCAN); //Scan mode enabled

    /* Примечание:
     * Прерывание EOC или JEOC генерируется только в конце преобразования последнего канала,
     * если установлен соответствующий бит EOCIE или JEOCIE.*/

    CLEAR_BIT(ADC1->CTLR1, ADC_AWDSGL); //Analog watchdog enabled on all channels
    CLEAR_BIT(ADC1->CTLR1, ADC_JAUTO); //Automatic injected group conversion disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_DISCEN); //Discontinuous mode on regular channels disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_JDISCEN); //Discontinuous mode on injected channels disabled
    MODIFY_REG(ADC1->CTLR1, ADC_DUALMOD, 0b0110 << ADC_DUALMOD_Pos); //0110: Regular simultaneous mode only
    CLEAR_BIT(ADC1->CTLR1, ADC_JAWDEN); //Analog watchdog disabled on injected channels
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDEN); //Analog watchdog disabled on regular channels

    //Control register 2 CTLR2
    SET_BIT(ADC1->CTLR2, ADC_ADON);//Запустить АЦП

    /* Примечание:
     * Если в этот же момент изменяется какой-либо другой бит в этом регистре,
     * кроме ADON, то конверсия не запускается.
     * Это сделано для предотвращения ошибочного преобразования.*/

    SET_BIT(ADC1->CTLR2, ADC_CONT); //Continuous conversion mode(непрерывные преобразования)
    SET_BIT(ADC1->CTLR2, ADC_CAL); //Enable calibration
    /*Примечание:
     * Этот бит устанавливается программой для запуска калибровки.
     * Он сбрасывается аппаратно после завершения калибровки.*/

    while (READ_BIT(ADC1->CTLR2, ADC_CAL));
    //Подождем окончания калибровки
    //Delay_ms(10);

    SET_BIT(ADC1->CTLR2, ADC_DMA);//DMA включен
    CLEAR_BIT(ADC1->CTLR2, ADC_ALIGN); //Выравнивание по правому краю
    MODIFY_REG(ADC1->CTLR2, ADC_EXTSEL, 0b111 << ADC_EXTSEL_Pos); //Запускать преобразование программно
    CLEAR_BIT(ADC1->CTLR2, ADC_EXTTRIG); //Conversion on external event disabled
    //SET_BIT(ADC1->CTLR2, ADC_SWSTART); //Начать преобразование
    //SET_BIT(ADC1->CTLR2, ADC_TSVREFE);//Temperature sensor and VREFINT channel enabled

    // 12.3.5 ADCx Sample Time Configuration Register 2 (ADCx_SAMPTR2) (x=1/2)
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP0, 0b111 << ADC_SMP0_Pos);//239.5 cycles
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP1, 0b111 << ADC_SMP1_Pos); //239.5 cycles
    //MODIFY_REG(ADC1->SAMPTR1, ADC_SMP17, 0b111 << ADC_SMP17_Pos); //239.5 cycles

    //12.3.9 ADCx Regular Channel Sequence Register1 (ADCx_RSQR1) (x=1/2)
    MODIFY_REG(ADC1->RSQR1, ADC_L, 0b0001 << ADC_L_Pos);//2 преобразования

    //12.3.11 ADCx Regular Channel Sequence Register 3 (ADCx_RSQR3) (x=1/2)
    MODIFY_REG(ADC1->RSQR3, ADC_SQ1, 0 << ADC_SQ1_Pos);
    MODIFY_REG(ADC1->RSQR3, ADC_SQ2, 1 << ADC_SQ2_Pos);
    //NVIC_EnableIRQ(ADC1_2_IRQn); //Разрешить прерывания по АЦП

    //SET_BIT(ADC1->CTLR2, ADC_SWSTART); //Начать преобразование. Не нужно запускать, если Continuous conversion mode(непрерывные преобразования) включен

}

__WEAK void ADC1_2_IRQHandler(void) {
    if (READ_BIT(ADC1->STATR, ADC_EOC)) {
        ADC1->IDATAR1; //Читаем канал, чтоб сбросить флаг
    }

}
__WEAK void DMA1_Channel1_IRQHandler(void) {
    if (READ_BIT(DMA1->INTFR, DMA_TCIF1)) {
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //Сбросим глобальный флаг.
        /*Здесь можно писать код*/

    } else if (READ_BIT(DMA1->INTFR, DMA_TEIF1)) {
        /*Здесь можно сделать какой-то обработчик ошибок*/
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //Сбросим глобальный флаг.
    }
}

/*================================= НАСТРОЙКА USART ============================================*/

/**
 ***************************************************************************************
 *  @breif Universal synchronous asynchronous receiver transmitter (USART)
 ***************************************************************************************
 */

struct USART_name husart1; //Объявляем структуру по USART.(см. ch32v203x_RVMSIS.h)
struct USART_name husart2; //Объявляем структуру по USART.(см. ch32v203x_RVMSIS.h)

/**
 ******************************************************************************
 *  @breif Настройка USART1. Параметры 9600 8 N 1
 ******************************************************************************
 */

void RVMSIS_USART1_Init(void) {

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Включение тактирование порта А
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //Включение альтернативных функций

    //Для конфигурирование ножек UART для Full Duplex нужно использовать Alternate function push-pull(См. п.п. 9.1.11 GPIO configurations for device peripherals стр.111 Reference Manual)
    //Tx - Alternative Function output Push-pull(Maximum output speed 50 MHz)
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_CNF9, 0b10 << GPIO_CFGHR_CNF9_Pos);
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_MODE9, 0b11 << GPIO_CFGHR_MODE9_Pos);
    //Rx - Input floating
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_CNF10, 0b1 << GPIO_CFGHR_CNF10_Pos);
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_MODE10, 0b00 << GPIO_CFGHR_MODE10_Pos);

    //Запустим тактирование USART1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_USART1);

    /*Расчет Fractional baud rate generation
     есть формула:
     Tx/Rx baud = fCK/(16*USARTDIV)
     где fCK - Input clock to the peripheral (PCLK1 for USART2, 3, 4, 5 or PCLK2 for USART1)
     в нашем случае fCK = 72000000
     допустим нам нужна скорость 9600
     9600 = 72000000/(16*USARTDIV)
     Тогда USARTDIV = 72000000/9600*16 = 468.75
     DIV_Mantissa в данном случае будет 468, что есть 0x1D4
     DIV_Fraction будет, как 0.75*16 = 12, что есть 0xC
     Тогда весь регистр USART->BRR для скорости 9600 будет выглядеть, как 0x1D4C.
     для примера еще разберем скорость 115200:
     115200 = 72000000/(16*USARTDIV)
     Тогда USARTDIV = 72000000/115200*16 = 39.0625
     DIV_Mantissa в данном случае будет 39, что есть 0x27
     DIV_Fraction будет, как 0.0625*16 = 1, что есть 0x1
     Тогда весь регистр USART->BRR для скорости 115200 будет выглядеть, как 0x271.
     */

    MODIFY_REG(USART1->BRR, USART_BRR_DIV_Mantissa, 0x1D4 << USART_BRR_DIV_Mantissa_Pos);
    MODIFY_REG(USART1->BRR, USART_BRR_DIV_Fraction, 0xC << USART_BRR_DIV_Mantissa_Pos);

    //18.10.4 USART Control Register1 (USARTx_CTLR1) (x=1/2/3/4/5/6/7/8)
    SET_BIT(USART1->CTLR1, USART_CTLR1_UE);//USART enable
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_M); //Word lenght 1 Start bit, 8 Data bits, n Stop bit
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_WAKE); //Wake up idle Line
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_PCE); //Partity control disabled
    //настройка прерываний
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_PEIE);//partity error interrupt disabled
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_TXEIE); //TXE interrupt is inhibited
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_TCIE); //Transmission complete interrupt disabled
    SET_BIT(USART1->CTLR1, USART_CTLR1_RXNEIE); //Прерывание по приему данных включено
    SET_BIT(USART1->CTLR1, USART_CTLR1_IDLEIE); //Прерывание по флагу IDLE включено
    SET_BIT(USART1->CTLR1, USART_CTLR1_TE); //Transmitter is enabled
    SET_BIT(USART1->CTLR1, USART_CTLR1_RE); //Receiver is enabled and begins searching for a start bit
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_RWU);
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_SBK);

    //Остальную настройку, не касающуюся стандартного USART, мы пока трогать не будем, но на всякий случай обнулим
    //18.10.5 USART Control Register2 (USARTx_CTLR2) (x=1/2/3/4/5/6/7/8)
    USART1->CTLR2 = 0;
    CLEAR_BIT(USART1->CTLR2, USART_CTLR2_STOP); //1 стоп бит.
    //18.10.6 USART Control Register 3 (USARTx_CTLR3) (x=1/2/3/4/5/6/7/8)
    USART1->CTLR3 = 0;
    //18.10.7 USART Guard Time and Prescaler Register (USARTx_GPR) (x=1/2/3/4/5/6/7/8)
    USART1->GPR = 0;

    NVIC_EnableIRQ(USART1_IRQn); //Включим прерывания по USART1
}

/**
 ******************************************************************************
 *  @breif Настройка USART2. Параметры 9600 8 N 1
 ******************************************************************************
 */

void RVMSIS_USART2_Init(void) {

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Включение тактирование порта А
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //Включение альтернативных функций

    //Для конфигурирование ножек UART для Full Duplex нужно использовать Alternate function push-pull(См. п.п. 9.1.11 GPIO configurations for device peripherals стр.111 Reference Manual)
    //Tx - Alternative Function output Push-pull(Maximum output speed 50 MHz)
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF2, 0b10 << GPIO_CFGLR_CNF2_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE2, 0b11 << GPIO_CFGLR_MODE2_Pos);
    //Rx - Input floating
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF3, 0b1 << GPIO_CFGLR_CNF3_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE3, 0b00 << GPIO_CFGLR_MODE3_Pos);

    //Запустим тактирование USART2
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_USART2);

    /*Расчет Fractional baud rate generation
     есть формула:
     Tx/Rx baud = fCK/(16*USARTDIV)
     где fCK - Input clock to the peripheral (PCLK1 for USART2, 3, 4, 5 or PCLK2 for USART1)
     в нашем случае fCK = 36000000
     допустим нам нужна скорость 9600
     9600 = 36000000/(16*USARTDIV)
     Тогда USARTDIV = 36000000/9600*16 = 234.375
     DIV_Mantissa в данном случае будет 234, что есть 0xEA
     DIV_Fraction будет, как 0.375*16 = 6, что есть 0x6
     Тогда весь регистр USART->BRR для скорости 9600 будет выглядеть, как 0xEA6.
     для примера еще разберем скорость 115200: (Неточность по скорости будет 0.15%. Не рекомендуется)
     115200 = 36000000/(16*USARTDIV)
     Тогда USARTDIV = 36000000/115200*16 = 19.53125
     DIV_Mantissa в данном случае будет 19, что есть 0x13
     DIV_Fraction будет, как 0.53125*16 = 8, что есть 0x8
     Тогда весь регистр USART->BRR для скорости 115200 будет выглядеть, как 0x138.
     */

    MODIFY_REG(USART2->BRR, USART_BRR_DIV_Mantissa, 0xEA << USART_BRR_DIV_Mantissa_Pos);
    MODIFY_REG(USART2->BRR, USART_BRR_DIV_Fraction, 0x6 << USART_BRR_DIV_Fraction_Pos);

    //18.10.4 USART Control Register1 (USARTx_CTLR1) (x=1/2/3/4/5/6/7/8)
    SET_BIT(USART2->CTLR1, USART_CTLR1_UE);//USART enable
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_M); //Word lenght 1 Start bit, 8 Data bits, n Stop bit
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_WAKE); //Wake up idle Line
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_PCE); //Partity control disabled
    //настройка прерываний
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_PEIE);//partity error interrupt disabled
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_TXEIE); //TXE interrupt is inhibited
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_TCIE); //Transmission complete interrupt disabled
    SET_BIT(USART2->CTLR1, USART_CTLR1_RXNEIE); //Прерывание по приему данных включено
    SET_BIT(USART2->CTLR1, USART_CTLR1_IDLEIE); //Прерывание по флагу IDLE включено
    SET_BIT(USART2->CTLR1, USART_CTLR1_TE); //Transmitter is enabled
    SET_BIT(USART2->CTLR1, USART_CTLR1_RE); //Receiver is enabled and begins searching for a start bit
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_RWU);
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_SBK);

    //Остальную настройку, не касающуюся стандартного USART, мы пока трогать не будем, но на всякий случай обнулим
    //18.10.5 USART Control Register2 (USARTx_CTLR2) (x=1/2/3/4/5/6/7/8)
    USART2->CTLR2 = 0;
    CLEAR_BIT(USART2->CTLR2, USART_CTLR2_STOP); //1 стоп бит.
    //18.10.6 USART Control Register 3 (USARTx_CTLR3) (x=1/2/3/4/5/6/7/8)
    USART2->CTLR3 = 0;
    //18.10.7 USART Guard Time and Prescaler Register (USARTx_GPR) (x=1/2/3/4/5/6/7/8)
    USART2->GPR = 0;

    NVIC_EnableIRQ(USART2_IRQn); //Включим прерывания по USART2
}

/**
 ******************************************************************************
 *  @breif Прерывание по USART1
 ******************************************************************************
 */

__WEAK void USART1_IRQHandler(void) {
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

/**
 ******************************************************************************
 *  @breif Прерывание по USART1
 ******************************************************************************
 */

__WEAK void USART2_IRQHandler(void) {
    if (READ_BIT(USART2->STATR, USART_STATR_RXNE)) {
        //Если пришли данные по USART
        husart2.rx_buffer[husart2.rx_counter] = USART2->DATAR; //Считаем данные в соответствующую ячейку в rx_buffer
        husart2.rx_counter++; //Увеличим счетчик принятых байт на 1
    }
    if (READ_BIT(USART2->STATR, USART_STATR_IDLE)) {
        //Если прилетел флаг IDLE
        USART2->DATAR; //Сбросим флаг IDLE
        husart2.rx_len = husart2.rx_counter; //Узнаем, сколько байт получили
        husart2.rx_counter = 0; //сбросим счетчик приходящих данных
    }
}

/**
 ******************************************************************************
 *  @breif Функция отправки данных по USART
 *  @param  *USART - USART, с которого будут отправляться данные
 *  @param  *data - данные, которые будем отправлять
 *  @param  Size - сколько байт требуется передать
 ******************************************************************************
 */

bool RVMSIS_USART_Transmit(USART_TypeDef* USART, uint8_t* data, uint16_t Size, uint32_t Timeout_ms) {
    for (uint16_t i = 0; i < Size; i++) {
        Timeout_counter_ms = Timeout_ms;
        //Ждем, пока линия не освободится
        while (READ_BIT(USART->STATR, USART_STATR_TXE) == 0) {
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        USART->DATAR = *data++; //Кидаем данные
    }
    return true;
}

/*================================= НАСТРОЙКА I2C ============================================*/

/**
 ***************************************************************************************
 *  @breif Inter-integrated circuit (I2C) interface
 ***************************************************************************************
 */

void RVMSIS_I2C_Reset(void) {
    //Сброс настроек I2C
    //19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    SET_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST); //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST) == 0);
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST); //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST));
    /* Примечание: Этот бит можно использовать для повторной инициализации
     * периферийного устройства после ошибки или заблокированного состояния.
     * Например, если бит BUSY установлен и остается заблокированным из-за сбоя на шине,
     * бит SWRST можно использовать для выхода из этого состояния.*/
}

/**
 *************************************************************************************
 *  @breif Функция инициализации шины I2C1. Sm.
 *************************************************************************************
 */

void RVMSIS_I2C1_Init(void) {
    //Настройки тактирования
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOB); //Запуск тактирование порта B
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //Запуск тактирования альтернативных функций
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_I2C1); //Запуск тактирования I2C1

    //Настройки ножек SDA и SCL
    //PB7 SDA (I2C Data I/O) Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_CNF7, 0b11 << GPIO_CFGLR_CNF7_Pos);//Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_MODE7, 0b11 << GPIO_CFGLR_MODE7_Pos); //Maximum output speed 50 MHz
    //PB6 SCL (I2C clock) Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_CNF6, 0b11 << GPIO_CFGLR_CNF6_Pos);//Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_MODE6, 0b11 << GPIO_CFGLR_MODE6_Pos); //Maximum output speed 50 MHz

    //19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    RVMSIS_I2C_Reset();

    /*Это все для инита не нужно. После сброса итак будет в 0. */
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ALERT); //Releases SMBA pin high.Alert Response Address Header followed by NACK
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_PEC); //No PEC transfer
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ACK); //No acknowledge returned
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_STOP); //No Stop generation
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_START); //No Start generation
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_NOSTRETCH); //Clock stretching enabled
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ENGC); //General call disabled. Address 00h is NACKed.
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ENPEC); //PEC calculation disabled
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ENARP); //ARP disable
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SMBTYPE); //SMBus Device
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SMBUS); //I2C mode

    //19.12.2 I2C Control Register 2 (I2Cx_CTLR2) (x=1/2)
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_LAST);//Next DMA EOT is not the last transfer
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_DMAEN); //DMA requests disabled
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_ITBUFEN); //TxE = 1 or RxNE = 1 does not generate any interrupt.
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_ITEVTEN); //Event interrupt disabled
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_ITERREN); //Error interrupt disabled
    MODIFY_REG(I2C1->CTLR2, I2C_CTLR2_FREQ, 36 << I2C_CTLR2_FREQ_Pos); //f PCLK1 = 36 Мгц

    //19.12.3 I2C Address Register 1 (I2Cx_OADDR1) (x=1/2)
    I2C1->OADDR1 = 0;
    //19.12.4 I2C Address Register2 (I2Cx_OADDR2) (x=1/2)
    I2C1->OADDR2 = 0;

    //19.12.8 I2C Clock Register (I2Cx_CKCFGR) (x=1/2)
    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS);//Standard mode I2C
    //SET_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS); //Fast mode I2C

    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_DUTY);//Fm mode tlow/thigh = 2
    //SET_BIT(I2C1->CCR, I2C_CCR_DUTY); //Fm mode tlow/thigh = 16/9 (see CCR)

    //Расчет CCR. Смотри примеры расчета
    MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 180 << I2C_CKCFGR_CCR_Pos);//для Sm mode
    //MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 30 << I2C_CKCFGR_CCR_Pos); //для Fm mode. DUTY 0.
    //MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 4 << I2C_CKCFGR_CCR_Pos); //для Fm mode. DUTY 1.

    //19.12.9 I2C Rise Time Register (I2Cx_RTR) (x=1/2)
    MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 37 << I2C_RTR_TRISE_Pos);//для Sm mode
    //MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 12 << I2C_RTR_TRISE_Pos); //для Fm mode

    SET_BIT(I2C1->CTLR1, I2C_CTLR1_PE); //I2C1 enable
}

/**
 *************************************************************************************
 *  @breif Функция сканирования устройства по заданному 7-битному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства
 *  @retval  Возвращает статус true - если устройство по заданному адресу отозвалось,
 *           false - если устройство по заданному адресу не отвечает
 *************************************************************************************
 */
bool RVMSIS_I2C_Adress_Device_Scan(I2C_TypeDef* I2C, uint8_t Adress_Device, uint32_t Timeout_ms) {

    /*-------------------Проверка занятости шины-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //Если шина занята

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //Если линия на самом деле свободна, а BUSY висит
            RVMSIS_I2C_Reset(); //ресет
            RVMSIS_I2C1_Init(); //повторная инициализация
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //Если стоит статус, что мы в мастере
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Отправим сигнал STOP
        }

        if (I2C->CTLR1 != 1) {
            //Если в CR1 что-то лишнее, то перезагрузим I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------Проверка занятости шины-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //Отправляем сигнал START

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //Ожидаем до момента, пока не сработает Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //ВНИМАНИЕ!
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью данных в регистр DR или когда PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //Адрес + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //Ждем, пока адрес отзовется

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //Если устройство отозвалось
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Отправляем сигнал STOP
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
        I2C->STAR1;
        I2C->STAR2;
        return true;
    } else {
        //Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Отправляем сигнал STOP
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //Сбрасываем бит AF
        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif Функция передачи данных по I2C
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства
 *  @param  *data - Данные, которые будем отправлять
 *  @param  Size_data - Размер, сколько байт будем отправлять.
 *  @retval  Возвращает статус отправки данных. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Transmit(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------Проверка занятости шины-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //Если шина занята

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //Если линия на самом деле свободна, а BUSY висит
            RVMSIS_I2C_Reset(); //ресет
            RVMSIS_I2C1_Init(); //повторная инициализация
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //Если стоит статус, что мы в мастере
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Отправим сигнал STOP
        }

        if (I2C->CTLR1 != 1) {
            //Если в CR1 что-то лишнее, то перезагрузим I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------Проверка занятости шины-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //Стартуем.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //Ожидаем до момента, пока не сработает Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //ВНИМАНИЕ!
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //Адрес + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //Ждем, пока адрес отзовется

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //Если устройство отозвалось, сбросим бит ADDR
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*Отправим данные*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i); //Запись байта
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //Ждем, пока данные загрузятся в регистр сдвига.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//Останавливаем
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//Сбрасываем бит AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Останавливаем

        return true;

    } else {
        //Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Останавливаем
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //Сбрасываем бит AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif Функция приема данных по I2C
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства
 *  @param  *data - Куда будем записывать принятые данные
 *  @param  Size_data - Размер, сколько байт будем принимать.
 *  @retval  Возвращает статус приема данных. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Receive(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------Проверка занятости шины-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //Если шина занята

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //Если линия на самом деле свободна, а BUSY висит
            RVMSIS_I2C_Reset(); //ресет
            RVMSIS_I2C1_Init(); //повторная инициализация
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //Если стоит статус, что мы в мастере
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Отправим сигнал STOP
        }

        if (I2C->CTLR1 != 1) {
            //Если в CR1 что-то лишнее, то перезагрузим I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------Проверка занятости шины-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //Стартуем.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //Ожидаем до момента, пока не сработает Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //ВНИМАНИЕ!
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1 | 1); //Адрес + команда Read

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //Ждем, пока адрес отзовется

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //Если устройство отозвалось, сбросим бит ADDR
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*Прочтем данные*/
        for (uint16_t i = 0; i < Size_data; i++) {
            if (i < Size_data - 1) {
                SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //Если мы хотим принять следующий байт, то отправляем ACK

                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    //Ожидаем, пока в сдвиговом регистре появятся данные
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }

                *(data + i) = I2C->DATAR; //Чтение байта
            } else {
                CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //Если мы знаем, что следующий принятый байт будет последним, то отправим NACK

                SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Останавливаем
                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    //Ожидаем, пока в сдвиговом регистре появятся данные
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }
                *(data + i) = I2C->DATAR; //Чтение байта
            }
        }
        return true;

    } else {
        //Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Останавливаем
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //Сбрасываем бит AF
        return false;
    }

}

/**
 **************************************************************************************************
 *  @breif Функция записи в память по указанному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства
 *  @param  Adress_data - Адрес в памяти, куда будем записывать данные
 *  @param  Size_adress - Размер адреса в байтах. Пример: 1 - 8 битный адрес. 2 - 16 битный адрес.
 *  @param  *data - Данные, которые будем записывать
 *  @param  Size_data - Размер, сколько байт будем записывать.
 *  @retval  Возвращает статус записи. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemWrite(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------Проверка занятости шины-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //Если шина занята

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //Если линия на самом деле свободна, а BUSY висит
            RVMSIS_I2C_Reset(); //ресет
            RVMSIS_I2C1_Init(); //повторная инициализация
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //Если стоит статус, что мы в мастере
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Отправим сигнал STOP
        }

        if (I2C->CTLR1 != 1) {
            //Если в CR1 что-то лишнее, то перезагрузим I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------Проверка занятости шины-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //Стартуем.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //Ожидаем до момента, пока не сработает Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //ВНИМАНИЕ!
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //Адрес + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //Ждем, пока адрес отзовется

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //Если устройство отозвалось, сбросим бит ADDR
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*Отправим адрес памяти*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*) &Adress_data + (Size_adress - 1 - i)); //Запись байта
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //Ждем, пока данные загрузятся в регистр сдвига.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//Останавливаем
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//Сбрасываем бит AF
                    return false;
                }
            }
        }

        /*Будем записывать данные в ячейку памяти, начиная с указанного адреса*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i); //Запись байта
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //Ждем, пока данные загрузятся в регистр сдвига.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//Останавливаем
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//Сбрасываем бит AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Останавливаем

        return true;

    } else {
        //Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Останавливаем
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //Сбрасываем бит AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif Функция чтения из памяти по указанному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства
 *  @param  Adress_data - Адрес в памяти, откуда будем считывать данные
 *  @param  Size_adress - Размер адреса в байтах. Пример: 1 - 8 битный адрес. 2 - 16 битный адрес.
 *  @param  *data - Данные, в которые будем записывать считанную информацию.
 *  @param  Size_data - Размер, сколько байт будем считывать.
 *  @retval  Возвращает статус считывания. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemRead(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------Проверка занятости шины-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //Если шина занята

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //Если линия на самом деле свободна, а BUSY висит
            RVMSIS_I2C_Reset(); //ресет
            RVMSIS_I2C1_Init(); //повторная инициализация
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //Если стоит статус, что мы в мастере
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Отправим сигнал STOP
        }

        if (I2C->CTLR1 != 1) {
            //Если в CR1 что-то лишнее, то перезагрузим I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------Проверка занятости шины-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //Стартуем.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //Ожидаем до момента, пока не сработает Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //ВНИМАНИЕ!
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //Адрес + команда Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //Ждем, пока адрес отзовется

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //Если устройство отозвалось, сбросим бит ADDR
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*Отправим адрес памяти*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*) &Adress_data + (Size_adress - 1 - i)); //Запись байта
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //Ждем, пока данные загрузятся в регистр сдвига.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//Останавливаем
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//Сбрасываем бит AF
                    return false;
                }
            }
        }

        //Повторный старт
        SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //Стартуем.

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
            //Ожидаем до момента, пока не сработает Start condition generated

            if (!Timeout_counter_ms) {
                return false;
            }

        }
        //ВНИМАНИЕ!
        /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
        I2C->STAR1;
        I2C->DATAR = (Adress_Device << 1 | 1); //Адрес + команда Read

        Timeout_counter_ms = Timeout_ms;
        while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
            //Ждем, пока адрес отзовется

            if (!Timeout_counter_ms) {
                return false;
            }

        }

        if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
            //Если устройство отозвалось, сбросим бит ADDR
            /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
            I2C->STAR1;
            I2C->STAR2;

            /*Прочтем данные, начиная с указанного адреса*/
            for (uint16_t i = 0; i < Size_data; i++) {
                if (i < Size_data - 1) {
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //Если мы хотим принять следующий байт, то отправляем ACK
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    *(data + i) = I2C->DATAR; //Чтение байта
                } else {
                    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //Если мы знаем, что следующий принятый байт будет последним, то отправим NACK

                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Останавливаем
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    //Подождем, пока сдвиговый регистр пополнится новым байтом данных
                    *(data + i) = I2C->DATAR; //Чтение байта
                }
            }
            return true;

        } else {
            //Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Останавливаем
            CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //Сбрасываем бит AF
            return false;
        }

    } else {
        //Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //Останавливаем
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //Сбрасываем бит AF
        return false;
    }
}

/*================================= НАСТРОЙКА SPI ============================================*/

/**
***************************************************************************************
*  @breif Serial peripheral interface (SPI)
***************************************************************************************
*/

void RVMSIS_SPI1_init(void) {
    /*Настройка GPIO*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //Включение альтернативных функций
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_SPI1); //Включение тактирования SPI1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Включение тактирования порта А
    /*Какие ножки:*/
    //PA4 - NSS
    //PA5 - SCK
    //PA6 - MISO
    //PA7 - MOSI
    //Мы будем настраивать SPI в режим Master
    /*
     * SPIx_SCK  Master - Alternate function push-pull
     * SPIx_MOSI:
     *             Full duplex / master - Alternate function push-pull
     *             Simplex bidirectional data wire / master - Alternate function push-pull
     * SPIx_MISO:
     *             Full duplex / master - Input floating / Input pull-up
     *
     * SPIx_NSS:
     *             Hardware master /slave - Input floating/ Input pull-up / Input pull-down
     *             Hardware master/ NSS output enabled - Alternate function push-pull
     *             Software - Not used. Can be used as a GPIO
     */
     //Настроим сами ножки уже после инициализации SPI, чтоб при старте не было лишних ногодерганий.

    //20.4.1 SPI Control Register 1 (SPIx_CTLR1) (x=1/2/3)
    MODIFY_REG(SPI1->CTLR1, SPI_CTLR1_BR, 0b011 << SPI_CTLR1_BR_Pos); //fPCLK/4. 72000000/32 = 2.22 MBits/s
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_CPOL); //Полярность
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_CPHA); //Фаза
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_DFF); //0: 8-bit data frame format is selected for transmission/reception
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_LSBFIRST); //0: MSB transmitted first
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_SSM); //1: Software slave management enabled
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_SSI); //1: Software slave management enabled
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_MSTR); //1: Master configuration
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_BIDIMODE); //0: 2-line unidirectional data mode selected
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_RXONLY); //0: Full duplex (Transmit and receive)

    SET_BIT(SPI1->CTLR1, SPI_CTLR1_SPE); //Включим SPI

    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_CRCEN); //0: CRC calculation disabled
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_CRCNEXT); // 0: Data phase (no CRC phase)



    //20.4.2 SPI Control Register 2 (SPIx_CTLR2) (x=1/2/3)
    CLEAR_BIT(SPI1->CTLR2, SPI_CTLR2_RXDMAEN); //0: Rx buffer DMA disabled
    CLEAR_BIT(SPI1->CTLR2, SPI_CTLR2_TXDMAEN); //0: Tx buffer DMA disabled
    CLEAR_BIT(SPI1->CTLR2, SPI_CTLR2_SSOE); //0: SS output is disabled in master mode and the cell can work in multimaster configuration
    CLEAR_BIT(SPI1->CTLR2, SPI_CTLR2_ERRIE); //0: Error interrupt is masked
    CLEAR_BIT(SPI1->CTLR2, SPI_CTLR2_RXNEIE); //0: RXNE interrupt masked
    CLEAR_BIT(SPI1->CTLR2, SPI_CTLR2_TXEIE); //0: TXE interrupt masked

    //20.4.8 SPI_I2S Configuration Register (SPI_I2S_CFGR) (x=1/2/3)
    CLEAR_BIT(SPI1->I2SCFGR, SPI_I2SCFGR_I2SMOD); //т.к. на F103C6T6 нет I2S, его вырезали, а регистр оставили, нужно просто обнулить данный регистр. Тем самым включим режим SPI mode.


    //SCK - PA5:
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE5, 0b11 << GPIO_CFGLR_MODE5_Pos); //Maximum output speed 50 MHz
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF5, 0b10 << GPIO_CFGLR_CNF5_Pos); //Alternate Function output Push-pull
    //MISO - PA6:
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE6, 0b00 << GPIO_CFGLR_MODE6_Pos); //Reserved
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF6, 0b1 << GPIO_CFGLR_CNF6_Pos); //Input pull-up
    SET_BIT(GPIOA->OUTDR, GPIO_OUTDR_ODR6); //Pull-Up
    //MOSI - PA7:
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE7, 0b11 << GPIO_CFGLR_MODE7_Pos); //Maximum output speed 50 MHz
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF7, 0b10 << GPIO_CFGLR_CNF7_Pos); //Alternate Function output Push-pull
}

/**
 **************************************************************************************************
 *  @breif Функция передачи данных по шине SPI
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, которые будем передавать.
 *  @param  Size_data - Размер, сколько байт будем передавать.
 *  @retval  Возвращает статус передачи. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Transmit_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    //(см. Reference Manual стр. 712 Transmit-only procedure (BIDIMODE=0 RXONLY=0))
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //Проверим занятость шины
        SPI->DATAR = *(data); //Запишем первый элемент данных для отправки в регистр SPI_DR
        //(При этом очищается бит TXE)

        for (uint16_t i = 1; i < Size_data; i++) {
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
                //Ждем, пока буфер на передачу не освободится
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            SPI->DATAR = *(data + i); //Запишем следующий элемент данных.
        }
        Timeout_counter_ms = Timeout_ms;
        while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
            //После записи последнего элемента данных в регистр SPI_DR,
            //подождем, пока TXE станет равным 1.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //Затем подождем, пока BSY станет равным 0.
            //Это указывает на то, что передача последних данных завершена.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
    //Примечание:
    //После передачи двух элементов данных в режиме "transmit-only mode" в регистре SPI_SR устанавливается флаг OVR, так как принятые данные никогда не считываются.
}

/**
 **************************************************************************************************
 *  @breif Функция передачи данных по шине SPI
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, которые будем передавать.
 *  @param  Size_data - сколько 16 - битных данных хотим передать. Т.е. 1 = 2 байта.
 *  @retval  Возвращает статус передачи. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Transmit_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    //(см. Reference Manual стр. 712 Transmit-only procedure (BIDIMODE=0 RXONLY=0))
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //Проверим занятость шины
        SPI->DATAR = *(data); //Запишем первый элемент данных для отправки в регистр SPI_DR
        //(При этом очищается бит TXE)

        for (uint16_t i = 1; i < Size_data; i++) {
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
                //Ждем, пока буфер на передачу не освободится
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            SPI->DATAR = *(data + i); //Запишем следующий элемент данных.
        }
        Timeout_counter_ms = Timeout_ms;
        while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
            //После записи последнего элемента данных в регистр SPI_DR,
            //подождем, пока TXE станет равным 1.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //Затем подождем, пока BSY станет равным 0.
            //Это указывает на то, что передача последних данных завершена.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
    //Примечание:
    //После передачи двух элементов данных в режиме "transmit-only mode" в регистре SPI_SR устанавливается флаг OVR, так как принятые данные никогда не считываются.
}

/**
 **************************************************************************************************
 *  @breif Функция приема данных по шине SPI
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, куда будем записывать принятые данные.
 *  @param  Size_data - Размер, сколько байт хотим принять.
 *  @retval  Возвращает статус приема. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Receive_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //Проверим занятость шины

        if (READ_BIT(SPI->STATR, SPI_STATR_OVR) || READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
            //Т.к. мы можем принимать данные в любой момент, например после режима "transmit-only mode"
            //то следует проверить статусы OVR и RXNE. Если хотя бы один из них установлен,
            //то сбросим их при помощи чтения регистра DR.
            SPI->DATAR;
        }

        //Начнем прием данных
        for (uint16_t i = 0; i < Size_data; i++) {
            SPI->DATAR = 0; //Запустим тактирование, чтоб считать 8 бит
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
                //Ждем, пока буфер на прием не заполнится
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            *(data + i) = SPI->DATAR; //Считываем данные
        }

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //Затем подождем, пока BSY станет равным 0.
            //Это указывает на то, что прием последних данных завершен.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif Функция приема данных по шине SPI
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, куда будем записывать принятые данные.
 *  @param  Size_data - Размер, сколько 16 - битных данных хотим принять. Т.е. 1 = 2 байта.
 *  @retval  Возвращает статус приема. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Receive_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //Проверим занятость шины

        if (READ_BIT(SPI->STATR, SPI_STATR_OVR) || READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
            //Т.к. мы можем принимать данные в любой момент, например после режима "transmit-only mode"
            //то следует проверить статусы OVR и RXNE. Если хотя бы один из них установлен,
            //то сбросим их при помощи чтения регистра DR.
            SPI->DATAR;
        }

        //Начнем прием данных
        for (uint16_t i = 0; i < Size_data; i++) {
            SPI->DATAR = 0; //Запустим тактирование, чтоб считать 16 бит
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
                //Ждем, пока буфер на прием не заполнится
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            *(data + i) = SPI->DATAR; //Считываем данные
        }

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //Затем подождем, пока BSY станет равным 0.
            //Это указывает на то, что прием последних данных завершен.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
}

/*================================= Работа с FLASH ============================================*/

/*Пример структуры для работы с FLASH*/
/*
typedef struct __attribute__((packed)) {
    uint8_t Data1;
    uint16_t Data2;
    uint32_t Data3;
    float Data4;
} Flash_struct;
Flash_struct Flash_data_CH32;
Flash_struct Flash_data_CH32_read;*/

/*Пример работы с FLASH*/
/*
Flash_data_CH32.Data1 = 0x23;
Flash_data_CH32.Data2 = 0x4567;
Flash_data_CH32.Data3 = 0x89101112;
Flash_data_CH32.Data4 = 3.14159f;
FLASH_Page_write(0x0800F000, (uint8_t*)&Flash_data_CH32, sizeof(Flash_data_CH32));
FLASH_Read_data(0x0800F000, (uint8_t*)&Flash_data_CH32_read, sizeof(Flash_data_CH32_read));
*/


/**
 ***************************************************************************************
 *  @breif Разблокировка FLASH
 *  Чтоб разблокировать FLASH, нужно в FLASH->KEYR ввести поочередно 2 ключа.
 *  32.4.1 FPEC Key Register (FLASH_KEYR)
 ***************************************************************************************
 */
void RVMSIS_FLASH_Unlock(void) {
    FLASH->KEYR = 0x45670123; //KEY1
    FLASH->KEYR = 0xCDEF89AB; //KEY2
}

/**
 ***************************************************************************************
 *  @breif Блокировка FLASH
 *  Чтоб заблокировать FLASH, нужно в FLASH->CR, FLASH_CR_LOCK выставить 1
 *  32.4.4 Control Register (FLASH_CTLR)
 ***************************************************************************************
 */
void RVMSIS_FLASH_Lock(void) {
    SET_BIT(FLASH->CTLR, FLASH_CTLR_LOCK);
}

/**
 ***************************************************************************************
 *  @breif Стирание страницы во FLASH
 ***************************************************************************************
 */
void RVMSIS_FLASH_Page_erase(uint16_t Adress) {
    //Если память заблокирована, то разблокируем ее
    if (READ_BIT(FLASH->CTLR, FLASH_CTLR_LOCK)) {
        RVMSIS_FLASH_Unlock();
    }
    SET_BIT(FLASH->CTLR, FLASH_CTLR_PER); //Выберем функцию очистки страницы
    FLASH->ADDR = Adress; //Укажем адрес
    SET_BIT(FLASH->CTLR, FLASH_CTLR_STRT); //Запустим стирание
    while (READ_BIT(FLASH->STATR, FLASH_STATR_BSY)) ; //Ожидаем, пока пройдет стирание
    while (READ_BIT(FLASH->STATR, FLASH_STATR_EOP) == 0) ; //Дождемся флага завершения программы
    CLEAR_BIT(FLASH->CTLR, FLASH_CTLR_PER); //Выключим функцию.
    RVMSIS_FLASH_Lock(); //Заблокируем память
}

/**
 ***************************************************************************************
 *  @breif Запись страницы во FLASH
 *  @param  Adress - Адрес во flash
 *  @param  *Data - Данные, которые будем писать во flash
 *  @param  Size - Размер даных, которые будем писать во flash
 ***************************************************************************************
 */
void RVMSIS_FLASH_Page_write(uint32_t Adress, uint8_t *Data, uint16_t Size) {
    //Проверка размера данных на четность
    //Если размер нечетный
    if (Size % 2) {
        Size = (Size / 2); //Размер в Half-word
        RVMSIS_FLASH_Page_erase(Adress); //Произведем стирание страницы
        //Если память заблокирована, то разблокируем ее
        if (READ_BIT(FLASH->CTLR, FLASH_CTLR_LOCK)) {
            RVMSIS_FLASH_Unlock();
        }
        SET_BIT(FLASH->CTLR, FLASH_CTLR_PG); //Выберем программу "programming"
        //Заполним ячейки по 16 бит
        for (int i = 0; i < Size; i++) {
            *(uint16_t*)(Adress + i * 2) = *((uint16_t*)(Data) + i);
            while (READ_BIT(FLASH->STATR, FLASH_STATR_BSY)) ;
        }
        //Заполним остаток в 8 бит
        *(uint16_t*)(Adress + Size * 2) = *((uint8_t*)(Data) + Size * 2);
    }
    //Если размер четный
    else {
        Size = (Size / 2); //Размер в Half-word
        RVMSIS_FLASH_Page_erase(Adress); //Произведем стирание страницы
        //Если память заблокирована, то разблокируем ее
        if (READ_BIT(FLASH->CTLR, FLASH_CTLR_LOCK)) {
            RVMSIS_FLASH_Unlock();
        }
        SET_BIT(FLASH->CTLR, FLASH_CTLR_PG); //Выберем программу "programming"
        //Заполним ячейки по 16 бит
        for (int i = 0; i < Size; i++) {
            *(uint16_t*)(Adress + i * 2) = *((uint16_t*)(Data) + i);
            while (READ_BIT(FLASH->STATR, FLASH_STATR_BSY)) ;
        }
    }
    while (READ_BIT(FLASH->STATR, FLASH_STATR_BSY)) ;
    while (READ_BIT(FLASH->STATR, FLASH_STATR_EOP) == 0) ;
    CLEAR_BIT(FLASH->CTLR, FLASH_CTLR_PG);
    RVMSIS_FLASH_Lock();
}



/**
 ***************************************************************************************
 *  @breif Считывание данных с FLASH.
 *  @param  Adress - Адрес во flash, откуда будем забирать данные
 *  @param  *Data - Данные, куда будем записывать информацию из flash с указанного адреса
 *  @param  Size - Размер даных. Сколько байт будем считывать.
 ***************************************************************************************
 */
void RVMSIS_FLASH_Read_data(uint32_t Adress, uint8_t *Data, uint16_t Size) {
    //Проверка размера данных на четность
    //Если размер структуры в байтах нечетный
    if (Size % 2) {
        Size = (Size / 2); //Размер в Half-word
        //Считаем данные по 16 бит
        for (uint16_t i = 0; i < Size; i++) {
            *((uint16_t*)Data + i) = *(uint16_t*)(Adress + i * 2);
        }
        //Считаем оставшиеся 8 бит
        *((uint8_t*)Data + Size * 2) = *(uint16_t*)(Adress + Size * 2);
    }//Если размер структуры в байтах четный
    else {
        Size = (Size / 2); //Размер в Half-word
        //Считаем информацию по 16 бит
        for (uint16_t i = 0; i < Size; i++) {
            *((uint16_t*)Data + i) = *(uint16_t*)(Adress + i * 2);
        }
    }
}
