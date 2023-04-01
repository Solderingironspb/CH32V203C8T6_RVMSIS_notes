/**
 ******************************************************************************
 *  @file ch32v20x_RVMSIS.h
 *  @brief RVMSIS §ß§Ñ §á§â§Ú§Þ§Ö§â§Ö §®§¬ CH32V203C8T6
 *  @author §£§à§Ý§Ü§à§Ó §°§Ý§Ö§Ô
 *  @date 31.03.2023
 *
 ******************************************************************************
 * @attention
 *
 *  §¢§Ú§Ò§Ý§Ú§à§ä§Ö§Ü§Ñ §á§à§Þ§à§Ô§Ñ§Ö§ä §â§Ñ§Ù§à§Ò§â§Ñ§ä§î§ã§ñ §ã §Ò§Ú§Ò§Ý§Ú§à§ä§Ö§Ü§à§Û RVMSIS §ß§Ñ §á§â§Ú§Þ§Ö§â§Ö
 *  §®§¬ CH32V203C8T6
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/STM32F103C8T6_CMSIS_notes
 *  §¤§â§å§á§á§Ñ §£§¬: https://vk.com/solderingiron.stm32
 *  §²§Ñ§Ò§à§ä§Ñ§Ý §á§à §Õ§à§Ü§å§Þ§Ö§ß§ä§Ñ§è§Ú§Ú: http://www.wch-ic.com/products/CH32V203.html?
 *
 ******************************************************************************
 */
#include "ch32v20x_RVMSIS.h"

/*================================= §¯§¡§³§´§²§°§«§¬§¡ DEBUG ============================================*/
/**
 ***************************************************************************************
 *  @breif Debug port mapping
 ***************************************************************************************
 */
void RVMSIS_Debug_init(void) {
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§©§Ñ§á§å§ã§ä§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ A
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //§©§Ñ§á§å§ã§ä§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§í§ç §æ§å§ß§Ü§è§Ú§Û
    MODIFY_REG(AFIO->PCFR1, AFIO_PCFR1_SWJ_CFG, 0b000 << AFIO_PCFR1_SWJ_CFG_Pos); //Serial wire

    /**
     *  §±§â§Ú §Ó§í§Ò§à§â§Ö Serial wire:
     *  PA13 /JTMS/SWDIO
     *  PA14 /JTCK/SWCLK.
     *  PA15, PB3 §Ú PB4 §ã§Ó§à§Ò§à§Õ§ß§í
     */
    /*§©§Ñ§Ò§Ý§à§Ü§Ú§â§å§Ö§Þ §Õ§à§ã§ä§å§á §Õ§Ý§ñ §â§Ö§Õ§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §Ü§à§ß§æ§Ú§Ô§å§â§Ñ§è§Ú§Ú PA13 §Ú PA14*/

    GPIOA->LCKR = GPIO_LCKK | GPIO_LCK13 | GPIO_LCK14;
    GPIOA->LCKR = GPIO_LCK13 | GPIO_LCK14;
    GPIOA->LCKR = GPIO_LCKK | GPIO_LCK13 | GPIO_LCK14;
    GPIOA->LCKR;
}

/*============================== §¯§¡§³§´§²§°§«§¬§¡ RCC =======================================*/
/**
 ***************************************************************************************
 *  @breif §¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §®§¬ CH32V203C8T6 §ß§Ñ §é§Ñ§ã§ä§à§ä§å 144MHz §à§ä §Ó§ß§Ö§ê§ß§Ö§Ô§à §Ü§Ó§Ñ§â§è§Ö§Ó§à§Ô§à §â§Ö§Ù§à§ß§Ñ§ä§à§â§Ñ
 *  §£§ß§Ö§ê§ß§Ú§Û §Ü§Ó§Ñ§â§è§Ö§Ó§í§Û §â§Ö§Ù§à§ß§Ñ§ä§à§â §ß§Ñ 8 MHz
 *  ADC §ß§Ñ§ã§ä§â§à§Ö§ß §ß§Ñ 12MHz
 *  USB §ß§Ñ§ã§ä§â§à§Ö§ß §ß§Ñ 48MHz
 *  MCO §á§à§Õ§Ü§Ý§ð§é§Ö§ß §Ü HSE §Ú §ä§Ñ§Ü§ä§Ú§â§å§Ö§ä§ã§ñ §à§ä 8MHz
 ***************************************************************************************
 */
void RVMSIS_RCC_SystemClock_144MHz(void) {
    SET_BIT(RCC->CTLR, RCC_HSION); //§©§Ñ§á§å§ã§ä§Ú§Þ §Ó§ß§å§ä§â§Ö§ß§ß§Ú§Û RC §Ô§Ö§ß§Ö§â§Ñ§ä§à§â §ß§Ñ 8 §®§¤§è
    while (READ_BIT(RCC->CTLR, RCC_HSIRDY) == 0);
    //§¥§à§Ø§Õ§Ö§Þ§ã§ñ §á§à§Õ§ß§ñ§ä§Ú§ñ §æ§Ý§Ñ§Ô§Ñ §à §Ô§à§ä§à§Ó§ß§à§ã§ä§Ú
    CLEAR_BIT(RCC->CTLR, RCC_HSEBYP);//§±§â§à§ã§ä§à §ã§Ò§â§à§ã§Ú§Þ §ï§ä§à§ä §Ò§Ú§ä §Ó 0(§·§à§ä§ñ §Ú§Ù§ß§Ñ§é§Ñ§Ý§î§ß§à §à§ß §Ú §ä§Ñ§Ü §Õ§à§Ý§Ø§Ö§ß §Ò§í§ä§î §Ó 0).
    SET_BIT(RCC->CTLR, RCC_HSEON); //§©§Ñ§á§å§ã§ä§Ú§Þ §Ó§ß§Ö§ê§ß§Ú§Û §Ü§Ó§Ñ§â§è§Ö§Ó§í§Û §â§Ö§Ù§à§ß§Ñ§ä§à§â. §°§ß §å §ß§Ñ§ã §ß§Ñ 8 MHz.
    while (READ_BIT(RCC->CTLR, RCC_HSERDY) == 0);
    //§¥§à§Ø§Õ§Ö§Þ§ã§ñ §á§à§Õ§ß§ñ§ä§Ú§ñ §æ§Ý§Ñ§Ô§Ñ §à §Ô§à§ä§à§Ó§ß§à§ã§ä§Ú
    SET_BIT(RCC->CTLR, RCC_CSSON);//§£§Ü§Ý§ð§é§Ú§Þ CSS
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b01 << RCC_SW_Pos); //§£§í§Ò§Ö§â§Ö§Þ HSE §Ó §Ü§Ñ§é§Ö§ã§ä§Ó§Ö System Clock(PLL §Ý§å§é§ê§Ö §á§à§Ü§Ñ §ß§Ö §Ó§í§Ò§Ú§â§Ñ§ä§î, §à§ß §å §ß§Ñ§ã §à§ä§Ü§Ý§ð§é§Ö§ß)
    CLEAR_BIT(RCC->CTLR, RCC_PLLON); //§£§í§Ü§Ý§ð§é§Ú§Þ PLL
    MODIFY_REG(RCC->CFGR0, RCC_HPRE, RCC_HPRE_DIV1); //AHB prescaler /1
    //Note: FLASH access clock frequency cannot
    //be more than 60 MHz.
    CLEAR_BIT(FLASH->CTLR, 1 << 25U); //0: FLASH access clock frequency = SYSCLK/2.
    MODIFY_REG(RCC->CFGR0, RCC_PPRE1, RCC_PPRE1_DIV1); //APB1 Prescaler /1,  §¢§å§Õ§Ö§ä 144MHz.
    MODIFY_REG(RCC->CFGR0, RCC_PPRE2, RCC_PPRE2_DIV2); //APB2 Prescaler /2. §¢§å§Õ§Ö§ä 72MHz.
    MODIFY_REG(RCC->CFGR0, RCC_ADCPRE, RCC_ADCPRE_DIV6); //ADC Prescaler /6, §é§ä§à§Ò §Ò§í§Ý§à 12MHz, §ä.§Ü. §Þ§Ñ§Ü§ã§Ú§Þ§Ñ§Ý§î§ß§Ñ§ñ §é§Ñ§ã§ä§à§ä§Ñ §ä§å§ä 14 MHz
    CLEAR_BIT(RCC->CFGR0, RCC_PLLXTPRE); //0: HSE clock not divided.
    SET_BIT(RCC->CFGR0, RCC_PLLSRC); //§£ §Ü§Ñ§é§Ö§ã§ä§Ó§Ö §Ó§ç§à§Õ§ß§à§Ô§à §ã§Ú§Ô§ß§Ñ§Ý§Ñ §Õ§Ý§ñ PLL §Ó§í§Ò§Ö§â§Ö§Þ HSE
    MODIFY_REG(RCC->CFGR0, RCC_PLLMULL, 0b1111 << RCC_PLLMULL_Pos); //§ä.§Ü. §Ü§Ó§Ñ§â§è §å §ß§Ñ§ã 8Mhz, §Ñ §ß§Ñ§Þ §ß§å§Ø§ß§à 144MHz, §ä§à §Ó PLL §ß§å§Ø§ß§à §ã§Õ§Ö§Ý§Ñ§ä§î §å§Þ§ß§à§Ø§Ö§ß§Ú§Ö §ß§Ñ 18. 8MHz * 18 = 144MHz.
    MODIFY_REG(RCC->CFGR0, RCC_USBPRE, 0b10 << RCC_USBPRE_Pos); //10: Divided by 3 (when PLLCLK=144MHz); 144/3 = 48 §®§¤§è
    MODIFY_REG(RCC->CFGR0, RCC_CFGR0_MCO, RCC_CFGR0_MCO_HSE); //§£ §Ü§Ñ§é§Ö§ã§ä§Ó§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §Õ§Ý§ñ MCO §Ó§í§Ò§â§Ñ§Ý HSE. §¢§å§Õ§Ö§ä 8 MHz.
    SET_BIT(RCC->CTLR, RCC_PLLON); //§©§Ñ§á§å§ã§ä§Ú§Þ PLL

    //§´.§Ü. PLL §å§Ø§Ö §Ù§Ñ§á§å§ë§Ö§ß, §Ó§í§Ò§Ö§â§Ö§Þ §Ö§Ô§à §Ó §Ü§Ñ§é§Ö§ã§ä§Ó§Ö System Clock:
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b10 << RCC_SW_Pos);//§£§í§Ò§Ö§â§Ö§Þ PLL §Ó §Ü§Ñ§é§Ö§ã§ä§Ó§Ö System Clock
    while (READ_BIT(RCC->CTLR, RCC_PLLRDY) == 0);
    //§¥§à§Ø§Ú§Õ§Ö§Þ§ã§ñ §á§à§Õ§ß§ñ§ä§Ú§ñ §æ§Ý§Ñ§Ô§Ñ §Ó§Ü§Ý§ð§é§Ö§ß§Ú§ñ PLL

}

/*========================= §¯§¡§³§´§²§°§«§¬§¡ §³§ª§³§´§¦§®§¯§°§¤§° §´§¡§«§®§¦§²§¡ ==============================*/

/**
 ***************************************************************************************
 *  @breif §¯§Ñ§ã§ä§â§à§Û§Ü§Ñ SysTick §ß§Ñ §Þ§Ú§Ü§â§à§ã§Ö§Ü§å§ß§Õ§í
 *  §¯§Ñ §ï§ä§à§Þ §ä§Ñ§Û§Þ§Ö§â§Ö §Þ§í §ß§Ñ§ã§ä§â§à§Ú§Þ Delay §Ú §Ñ§ß§Ñ§Ý§à§Ô HAL_GetTick()
 ***************************************************************************************
 */
void RVMSIS_SysTick_Timer_init(void) {
    SysTick->CTLR &= ~(1 << 0); //§£§í§Ü§Ý§ð§é§Ú§Þ §ä§Ñ§Û§Þ§Ö§â §Õ§Ý§ñ §á§â§à§Ó§Ö§Õ§Ö§ß§Ú§ñ §ß§Ñ§ã§ä§â§à§Ö§Ü.
    SysTick->CTLR |= (1 << 1); //1: Enable counter interrupts.
    SysTick->CTLR &= ~(1 << 2); //0: HCLK for time base.144/8 =18
    SysTick->CTLR |= (1 << 3); //1: Re-counting from 0 after counting up to the comparison value, and re-counting from the comparison value after counting down to 0
    SysTick->CTLR |= (1 << 4); //0: Counting up.
    SysTick->CTLR |= (1 << 5); //1: Updated to 0 on up counts, updated to the comparison value on down counts.
    SysTick->CMP = 17999; ////§¯§Ñ§ã§ä§â§à§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §ß§Ñ §é§Ñ§ã§ä§à§ä§å §Ó 1 §Ü§¤§è(§ä.§Ö. §ã§â§Ñ§Ò§à§ä§Ü§Ñ §Ò§å§Õ§Ö§ä §Ü§Ñ§Ø§Õ§å§ð §Þ§ã) 18000000 / 18000 = 1000§¤§è
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->CTLR |= (1 << 0); //§©§Ñ§á§å§ã§ä§Ú§Þ §ä§Ñ§Û§Þ§Ö§â.
}

/**
 ***************************************************************************************
 *  @breif §¯§Ñ§ã§ä§â§à§Û§Ü§Ñ Delay §Ú §Ñ§ß§Ñ§Ý§à§Ô HAL_GetTick()
 ***************************************************************************************
 */
volatile uint32_t SysTimer_ms = 0; //§±§Ö§â§Ö§Þ§Ö§ß§ß§Ñ§ñ, §Ñ§ß§Ñ§Ý§à§Ô§Ú§é§ß§Ñ§ñ HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0; //§³§é§Ö§ä§é§Ú§Ü §Õ§Ý§ñ §æ§å§ß§Ü§è§Ú§Ú Delay_ms
volatile uint32_t Timeout_counter_ms = 0; //§±§Ö§â§Ö§Þ§Ö§ß§ß§Ñ§ñ §Õ§Ý§ñ §ä§Ñ§Û§Þ§Ñ§å§ä§Ñ §æ§å§ß§Ü§è§Ú§Û

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §æ§Ý§Ñ§Ô§å CNTIF
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
 *  @param   uint32_t Milliseconds - §¥§Ý§Ú§ß§Ñ §Ù§Ñ§Õ§Ö§â§Ø§Ü§Ú §Ó §Þ§Ú§Ý§Ý§Ú§ã§Ö§Ü§å§ß§Õ§Ñ§ç
 ******************************************************************************
 */
void Delay_ms(uint32_t Milliseconds) {
    Delay_counter_ms = Milliseconds;
    while (Delay_counter_ms != 0);
}

/*============================== §¯§¡§³§´§²§°§«§¬§¡ GPIO =======================================*/

/**
 ***************************************************************************************
 *  @breif §ª§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ PIN PC13 §ß§Ñ §Ó§í§ç§à§Õ §Ó §â§Ö§Ø§Ú§Þ§Ö Push-Pull §ã §Þ§Ñ§Ü§ã§Ú§Þ§Ñ§Ý§î§ß§à§Û §ã§Ü§à§â§à§ã§ä§î§ð 50 MHz
 *  §±§Ö§â§Ö§Õ §ß§Ñ§ã§ä§â§à§Û§Ü§à§Û (GPIOs and AFIOs) §ß§å§Ø§ß§à §Ó§Ü§Ý§ð§é§Ú§ä§î §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ.
 ***************************************************************************************
 */
void RVMSIS_PC13_OUTPUT_Push_Pull_init(void) {
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //§©§Ñ§á§å§ã§ä§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ C
    MODIFY_REG(GPIOC->CFGHR, GPIO_CFGHR_MODE13, 0b10 << GPIO_CFGHR_MODE13_Pos); //§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ GPIOC §á§à§â§ä§Ñ 13 §ß§Ñ §Ó§í§ç§à§Õ §ã§à §Þ§Ñ§Ü§ã§Ú§Þ§Ñ§Ý§î§ß§à§Û §ã§Ü§à§â§à§ã§ä§î§ð §Ó 50 MHz
    MODIFY_REG(GPIOC->CFGHR, GPIO_CFGHR_CNF13, 0b00 << GPIO_CFGHR_CNF13_Pos); //§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ GPIOC §á§à§â§ä§Ñ 13 §ß§Ñ §Ó§í§ç§à§Õ §Ó §â§Ö§Ø§Ú§Þ§Ö Push-Pull

}

/**
 ***************************************************************************************
 *  @breif Blink PIN PC13 §ß§Ñ §Ó§í§ç§à§Õ §Ó §â§Ö§Ø§Ú§Þ§Ö Push-Pull
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
 *  @breif §¯§Ñ§ã§ä§â§à§Û§Ü§Ñ MCO c §Ó§í§ç§à§Õ§à§Þ §ß§Ñ §ß§à§Ø§Ü§å PA8
 *  §±§Ö§â§Ö§Õ §ß§Ñ§ã§ä§â§à§Û§Ü§à§Û (GPIOs and AFIOs) §ß§å§Ø§ß§à §Ó§Ü§Ý§ð§é§Ú§ä§î §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ.
 ***************************************************************************************
 */
void RVMSIS_PA8_MCO_init(void) {
    //§´§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö MCO §Õ§à§Ý§Ø§ß§à §Ò§í§ä§î §ß§Ñ§ã§ä§â§à§Ö§ß§à §Ó §â§Ö§Ô§Ú§ã§ä§â§Ö RCC
    MODIFY_REG(RCC->CFGR0, RCC_CFGR0_MCO, RCC_CFGR0_MCO_SYSCLK); //§£ §Ü§Ñ§é§Ö§ã§ä§Ó§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §Õ§Ý§ñ MCO §Ó§í§Ò§â§Ñ§Ý HSE. §¢§å§Õ§Ö§ä 8 MHz.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§©§Ñ§á§å§ã§ä§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ A
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_MODE8, 0b11 << GPIO_CFGHR_MODE8_Pos); //§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ GPIOA §á§à§â§ä§Ñ 8 §ß§Ñ §Ó§í§ç§à§Õ §ã§à §Þ§Ñ§Ü§ã§Ú§Þ§Ñ§Ý§î§ß§à§Û §ã§Ü§à§â§à§ã§ä§î§ð §Ó 50 MHz
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_CNF8, 0b10 << GPIO_CFGHR_CNF8_Pos); //§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ GPIOA §á§à§â§ä§Ñ 8, §Ü§Ñ§Ü §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§Ñ§ñ §æ§å§ß§Ü§è§Ú§ñ, §Ó §â§Ö§Ø§Ú§Þ§Ö Push-Pull
}

/*================================ §²§¦§¨§ª§® EXTI =======================================*/

/**
 ***************************************************************************************
 *  @breif §²§Ö§Ø§Ú§Þ EXTI.
 ***************************************************************************************
 */
void RVMSIS_EXTI0_init(void) {
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§í§ç §æ§å§ß§Ü§è§Ú§Û
    MODIFY_REG(AFIO->EXTICR[0], AFIO_EXTICR1_EXTI0, AFIO_EXTICR1_EXTI0_PB); //AFIO_EXTICR1, EXTI0, §Ó§í§Ò§â§Ñ§ß §á§à§â§ä B.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOB); //§£§Ü§Ý§ð§é§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ B
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_CNF0, 0b10 << GPIO_CFGLR_CNF0_Pos); //§¯§Ñ§ã§ä§â§à§Ú§Þ §ß§à§Ø§Ü§å PB0 §Ó §â§Ö§Ø§Ú§Þ Input with pull-up / pull-down
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_MODE0, 0b00 << GPIO_CFGLR_MODE0_Pos); //§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §Ó §â§Ö§Ø§Ú§Þ Input
    GPIOB->BSHR = GPIO_BSHR_BR0; //§±§à§Õ§ä§ñ§Ø§Ü§Ñ §Ü §Ù§Ö§Þ§Ý§Ö
    SET_BIT(EXTI->INTENR, EXTI_INTENR_MR0); //§£§Ü§Ý§ð§é§Ñ§Ö§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö EXTI0 §á§à §Ó§ç§à§Õ§ß§à§Þ§å §ã§Ú§Ô§ß§Ñ§Ý§å
    SET_BIT(EXTI->RTENR, EXTI_RTENR_TR0); //§²§Ö§Ñ§Ô§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à §æ§â§à§ß§ä§å §Ó§Ü§Ý.
    SET_BIT(EXTI->FTENR, EXTI_FTENR_TR0); //§²§Ö§Ñ§Ô§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à §ã§á§Ñ§Õ§å §Ó§Ü§Ý.
    //SET_BIT(EXTI->SWIEVR, EXTI_SWIEVR_SWIEVR0);//§¿§ä§à §ã§à§æ§ä§Ó§Ñ§â§ß§à§Ö §Ó§Ü§Ý§ð§é§Ö§ß§Ú§Ö §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ
    //SET_BIT(EXTI->INTFR, EXTI_INTF_INTF0); //§¬§à§Þ§Ñ§ß§Õ§Ñ §Ó§í§ç§à§Õ§Ñ §Ú§Ù §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ
    NVIC_EnableIRQ(EXTI0_IRQn);

}

__WEAK void EXTI0_IRQHandler(void) {

    SET_BIT(EXTI->INTFR, EXTI_INTF_INTF0); //§¬§à§Þ§Ñ§ß§Õ§Ñ §Ó§í§ç§à§Õ§Ñ §Ú§Ù §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ

}

/*================================ §´§Ñ§Û§Þ§Ö§â§í §ß§Ñ §á§â§Ú§Þ§Ö§â§Ö TIM3 =======================================*/

void RVMSIS_TIM3_init(void) {
    /*§£§Ü§Ý§ð§é§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §ä§Ñ§Û§Þ§Ö§â§Ñ (§ã§ä§â§Ñ§ß§Ú§è§Ñ 48)*/
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_TIM3); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §ä§Ñ§Û§Þ§Ö§â§Ñ 3
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§í§ç §æ§å§ß§Ü§è§Ú§Û

    CLEAR_BIT(TIM3->CTLR1, TIM_UDIS); //§¤§Ö§ß§Ö§â§Ú§â§à§Ó§Ñ§ä§î §ã§à§Ò§í§ä§Ú§Ö Update
    CLEAR_BIT(TIM3->CTLR1, TIM_URS); //§¤§Ö§ß§Ö§â§Ú§â§à§Ó§Ñ§ä§î §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö
    CLEAR_BIT(TIM3->CTLR1, TIM_OPM); //One pulse mode off(§³§é§Ö§ä§é§Ú§Ü §ß§Ö §à§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§ä§ã§ñ §á§â§Ú §à§Ò§ß§à§Ó§Ý§Ö§ß§Ú§Ú)
    CLEAR_BIT(TIM3->CTLR1, TIM_DIR); //§³§é§Ú§ä§Ñ§Ö§Þ §Ó§ß§Ú§Ù
    MODIFY_REG(TIM3->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos); //§£§í§â§Ñ§Ó§ß§Ú§Ó§Ñ§ß§Ú§Ö §á§à §Ü§â§Ñ§ð
    SET_BIT(TIM3->CTLR1, TIM_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM3->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos); //§±§â§Ö§Õ§Õ§Ö§Ý§Ö§ß§Ú§Ö §Ó§í§Ü§Ý§ð§é§Ö§ß§à

    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Û (§³§ä§â§Ñ§ß§Ú§è§Ñ 409)*/
    SET_BIT(TIM3->DMAINTENR, TIM_UIE); //Update interrupt enable

    TIM3->PSC = 14400 - 1;
    TIM3->ATRLR = 10 - 1;

    NVIC_EnableIRQ(TIM3_IRQn); //§²§Ñ§Ù§â§Ö§ê§Ú§ä§î §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ §á§à §ä§Ñ§Û§Þ§Ö§â§å 3
    SET_BIT(TIM3->CTLR1, TIM_CEN); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Û§Þ§Ö§â§Ñ
}

__WEAK void TIM3_IRQHandler(void) {
    if (READ_BIT(TIM3->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM3->INTFR, TIM_UIF); //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ
    }
}

void RVMSIS_TIM3_PWM_CHANNEL1_init(void) {
    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ß§à§Ø§Ü§Ú PA6 §á§à§Õ §º§ª§®*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§£§Ü§Ý§ð§é§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ §¡
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF6, 0b10 << GPIO_CFGLR_CNF6_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE6, 0b11 << GPIO_CFGLR_MODE6_Pos);

    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ê§Ú§Þ(§¬§Ñ§ß§Ñ§Ý 1)*/
    MODIFY_REG(TIM3->CHCTLR1, TIM_CC1S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC1FE); //Fast mode disable
    SET_BIT(TIM3->CHCTLR1, TIM_OC1PE); //Preload enable
    MODIFY_REG(TIM3->CHCTLR1, TIM_OC1M, 0b110 << TIM_OC1M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC1CE); //OC1Ref is not affected by the ETRF input

    /*§©§Ñ§á§å§ã§Ü §º§ª§®*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM3->CCER, TIM_CC1E);//On - OC1 signal is output on the corresponding output pin.
    SET_BIT(TIM3->CCER, TIM_CC1P); //OC1 active high.

    TIM3->CH1CVR = 5;
}

void RVMSIS_TIM3_PWM_CHANNEL2_init(void) {
    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ß§à§Ø§Ü§Ú PA7 §á§à§Õ §º§ª§®*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§£§Ü§Ý§ð§é§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ §¡
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF7, 0b10 << GPIO_CFGLR_CNF7_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE7, 0b11 << GPIO_CFGLR_MODE7_Pos);

    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ê§Ú§Þ(§¬§Ñ§ß§Ñ§Ý 2)*/
    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ê§Ú§Þ(§¬§Ñ§ß§Ñ§Ý 1)*/
    MODIFY_REG(TIM3->CHCTLR1, TIM_CC2S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC2FE); //Fast mode disable
    SET_BIT(TIM3->CHCTLR1, TIM_OC2PE); //Preload enable
    MODIFY_REG(TIM3->CHCTLR1, TIM_OC2M, 0b110 << TIM_OC2M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC2CE); //OC1Ref is not affected by the ETRF input

    /*§©§Ñ§á§å§ã§Ü §º§ª§®*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM3->CCER, TIM_CC2E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM3->CCER, TIM_CC2P); //OC1 active high.

    TIM3->CH2CVR = 5;
}

/*================================= §¯§¡§³§´§²§°§«§¬§¡ ADC ============================================*/

/**
 ***************************************************************************************
 *  @breif Analog-to-digital converter (ADC)
 ***************************************************************************************
 */

volatile uint16_t ADC_RAW_Data[2] = { 0, }; //§®§Ñ§ã§ã§Ú§Ó, §Ü§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §Ü§Ú§Õ§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö §ã §¡§¸§±

void RVMSIS_ADC_DMA_init(void) {
    //Chapter 11 Direct Memory Access Control (DMA)
    SET_BIT(RCC->AHBPCENR, RCC_AHBPeriph_DMA1); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ DMA1
    DMA1_Channel1->PADDR = (uint32_t) &(ADC1->RDATAR); //§©§Ñ§Õ§Ñ§Ö§Þ §Ñ§Õ§â§Ö§ã §á§Ö§â§Ú§æ§Ö§â§Ú§Û§ß§à§Ô§à §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
    DMA1_Channel1->MADDR = (uint32_t) ADC_RAW_Data; //§©§Ñ§Õ§Ñ§Ö§Þ §Ñ§Õ§â§Ö§ã §Ó §á§Ñ§Þ§ñ§ä§Ú, §Ü§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §Ü§Ú§Õ§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö.
    DMA1_Channel1->CNTR = 2; //§¯§Ñ§ã§ä§â§à§Ú§Þ §Ü§à§Ý§Ú§é§Ö§ã§ä§Ó§à §Õ§Ñ§ß§ß§í§ç §Õ§Ý§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú. §±§à§ã§Ý§Ö §Ü§Ñ§Ø§Õ§à§Ô§à §á§Ö§â§Ú§æ§Ö§â§Ú§Û§ß§à§Ô§à §ã§à§Ò§í§ä§Ú§ñ §ï§ä§à §Ù§ß§Ñ§é§Ö§ß§Ú§Ö §Ò§å§Õ§Ö§ä §å§Þ§Ö§ß§î§ê§Ñ§ä§î§ã§ñ.
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PL, 0b00 << DMA_CFGR1_PL_Pos); //§©§Ñ§Õ§Ñ§Õ§Ú§Þ §á§â§Ú§à§â§Ú§ä§Ö§ä §Ü§Ñ§ß§Ñ§Ý§Ñ §ß§Ñ §Ó§í§ã§à§Ü§Ú§Û
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_DIR); //§¹§ä§Ö§ß§Ú§Ö §Ò§å§Õ§Ö§Þ §à§ã§å§ë§Ö§ã§ä§Ó§Ý§ñ§ä§î §ã §á§Ö§â§Ú§æ§Ö§â§Ú§Ú
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_CIRC); //§¯§Ñ§ã§ä§â§à§Ú§Þ DMA §Ó Circular mode
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PSIZE, 0b01 << DMA_CFGR1_PSIZE_Pos); //§²§Ñ§Ù§Þ§Ö§â §Õ§Ñ§ß§ß§í§ç §á§Ö§â§Ú§æ§Ö§â§Ú§Û§ß§à§Ô§à §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ 16 §Ò§Ú§ä
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_MSIZE, 0b01 << DMA_CFGR1_MSIZE_Pos); //§²§Ñ§Ù§Þ§Ö§â §Õ§Ñ§ß§ß§í§ç §Ó §á§Ñ§Þ§ñ§ä§Ú 16 §Ò§Ú§ä
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TCIE); //§£§Ü§Ý§ð§é§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §á§à§Ý§ß§à§Û §á§Ö§â§Ö§Õ§Ñ§é§Ö
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_HTIE); //§°§ä§Ü§Ý§ð§é§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §á§à§Ý§à§Ó§Ú§ß§ß§à§Û §á§Ö§â§Ö§Õ§Ñ§é§Ö
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TEIE); //§£§Ü§Ý§ð§é§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §à§ê§Ú§Ò§Ü§Ö §á§Ö§â§Ö§Õ§Ñ§é§Ú.
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_MINC); //§£§Ü§Ý§ð§é§Ú§Þ §Ú§ß§Ü§â§Ö§Þ§Ö§ß§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§Ñ§Þ§ñ§ä§Ú
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_EN); //DMA ON
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_ADC1); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ ADC1.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §á§à§â§ä§Ñ §¡.

    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ß§à§Ø§Ö§Ü PA0 §Ú PA1 §ß§Ñ §Ñ§ß§Ñ§Ý§à§Ô§à§Ó§í§Û §Ó§ç§à§Õ*/
    /*Pin PA0 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF0, 0b00 << GPIO_CFGLR_CNF0_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE0, 0b00 << GPIO_CFGLR_MODE0_Pos);

    /*Pin PA1 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b00 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b00 << GPIO_CFGLR_MODE1_Pos);

    //§±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §¡§¸§±: §â§Ö§Ô§å§Ý§ñ§â§ß§í§Ö §Ü§Ñ§ß§Ñ§Ý§í (§Ó§Ü§Ý/§Ó§í§Ü§Ý)
    CLEAR_BIT(ADC1->CTLR1, ADC_EOCIE);//EOC interrupt enabled/disabled. An interrupt is generated when the EOC bit is set

    //§±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §¡§¸§±: analog watchdog (§Ó§Ü§Ý/§Ó§í§Ü§Ý)
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDIE);//Analog watchdog interrupt disabled

    //§±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §¡§¸§±: §Ú§ß§Ø§Ö§Ü§ä§Ú§â§à§Ó§Ñ§ß§ß§í§Ö §Ü§Ñ§ß§Ñ§Ý§í (§Ó§Ü§Ý/§Ó§í§Ü§Ý)
    CLEAR_BIT(ADC1->CTLR1, ADC_JEOCIE);//JEOC interrupt disabled

    SET_BIT(ADC1->CTLR1, ADC_SCAN); //Scan mode enabled

    /* §±§â§Ú§Þ§Ö§é§Ñ§ß§Ú§Ö:
     * §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö EOC §Ú§Ý§Ú JEOC §Ô§Ö§ß§Ö§â§Ú§â§å§Ö§ä§ã§ñ §ä§à§Ý§î§Ü§à §Ó §Ü§à§ß§è§Ö §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§ñ §á§à§ã§Ý§Ö§Õ§ß§Ö§Ô§à §Ü§Ñ§ß§Ñ§Ý§Ñ,
     * §Ö§ã§Ý§Ú §å§ã§ä§Ñ§ß§à§Ó§Ý§Ö§ß §ã§à§à§ä§Ó§Ö§ä§ã§ä§Ó§å§ð§ë§Ú§Û §Ò§Ú§ä EOCIE §Ú§Ý§Ú JEOCIE.*/

    CLEAR_BIT(ADC1->CTLR1, ADC_AWDSGL); //Analog watchdog enabled on all channels
    CLEAR_BIT(ADC1->CTLR1, ADC_JAUTO); //Automatic injected group conversion disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_DISCEN); //Discontinuous mode on regular channels disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_JDISCEN); //Discontinuous mode on injected channels disabled
    MODIFY_REG(ADC1->CTLR1, ADC_DUALMOD, 0b0110 << ADC_DUALMOD_Pos); //0110: Regular simultaneous mode only
    CLEAR_BIT(ADC1->CTLR1, ADC_JAWDEN); //Analog watchdog disabled on injected channels
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDEN); //Analog watchdog disabled on regular channels

    //Control register 2 CTLR2
    SET_BIT(ADC1->CTLR2, ADC_ADON);//§©§Ñ§á§å§ã§ä§Ú§ä§î §¡§¸§±

    /* §±§â§Ú§Þ§Ö§é§Ñ§ß§Ú§Ö:
     * §¦§ã§Ý§Ú §Ó §ï§ä§à§ä §Ø§Ö §Þ§à§Þ§Ö§ß§ä §Ú§Ù§Þ§Ö§ß§ñ§Ö§ä§ã§ñ §Ü§Ñ§Ü§à§Û-§Ý§Ú§Ò§à §Õ§â§å§Ô§à§Û §Ò§Ú§ä §Ó §ï§ä§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö,
     * §Ü§â§à§Þ§Ö ADON, §ä§à §Ü§à§ß§Ó§Ö§â§ã§Ú§ñ §ß§Ö §Ù§Ñ§á§å§ã§Ü§Ñ§Ö§ä§ã§ñ.
     * §¿§ä§à §ã§Õ§Ö§Ý§Ñ§ß§à §Õ§Ý§ñ §á§â§Ö§Õ§à§ä§Ó§â§Ñ§ë§Ö§ß§Ú§ñ §à§ê§Ú§Ò§à§é§ß§à§Ô§à §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§ñ.*/

    SET_BIT(ADC1->CTLR2, ADC_CONT); //Continuous conversion mode(§ß§Ö§á§â§Ö§â§í§Ó§ß§í§Ö §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§ñ)
    SET_BIT(ADC1->CTLR2, ADC_CAL); //Enable calibration
    /*§±§â§Ú§Þ§Ö§é§Ñ§ß§Ú§Ö:
     * §¿§ä§à§ä §Ò§Ú§ä §å§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§à§Û §Õ§Ý§ñ §Ù§Ñ§á§å§ã§Ü§Ñ §Ü§Ñ§Ý§Ú§Ò§â§à§Ó§Ü§Ú.
     * §°§ß §ã§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§ä§ã§ñ §Ñ§á§á§Ñ§â§Ñ§ä§ß§à §á§à§ã§Ý§Ö §Ù§Ñ§Ó§Ö§â§ê§Ö§ß§Ú§ñ §Ü§Ñ§Ý§Ú§Ò§â§à§Ó§Ü§Ú.*/

    while (READ_BIT(ADC1->CTLR2, ADC_CAL));
    //§±§à§Õ§à§Ø§Õ§Ö§Þ §à§Ü§à§ß§é§Ñ§ß§Ú§ñ §Ü§Ñ§Ý§Ú§Ò§â§à§Ó§Ü§Ú
    //Delay_ms(10);

    SET_BIT(ADC1->CTLR2, ADC_DMA); //DMA §Ó§Ü§Ý§ð§é§Ö§ß
    CLEAR_BIT(ADC1->CTLR2, ADC_ALIGN); //§£§í§â§Ñ§Ó§ß§Ú§Ó§Ñ§ß§Ú§Ö §á§à §á§â§Ñ§Ó§à§Þ§å §Ü§â§Ñ§ð
    MODIFY_REG(ADC1->CTLR2, ADC_EXTSEL, 0b111 << ADC_EXTSEL_Pos); //§©§Ñ§á§å§ã§Ü§Ñ§ä§î §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§Ö §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à
    CLEAR_BIT(ADC1->CTLR2, ADC_EXTTRIG); //Conversion on external event disabled
    //SET_BIT(ADC1->CTLR2, ADC_SWSTART); //§¯§Ñ§é§Ñ§ä§î §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§Ö
    //SET_BIT(ADC1->CTLR2, ADC_TSVREFE);//Temperature sensor and VREFINT channel enabled

    // 12.3.5 ADCx Sample Time Configuration Register 2 (ADCx_SAMPTR2) (x=1/2)
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP0, 0b111 << ADC_SMP0_Pos);//239.5 cycles
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP1, 0b111 << ADC_SMP1_Pos); //239.5 cycles
    //MODIFY_REG(ADC1->SAMPTR1, ADC_SMP17, 0b111 << ADC_SMP17_Pos); //239.5 cycles

    //12.3.9 ADCx Regular Channel Sequence Register1 (ADCx_RSQR1) (x=1/2)
    MODIFY_REG(ADC1->RSQR1, ADC_L, 0b0001 << ADC_L_Pos);//2 §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§ñ

    //12.3.11 ADCx Regular Channel Sequence Register 3 (ADCx_RSQR3) (x=1/2)
    MODIFY_REG(ADC1->RSQR3, ADC_SQ1, 0 << ADC_SQ1_Pos);
    MODIFY_REG(ADC1->RSQR3, ADC_SQ2, 1 << ADC_SQ2_Pos);
    //NVIC_EnableIRQ(ADC1_2_IRQn); //§²§Ñ§Ù§â§Ö§ê§Ú§ä§î §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ §á§à §¡§¸§±

    //SET_BIT(ADC1->CTLR2, ADC_SWSTART); //§¯§Ñ§é§Ñ§ä§î §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§Ö. §¯§Ö §ß§å§Ø§ß§à §Ù§Ñ§á§å§ã§Ü§Ñ§ä§î, §Ö§ã§Ý§Ú Continuous conversion mode(§ß§Ö§á§â§Ö§â§í§Ó§ß§í§Ö §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§ñ) §Ó§Ü§Ý§ð§é§Ö§ß

}

__WEAK void ADC1_2_IRQHandler(void) {
    if (READ_BIT(ADC1->STATR, ADC_EOC)) {
        ADC1->IDATAR1; //§¹§Ú§ä§Ñ§Ö§Þ §Ü§Ñ§ß§Ñ§Ý, §é§ä§à§Ò §ã§Ò§â§à§ã§Ú§ä§î §æ§Ý§Ñ§Ô
    }

}
__WEAK void DMA1_Channel1_IRQHandler(void) {
    if (READ_BIT(DMA1->INTFR, DMA_TCIF1)) {
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //§³§Ò§â§à§ã§Ú§Þ §Ô§Ý§à§Ò§Ñ§Ý§î§ß§í§Û §æ§Ý§Ñ§Ô.
        /*§©§Õ§Ö§ã§î §Þ§à§Ø§ß§à §á§Ú§ã§Ñ§ä§î §Ü§à§Õ*/

    }
    else if (READ_BIT(DMA1->INTFR, DMA_TEIF1)) {
        /*§©§Õ§Ö§ã§î §Þ§à§Ø§ß§à §ã§Õ§Ö§Ý§Ñ§ä§î §Ü§Ñ§Ü§à§Û-§ä§à §à§Ò§â§Ñ§Ò§à§ä§é§Ú§Ü §à§ê§Ú§Ò§à§Ü*/
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //§³§Ò§â§à§ã§Ú§Þ §Ô§Ý§à§Ò§Ñ§Ý§î§ß§í§Û §æ§Ý§Ñ§Ô.
    }
}

/*================================= §¯§¡§³§´§²§°§«§¬§¡ USART ============================================*/

/**
***************************************************************************************
*  @breif Universal synchronous asynchronous receiver transmitter (USART)
***************************************************************************************
*/

