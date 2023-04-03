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
 *  P.S. §Þ§à§Ø§Ö§ä§Ö §Ü§Ú§Õ§Ñ§ä§î §Ó §Þ§Ö§ß§ñ §Ü§Ñ§Þ§ß§ñ§Þ§Ú, §ß§à §ñ §ã§é§Ú§ä§Ñ§ð, §é§ä§à §à§ä 144§®§¤§è §ä§å§ä §á§à§Ý§î§Ù§í, §Ü§Ñ§Ü
 *  §à§ä §Ü§à§Ù§Ý§Ñ §Þ§à§Ý§à§Ü§Ñ) I2C1 §Ú I2C2 §ß§Ñ APB1. §¥§Ý§ñ §ß§Ú§ç §Þ§Ñ§Ü§ã§Ú§Þ§å§Þ 36 §®§¤§è.
 *  §³§Ü§à§â§à§ã§ä§î FLASH §Þ§Ñ§Ü§ã§Ú§Þ§å§Þ 60 §®§¤§è, §Ñ §Õ§Ö§Ý§Ú§ä§Ö§Ý§î §ä§Ñ§Þ §Ö§ã§ä§î §ä§à§Ý§î§Ü§à §ß§Ñ 2, §é§ä§à §à§á§ñ§ä§î
 *  §ß§Ö §Õ§Ñ§Ö§ä §ß§Ñ§Þ §Ó§í§ã§ä§Ñ§Ó§Ú§ä§î 144 §®§¤§è. §¡ §Ö§ã§Ý§Ú §Þ§í §ç§à§ä§Ú§Þ §â§Ñ§Ò§à§ä§Ñ§ä§î §ã ADC, §ä§à §à§á§ñ§ä§î §Ø§Ö, §ß§Ñ APB2
 *  §Þ§í §ß§Ö §Þ§à§Ø§Ö§Þ §Ó§í§ã§ä§Ñ§Ó§Ú§ä§î 144 §®§¤§è, §ä.§Ü. §Þ§Ñ§Ü§ã§Ú§Þ§Ñ§Ý§î§ß§í§Û §Õ§Ö§Ý§Ú§ä§Ö§Ý§î 8(144/8 = 18§®§¤§è.)
 *  §¡ §Þ§Ñ§Ü§ã§Ú§Þ§å§Þ §Õ§Ý§ñ §¡§¸§± - 14 §®§¤§è...§±§à§ï§ä§à§Þ§å §Õ§Ý§ñ §å§ß§Ú§Ó§Ö§â§ã§Ñ§Ý§î§ß§à§Û §â§Ñ§Ò§à§ä§í §Ó§ã§Ö§Ô§à, §é§Ñ§ã§ä§à§ä§í §Ù§Ñ§Õ§Ú§â§Ñ§ä§î
 *  §Ó§í§ê§Ö 72 §®§¤§è §ß§Ö §Ò§å§Õ§Ö§Þ.
 *  §£§ß§Ö§ê§ß§Ú§Û §Ü§Ó§Ñ§â§è§Ö§Ó§í§Û §â§Ö§Ù§à§ß§Ñ§ä§à§â §ß§Ñ 8 MHz
 *  ADC §ß§Ñ§ã§ä§â§à§Ö§ß §ß§Ñ 12MHz
 *  USB §ß§Ñ§ã§ä§â§à§Ö§ß §ß§Ñ 48MHz
 *  MCO §á§à§Õ§Ü§Ý§ð§é§Ö§ß §Ü HSE §Ú §ä§Ñ§Ü§ä§Ú§â§å§Ö§ä§ã§ñ §à§ä 8MHz
 *
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
    MODIFY_REG(RCC->CFGR0, RCC_HPRE, RCC_HPRE_DIV2); //AHB prescaler /2
    //Note: FLASH access clock frequency cannot
    //be more than 60 MHz.
    CLEAR_BIT(FLASH->CTLR, 1 << 25U); //0: FLASH access clock frequency = SYSCLK/2.
    MODIFY_REG(RCC->CFGR0, RCC_PPRE1, RCC_PPRE1_DIV2); //APB1 Prescaler /2,  72/2 = 36 MHz.(§ª§ß§Ñ§é§Ö §å §ß§Ñ§ã I2C §Ò§å§Õ§Ö§ä §ß§Ö §ß§Ñ§ã§ä§â§à§Ú§ä§î...)
    MODIFY_REG(RCC->CFGR0, RCC_PPRE2, RCC_PPRE2_DIV1); //APB2 Prescaler /1. §¢§å§Õ§Ö§ä 72MHz.
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
    SysTick->CMP = 8999; ////§¯§Ñ§ã§ä§â§à§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §ß§Ñ §é§Ñ§ã§ä§à§ä§å §Ó 1 §Ü§¤§è(§ä.§Ö. §ã§â§Ñ§Ò§à§ä§Ü§Ñ §Ò§å§Õ§Ö§ä §Ü§Ñ§Ø§Õ§å§ð §Þ§ã) 18000000 / 18000 = 1000§¤§è
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

    SET_BIT(ADC1->CTLR2, ADC_DMA);//DMA §Ó§Ü§Ý§ð§é§Ö§ß
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

    } else if (READ_BIT(DMA1->INTFR, DMA_TEIF1)) {
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

struct USART_name husart1; //§°§Ò§ì§ñ§Ó§Ý§ñ§Ö§Þ §ã§ä§â§å§Ü§ä§å§â§å §á§à USART.(§ã§Þ. ch32v203x_RVMSIS.h)
struct USART_name husart2; //§°§Ò§ì§ñ§Ó§Ý§ñ§Ö§Þ §ã§ä§â§å§Ü§ä§å§â§å §á§à USART.(§ã§Þ. ch32v203x_RVMSIS.h)

/**
 ******************************************************************************
 *  @breif §¯§Ñ§ã§ä§â§à§Û§Ü§Ñ USART1. §±§Ñ§â§Ñ§Þ§Ö§ä§â§í 9600 8 N 1
 ******************************************************************************
 */

void RVMSIS_USART1_Init(void) {

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ §¡
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§í§ç §æ§å§ß§Ü§è§Ú§Û

    //§¥§Ý§ñ §Ü§à§ß§æ§Ú§Ô§å§â§Ú§â§à§Ó§Ñ§ß§Ú§Ö §ß§à§Ø§Ö§Ü UART §Õ§Ý§ñ Full Duplex §ß§å§Ø§ß§à §Ú§ã§á§à§Ý§î§Ù§à§Ó§Ñ§ä§î Alternate function push-pull(§³§Þ. §á.§á. 9.1.11 GPIO configurations for device peripherals §ã§ä§â.111 Reference Manual)
    //Tx - Alternative Function output Push-pull(Maximum output speed 50 MHz)
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_CNF9, 0b10 << GPIO_CFGHR_CNF9_Pos);
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_MODE9, 0b11 << GPIO_CFGHR_MODE9_Pos);
    //Rx - Input floating
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_CNF10, 0b1 << GPIO_CFGHR_CNF10_Pos);
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_MODE10, 0b00 << GPIO_CFGHR_MODE10_Pos);

    //§©§Ñ§á§å§ã§ä§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö USART1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_USART1);

    /*§²§Ñ§ã§é§Ö§ä Fractional baud rate generation
     §Ö§ã§ä§î §æ§à§â§Þ§å§Ý§Ñ:
     Tx/Rx baud = fCK/(16*USARTDIV)
     §Ô§Õ§Ö fCK - Input clock to the peripheral (PCLK1 for USART2, 3, 4, 5 or PCLK2 for USART1)
     §Ó §ß§Ñ§ê§Ö§Þ §ã§Ý§å§é§Ñ§Ö fCK = 72000000
     §Õ§à§á§å§ã§ä§Ú§Þ §ß§Ñ§Þ §ß§å§Ø§ß§Ñ §ã§Ü§à§â§à§ã§ä§î 9600
     9600 = 72000000/(16*USARTDIV)
     §´§à§Ô§Õ§Ñ USARTDIV = 72000000/9600*16 = 468.75
     DIV_Mantissa §Ó §Õ§Ñ§ß§ß§à§Þ §ã§Ý§å§é§Ñ§Ö §Ò§å§Õ§Ö§ä 468, §é§ä§à §Ö§ã§ä§î 0x1D4
     DIV_Fraction §Ò§å§Õ§Ö§ä, §Ü§Ñ§Ü 0.75*16 = 12, §é§ä§à §Ö§ã§ä§î 0xC
     §´§à§Ô§Õ§Ñ §Ó§Ö§ã§î §â§Ö§Ô§Ú§ã§ä§â USART->BRR §Õ§Ý§ñ §ã§Ü§à§â§à§ã§ä§Ú 9600 §Ò§å§Õ§Ö§ä §Ó§í§Ô§Ý§ñ§Õ§Ö§ä§î, §Ü§Ñ§Ü 0x1D4C.
     §Õ§Ý§ñ §á§â§Ú§Þ§Ö§â§Ñ §Ö§ë§Ö §â§Ñ§Ù§Ò§Ö§â§Ö§Þ §ã§Ü§à§â§à§ã§ä§î 115200:
     115200 = 72000000/(16*USARTDIV)
     §´§à§Ô§Õ§Ñ USARTDIV = 72000000/115200*16 = 39.0625
     DIV_Mantissa §Ó §Õ§Ñ§ß§ß§à§Þ §ã§Ý§å§é§Ñ§Ö §Ò§å§Õ§Ö§ä 39, §é§ä§à §Ö§ã§ä§î 0x27
     DIV_Fraction §Ò§å§Õ§Ö§ä, §Ü§Ñ§Ü 0.0625*16 = 1, §é§ä§à §Ö§ã§ä§î 0x1
     §´§à§Ô§Õ§Ñ §Ó§Ö§ã§î §â§Ö§Ô§Ú§ã§ä§â USART->BRR §Õ§Ý§ñ §ã§Ü§à§â§à§ã§ä§Ú 115200 §Ò§å§Õ§Ö§ä §Ó§í§Ô§Ý§ñ§Õ§Ö§ä§î, §Ü§Ñ§Ü 0x271.
     */

    MODIFY_REG(USART1->BRR, USART_BRR_DIV_Mantissa, 0x1D4 << USART_BRR_DIV_Mantissa_Pos);
    MODIFY_REG(USART1->BRR, USART_BRR_DIV_Fraction, 0xC << USART_BRR_DIV_Mantissa_Pos);

    //18.10.4 USART Control Register1 (USARTx_CTLR1) (x=1/2/3/4/5/6/7/8)
    SET_BIT(USART1->CTLR1, USART_CTLR1_UE);//USART enable
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_M); //Word lenght 1 Start bit, 8 Data bits, n Stop bit
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_WAKE); //Wake up idle Line
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_PCE); //Partity control disabled
    //§ß§Ñ§ã§ä§â§à§Û§Ü§Ñ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Û
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_PEIE);//partity error interrupt disabled
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_TXEIE); //TXE interrupt is inhibited
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_TCIE); //Transmission complete interrupt disabled
    SET_BIT(USART1->CTLR1, USART_CTLR1_RXNEIE); //§±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §á§â§Ú§Ö§Þ§å §Õ§Ñ§ß§ß§í§ç §Ó§Ü§Ý§ð§é§Ö§ß§à
    SET_BIT(USART1->CTLR1, USART_CTLR1_IDLEIE); //§±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §æ§Ý§Ñ§Ô§å IDLE §Ó§Ü§Ý§ð§é§Ö§ß§à
    SET_BIT(USART1->CTLR1, USART_CTLR1_TE); //Transmitter is enabled
    SET_BIT(USART1->CTLR1, USART_CTLR1_RE); //Receiver is enabled and begins searching for a start bit
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_RWU);
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_SBK);

    //§°§ã§ä§Ñ§Ý§î§ß§å§ð §ß§Ñ§ã§ä§â§à§Û§Ü§å, §ß§Ö §Ü§Ñ§ã§Ñ§ð§ë§å§ð§ã§ñ §ã§ä§Ñ§ß§Õ§Ñ§â§ä§ß§à§Ô§à USART, §Þ§í §á§à§Ü§Ñ §ä§â§à§Ô§Ñ§ä§î §ß§Ö §Ò§å§Õ§Ö§Þ, §ß§à §ß§Ñ §Ó§ã§ñ§Ü§Ú§Û §ã§Ý§å§é§Ñ§Û §à§Ò§ß§å§Ý§Ú§Þ
    //18.10.5 USART Control Register2 (USARTx_CTLR2) (x=1/2/3/4/5/6/7/8)
    USART1->CTLR2 = 0;
    CLEAR_BIT(USART1->CTLR2, USART_CTLR2_STOP); //1 §ã§ä§à§á §Ò§Ú§ä.
    //18.10.6 USART Control Register 3 (USARTx_CTLR3) (x=1/2/3/4/5/6/7/8)
    USART1->CTLR3 = 0;
    //18.10.7 USART Guard Time and Prescaler Register (USARTx_GPR) (x=1/2/3/4/5/6/7/8)
    USART1->GPR = 0;

    NVIC_EnableIRQ(USART1_IRQn); //§£§Ü§Ý§ð§é§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ §á§à USART1
}

/**
 ******************************************************************************
 *  @breif §¯§Ñ§ã§ä§â§à§Û§Ü§Ñ USART2. §±§Ñ§â§Ñ§Þ§Ö§ä§â§í 9600 8 N 1
 ******************************************************************************
 */

void CMSIS_USART2_Init(void) {

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ §¡
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§í§ç §æ§å§ß§Ü§è§Ú§Û

    //§¥§Ý§ñ §Ü§à§ß§æ§Ú§Ô§å§â§Ú§â§à§Ó§Ñ§ß§Ú§Ö §ß§à§Ø§Ö§Ü UART §Õ§Ý§ñ Full Duplex §ß§å§Ø§ß§à §Ú§ã§á§à§Ý§î§Ù§à§Ó§Ñ§ä§î Alternate function push-pull(§³§Þ. §á.§á. 9.1.11 GPIO configurations for device peripherals §ã§ä§â.111 Reference Manual)
    //Tx - Alternative Function output Push-pull(Maximum output speed 50 MHz)
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF2, 0b10 << GPIO_CFGLR_CNF2_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE2, 0b11 << GPIO_CFGLR_MODE2_Pos);
    //Rx - Input floating
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF3, 0b1 << GPIO_CFGLR_CNF3_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE3, 0b00 << GPIO_CFGLR_MODE3_Pos);

    //§©§Ñ§á§å§ã§ä§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö USART2
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_USART2);

    /*§²§Ñ§ã§é§Ö§ä Fractional baud rate generation
     §Ö§ã§ä§î §æ§à§â§Þ§å§Ý§Ñ:
     Tx/Rx baud = fCK/(16*USARTDIV)
     §Ô§Õ§Ö fCK - Input clock to the peripheral (PCLK1 for USART2, 3, 4, 5 or PCLK2 for USART1)
     §Ó §ß§Ñ§ê§Ö§Þ §ã§Ý§å§é§Ñ§Ö fCK = 36000000
     §Õ§à§á§å§ã§ä§Ú§Þ §ß§Ñ§Þ §ß§å§Ø§ß§Ñ §ã§Ü§à§â§à§ã§ä§î 9600
     9600 = 36000000/(16*USARTDIV)
     §´§à§Ô§Õ§Ñ USARTDIV = 36000000/9600*16 = 234.375
     DIV_Mantissa §Ó §Õ§Ñ§ß§ß§à§Þ §ã§Ý§å§é§Ñ§Ö §Ò§å§Õ§Ö§ä 234, §é§ä§à §Ö§ã§ä§î 0xEA
     DIV_Fraction §Ò§å§Õ§Ö§ä, §Ü§Ñ§Ü 0.375*16 = 6, §é§ä§à §Ö§ã§ä§î 0x6
     §´§à§Ô§Õ§Ñ §Ó§Ö§ã§î §â§Ö§Ô§Ú§ã§ä§â USART->BRR §Õ§Ý§ñ §ã§Ü§à§â§à§ã§ä§Ú 9600 §Ò§å§Õ§Ö§ä §Ó§í§Ô§Ý§ñ§Õ§Ö§ä§î, §Ü§Ñ§Ü 0xEA6.
     §Õ§Ý§ñ §á§â§Ú§Þ§Ö§â§Ñ §Ö§ë§Ö §â§Ñ§Ù§Ò§Ö§â§Ö§Þ §ã§Ü§à§â§à§ã§ä§î 115200: (§¯§Ö§ä§à§é§ß§à§ã§ä§î §á§à §ã§Ü§à§â§à§ã§ä§Ú §Ò§å§Õ§Ö§ä 0.15%. §¯§Ö §â§Ö§Ü§à§Þ§Ö§ß§Õ§å§Ö§ä§ã§ñ)
     115200 = 36000000/(16*USARTDIV)
     §´§à§Ô§Õ§Ñ USARTDIV = 36000000/115200*16 = 19.53125
     DIV_Mantissa §Ó §Õ§Ñ§ß§ß§à§Þ §ã§Ý§å§é§Ñ§Ö §Ò§å§Õ§Ö§ä 19, §é§ä§à §Ö§ã§ä§î 0x13
     DIV_Fraction §Ò§å§Õ§Ö§ä, §Ü§Ñ§Ü 0.53125*16 = 8, §é§ä§à §Ö§ã§ä§î 0x8
     §´§à§Ô§Õ§Ñ §Ó§Ö§ã§î §â§Ö§Ô§Ú§ã§ä§â USART->BRR §Õ§Ý§ñ §ã§Ü§à§â§à§ã§ä§Ú 115200 §Ò§å§Õ§Ö§ä §Ó§í§Ô§Ý§ñ§Õ§Ö§ä§î, §Ü§Ñ§Ü 0x138.
     */

    MODIFY_REG(USART2->BRR, USART_BRR_DIV_Mantissa, 0xEA << USART_BRR_DIV_Mantissa_Pos);
    MODIFY_REG(USART2->BRR, USART_BRR_DIV_Fraction, 0x6 << USART_BRR_DIV_Fraction_Pos);

    //18.10.4 USART Control Register1 (USARTx_CTLR1) (x=1/2/3/4/5/6/7/8)
    SET_BIT(USART2->CTLR1, USART_CTLR1_UE);//USART enable
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_M); //Word lenght 1 Start bit, 8 Data bits, n Stop bit
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_WAKE); //Wake up idle Line
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_PCE); //Partity control disabled
    //§ß§Ñ§ã§ä§â§à§Û§Ü§Ñ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Û
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_PEIE);//partity error interrupt disabled
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_TXEIE); //TXE interrupt is inhibited
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_TCIE); //Transmission complete interrupt disabled
    SET_BIT(USART2->CTLR1, USART_CTLR1_RXNEIE); //§±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §á§â§Ú§Ö§Þ§å §Õ§Ñ§ß§ß§í§ç §Ó§Ü§Ý§ð§é§Ö§ß§à
    SET_BIT(USART2->CTLR1, USART_CTLR1_IDLEIE); //§±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §æ§Ý§Ñ§Ô§å IDLE §Ó§Ü§Ý§ð§é§Ö§ß§à
    SET_BIT(USART2->CTLR1, USART_CTLR1_TE); //Transmitter is enabled
    SET_BIT(USART2->CTLR1, USART_CTLR1_RE); //Receiver is enabled and begins searching for a start bit
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_RWU);
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_SBK);

    //§°§ã§ä§Ñ§Ý§î§ß§å§ð §ß§Ñ§ã§ä§â§à§Û§Ü§å, §ß§Ö §Ü§Ñ§ã§Ñ§ð§ë§å§ð§ã§ñ §ã§ä§Ñ§ß§Õ§Ñ§â§ä§ß§à§Ô§à USART, §Þ§í §á§à§Ü§Ñ §ä§â§à§Ô§Ñ§ä§î §ß§Ö §Ò§å§Õ§Ö§Þ, §ß§à §ß§Ñ §Ó§ã§ñ§Ü§Ú§Û §ã§Ý§å§é§Ñ§Û §à§Ò§ß§å§Ý§Ú§Þ
    //18.10.5 USART Control Register2 (USARTx_CTLR2) (x=1/2/3/4/5/6/7/8)
    USART2->CTLR2 = 0;
    CLEAR_BIT(USART2->CTLR2, USART_CTLR2_STOP); //1 §ã§ä§à§á §Ò§Ú§ä.
    //18.10.6 USART Control Register 3 (USARTx_CTLR3) (x=1/2/3/4/5/6/7/8)
    USART2->CTLR3 = 0;
    //18.10.7 USART Guard Time and Prescaler Register (USARTx_GPR) (x=1/2/3/4/5/6/7/8)
    USART2->GPR = 0;

    NVIC_EnableIRQ(USART2_IRQn); //§£§Ü§Ý§ð§é§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ §á§à USART2
}

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à USART1
 ******************************************************************************
 */

__WEAK void USART1_IRQHandler(void) {
    if (READ_BIT(USART1->STATR, USART_STATR_RXNE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§ê§Ý§Ú §Õ§Ñ§ß§ß§í§Ö §á§à USART
        husart1.rx_buffer[husart1.rx_counter] = USART1->DATAR; //§³§é§Ú§ä§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö §Ó §ã§à§à§ä§Ó§Ö§ä§ã§ä§Ó§å§ð§ë§å§ð §ñ§é§Ö§Û§Ü§å §Ó rx_buffer
        husart1.rx_counter++; //§µ§Ó§Ö§Ý§Ú§é§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ß§ñ§ä§í§ç §Ò§Ñ§Û§ä §ß§Ñ 1
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
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à USART1
 ******************************************************************************
 */

__WEAK void USART2_IRQHandler(void) {
    if (READ_BIT(USART2->STATR, USART_STATR_RXNE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§ê§Ý§Ú §Õ§Ñ§ß§ß§í§Ö §á§à USART
        husart2.rx_buffer[husart2.rx_counter] = USART2->DATAR; //§³§é§Ú§ä§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö §Ó §ã§à§à§ä§Ó§Ö§ä§ã§ä§Ó§å§ð§ë§å§ð §ñ§é§Ö§Û§Ü§å §Ó rx_buffer
        husart2.rx_counter++; //§µ§Ó§Ö§Ý§Ú§é§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ß§ñ§ä§í§ç §Ò§Ñ§Û§ä §ß§Ñ 1
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
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §à§ä§á§â§Ñ§Ó§Ü§Ú §Õ§Ñ§ß§ß§í§ç §á§à USART
 *  @param  *USART - USART, §ã §Ü§à§ä§à§â§à§Ô§à §Ò§å§Õ§å§ä §à§ä§á§â§Ñ§Ó§Ý§ñ§ä§î§ã§ñ §Õ§Ñ§ß§ß§í§Ö
 *  @param  *data - §Õ§Ñ§ß§ß§í§Ö, §Ü§à§ä§à§â§í§Ö §Ò§å§Õ§Ö§Þ §à§ä§á§â§Ñ§Ó§Ý§ñ§ä§î
 *  @param  Size - §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §ä§â§Ö§Ò§å§Ö§ä§ã§ñ §á§Ö§â§Ö§Õ§Ñ§ä§î
 ******************************************************************************
 */

bool RVMSIS_USART_Transmit(USART_TypeDef* USART, uint8_t* data, uint16_t Size, uint32_t Timeout_ms) {
    for (uint16_t i = 0; i < Size; i++) {
        Timeout_counter_ms = Timeout_ms;
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ý§Ú§ß§Ú§ñ §ß§Ö §à§ã§Ó§à§Ò§à§Õ§Ú§ä§ã§ñ
        while (READ_BIT(USART->STATR, USART_STATR_TXE) == 0) {
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        USART->DATAR = *data++; //§¬§Ú§Õ§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö
    }
    return true;
}

/*================================= §¯§¡§³§´§²§°§«§¬§¡ I2C ============================================*/

/**
 ***************************************************************************************
 *  @breif Inter-integrated circuit (I2C) interface
 ***************************************************************************************
 */

void RVMSIS_I2C_Reset(void) {
    //§³§Ò§â§à§ã §ß§Ñ§ã§ä§â§à§Ö§Ü I2C
    //19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    SET_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST); //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST) == 0);
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST); //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST));
    /* §±§â§Ú§Þ§Ö§é§Ñ§ß§Ú§Ö: §¿§ä§à§ä §Ò§Ú§ä §Þ§à§Ø§ß§à §Ú§ã§á§à§Ý§î§Ù§à§Ó§Ñ§ä§î §Õ§Ý§ñ §á§à§Ó§ä§à§â§ß§à§Û §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§Ú
     * §á§Ö§â§Ú§æ§Ö§â§Ú§Û§ß§à§Ô§à §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ §á§à§ã§Ý§Ö §à§ê§Ú§Ò§Ü§Ú §Ú§Ý§Ú §Ù§Ñ§Ò§Ý§à§Ü§Ú§â§à§Ó§Ñ§ß§ß§à§Ô§à §ã§à§ã§ä§à§ñ§ß§Ú§ñ.
     * §¯§Ñ§á§â§Ú§Þ§Ö§â, §Ö§ã§Ý§Ú §Ò§Ú§ä BUSY §å§ã§ä§Ñ§ß§à§Ó§Ý§Ö§ß §Ú §à§ã§ä§Ñ§Ö§ä§ã§ñ §Ù§Ñ§Ò§Ý§à§Ü§Ú§â§à§Ó§Ñ§ß§ß§í§Þ §Ú§Ù-§Ù§Ñ §ã§Ò§à§ñ §ß§Ñ §ê§Ú§ß§Ö,
     * §Ò§Ú§ä SWRST §Þ§à§Ø§ß§à §Ú§ã§á§à§Ý§î§Ù§à§Ó§Ñ§ä§î §Õ§Ý§ñ §Ó§í§ç§à§Õ§Ñ §Ú§Ù §ï§ä§à§Ô§à §ã§à§ã§ä§à§ñ§ß§Ú§ñ.*/
}

/**
 *************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§Ú §ê§Ú§ß§í I2C1. Sm.
 *************************************************************************************
 */

void RVMSIS_I2C1_Init(void) {
    //§¯§Ñ§ã§ä§â§à§Û§Ü§Ú §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOB); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ B
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§í§ç §æ§å§ß§Ü§è§Ú§Û
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_I2C1); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ I2C1

    //§¯§Ñ§ã§ä§â§à§Û§Ü§Ú §ß§à§Ø§Ö§Ü SDA §Ú SCL
    //PB7 SDA (I2C Data I/O) Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_CNF7, 0b11 << GPIO_CFGLR_CNF7_Pos);//Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_MODE7, 0b11 << GPIO_CFGLR_MODE7_Pos); //Maximum output speed 50 MHz
    //PB6 SCL (I2C clock) Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_CNF6, 0b11 << GPIO_CFGLR_CNF6_Pos);//Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_MODE6, 0b11 << GPIO_CFGLR_MODE6_Pos); //Maximum output speed 50 MHz

    //19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    RVMSIS_I2C_Reset();

    /*§¿§ä§à §Ó§ã§Ö §Õ§Ý§ñ §Ú§ß§Ú§ä§Ñ §ß§Ö §ß§å§Ø§ß§à. §±§à§ã§Ý§Ö §ã§Ò§â§à§ã§Ñ §Ú§ä§Ñ§Ü §Ò§å§Õ§Ö§ä §Ó 0. */
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
    MODIFY_REG(I2C1->CTLR2, I2C_CTLR2_FREQ, 36 << I2C_CTLR2_FREQ_Pos); //f PCLK1 = 36 §®§Ô§è

    //19.12.3 I2C Address Register 1 (I2Cx_OADDR1) (x=1/2)
    I2C1->OADDR1 = 0;
    //19.12.4 I2C Address Register2 (I2Cx_OADDR2) (x=1/2)
    I2C1->OADDR2 = 0;

    //19.12.8 I2C Clock Register (I2Cx_CKCFGR) (x=1/2)
    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS);//Standard mode I2C
    //SET_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS); //Fast mode I2C

    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_DUTY);//Fm mode tlow/thigh = 2
    //SET_BIT(I2C1->CCR, I2C_CCR_DUTY); //Fm mode tlow/thigh = 16/9 (see CCR)

    //§²§Ñ§ã§é§Ö§ä CCR. §³§Þ§à§ä§â§Ú §á§â§Ú§Þ§Ö§â§í §â§Ñ§ã§é§Ö§ä§Ñ
    MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 180 << I2C_CKCFGR_CCR_Pos);//§Õ§Ý§ñ Sm mode
    //MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 30 << I2C_CKCFGR_CCR_Pos); //§Õ§Ý§ñ Fm mode. DUTY 0.
    //MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 4 << I2C_CKCFGR_CCR_Pos); //§Õ§Ý§ñ Fm mode. DUTY 1.

    //19.12.9 I2C Rise Time Register (I2Cx_RTR) (x=1/2)
    MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 37 << I2C_RTR_TRISE_Pos);//§Õ§Ý§ñ Sm mode
    //MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 12 << I2C_RTR_TRISE_Pos); //§Õ§Ý§ñ Fm mode

    SET_BIT(I2C1->CTLR1, I2C_CTLR1_PE); //I2C1 enable
}

/**
 *************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §ã§Ü§Ñ§ß§Ú§â§à§Ó§Ñ§ß§Ú§ñ §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ §á§à §Ù§Ñ§Õ§Ñ§ß§ß§à§Þ§å 7-§Ò§Ú§ä§ß§à§Þ§å §Ñ§Õ§â§Ö§ã§å
 *  @param  *I2C - §ê§Ú§ß§Ñ I2C
 *  @param  Adress_Device - §¡§Õ§â§Ö§ã §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã true - §Ö§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §á§à §Ù§Ñ§Õ§Ñ§ß§ß§à§Þ§å §Ñ§Õ§â§Ö§ã§å §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î,
 *           false - §Ö§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §á§à §Ù§Ñ§Õ§Ñ§ß§ß§à§Þ§å §Ñ§Õ§â§Ö§ã§å §ß§Ö §à§ä§Ó§Ö§é§Ñ§Ö§ä
 *************************************************************************************
 */
bool RVMSIS_I2C_Adress_Device_Scan(I2C_TypeDef* I2C, uint8_t Adress_Device, uint32_t Timeout_ms) {

    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //§¦§ã§Ý§Ú §ê§Ú§ß§Ñ §Ù§Ñ§ß§ñ§ä§Ñ

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //§¦§ã§Ý§Ú §Ý§Ú§ß§Ú§ñ §ß§Ñ §ã§Ñ§Þ§à§Þ §Õ§Ö§Ý§Ö §ã§Ó§à§Ò§à§Õ§ß§Ñ, §Ñ BUSY §Ó§Ú§ã§Ú§ä
            RVMSIS_I2C_Reset(); //§â§Ö§ã§Ö§ä
            RVMSIS_I2C1_Init(); //§á§à§Ó§ä§à§â§ß§Ñ§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //§¦§ã§Ý§Ú §ã§ä§à§Ú§ä §ã§ä§Ñ§ä§å§ã, §é§ä§à §Þ§í §Ó §Þ§Ñ§ã§ä§Ö§â§Ö
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ú§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        }

        if (I2C->CTLR1 != 1) {
            //§¦§ã§Ý§Ú §Ó CR1 §é§ä§à-§ä§à §Ý§Ú§ê§ß§Ö§Ö, §ä§à §á§Ö§â§Ö§Ù§Ñ§Ô§â§å§Ù§Ú§Þ I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //§¢§Ú§ä ACK §å§á§â§Ñ§Ó§Ý§ñ§Ö§ä (N)ACK §ä§Ö§Ü§å§ë§Ö§Ô§à §Ò§Ñ§Û§ä§Ñ, §á§â§Ú§ß§Ú§Þ§Ñ§Ö§Þ§à§Ô§à §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§°§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ §ã§Ú§Ô§ß§Ñ§Ý START

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //§£§¯§ª§®§¡§¯§ª§¦!
    /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Õ§Ñ§ß§ß§í§ç §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //§¡§Õ§â§Ö§ã + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
        I2C->STAR1;
        I2C->STAR2;
        return true;
    } else {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú §Õ§Ñ§ß§ß§í§ç §á§à I2C
 *  @param  *I2C - §ê§Ú§ß§Ñ I2C
 *  @param  Adress_Device - §¡§Õ§â§Ö§ã §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
 *  @param  *data - §¥§Ñ§ß§ß§í§Ö, §Ü§à§ä§à§â§í§Ö §Ò§å§Õ§Ö§Þ §à§ä§á§â§Ñ§Ó§Ý§ñ§ä§î
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§Þ §à§ä§á§â§Ñ§Ó§Ý§ñ§ä§î.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §à§ä§á§â§Ñ§Ó§Ü§Ú §Õ§Ñ§ß§ß§í§ç. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Transmit(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //§¦§ã§Ý§Ú §ê§Ú§ß§Ñ §Ù§Ñ§ß§ñ§ä§Ñ

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //§¦§ã§Ý§Ú §Ý§Ú§ß§Ú§ñ §ß§Ñ §ã§Ñ§Þ§à§Þ §Õ§Ö§Ý§Ö §ã§Ó§à§Ò§à§Õ§ß§Ñ, §Ñ BUSY §Ó§Ú§ã§Ú§ä
            RVMSIS_I2C_Reset(); //§â§Ö§ã§Ö§ä
            RVMSIS_I2C1_Init(); //§á§à§Ó§ä§à§â§ß§Ñ§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //§¦§ã§Ý§Ú §ã§ä§à§Ú§ä §ã§ä§Ñ§ä§å§ã, §é§ä§à §Þ§í §Ó §Þ§Ñ§ã§ä§Ö§â§Ö
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ú§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        }

        if (I2C->CTLR1 != 1) {
            //§¦§ã§Ý§Ú §Ó CR1 §é§ä§à-§ä§à §Ý§Ú§ê§ß§Ö§Ö, §ä§à §á§Ö§â§Ö§Ù§Ñ§Ô§â§å§Ù§Ú§Þ I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //§¢§Ú§ä ACK §å§á§â§Ñ§Ó§Ý§ñ§Ö§ä (N)ACK §ä§Ö§Ü§å§ë§Ö§Ô§à §Ò§Ñ§Û§ä§Ñ, §á§â§Ú§ß§Ú§Þ§Ñ§Ö§Þ§à§Ô§à §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§³§ä§Ñ§â§ä§å§Ö§Þ.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //§£§¯§ª§®§¡§¯§ª§¦!
    /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //§¡§Õ§â§Ö§ã + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §ã§Ò§â§à§ã§Ú§Þ §Ò§Ú§ä ADDR
        /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*§°§ä§á§â§Ñ§Ó§Ú§Þ §Õ§Ñ§ß§ß§í§Ö*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i); //§©§Ñ§á§Ú§ã§î §Ò§Ñ§Û§ä§Ñ
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Õ§Ñ§ß§ß§í§Ö §Ù§Ñ§Ô§â§å§Ù§ñ§ä§ã§ñ §Ó §â§Ö§Ô§Ú§ã§ä§â §ã§Õ§Ó§Ú§Ô§Ñ.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ

        return true;

    } else {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §á§â§Ú§Ö§Þ§Ñ §Õ§Ñ§ß§ß§í§ç §á§à I2C
 *  @param  *I2C - §ê§Ú§ß§Ñ I2C
 *  @param  Adress_Device - §¡§Õ§â§Ö§ã §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
 *  @param  *data - §¬§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î §á§â§Ú§ß§ñ§ä§í§Ö §Õ§Ñ§ß§ß§í§Ö
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§Þ §á§â§Ú§ß§Ú§Þ§Ñ§ä§î.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §á§â§Ú§Ö§Þ§Ñ §Õ§Ñ§ß§ß§í§ç. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Receive(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //§¦§ã§Ý§Ú §ê§Ú§ß§Ñ §Ù§Ñ§ß§ñ§ä§Ñ

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //§¦§ã§Ý§Ú §Ý§Ú§ß§Ú§ñ §ß§Ñ §ã§Ñ§Þ§à§Þ §Õ§Ö§Ý§Ö §ã§Ó§à§Ò§à§Õ§ß§Ñ, §Ñ BUSY §Ó§Ú§ã§Ú§ä
            RVMSIS_I2C_Reset(); //§â§Ö§ã§Ö§ä
            RVMSIS_I2C1_Init(); //§á§à§Ó§ä§à§â§ß§Ñ§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //§¦§ã§Ý§Ú §ã§ä§à§Ú§ä §ã§ä§Ñ§ä§å§ã, §é§ä§à §Þ§í §Ó §Þ§Ñ§ã§ä§Ö§â§Ö
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ú§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        }

        if (I2C->CTLR1 != 1) {
            //§¦§ã§Ý§Ú §Ó CR1 §é§ä§à-§ä§à §Ý§Ú§ê§ß§Ö§Ö, §ä§à §á§Ö§â§Ö§Ù§Ñ§Ô§â§å§Ù§Ú§Þ I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //§¢§Ú§ä ACK §å§á§â§Ñ§Ó§Ý§ñ§Ö§ä (N)ACK §ä§Ö§Ü§å§ë§Ö§Ô§à §Ò§Ñ§Û§ä§Ñ, §á§â§Ú§ß§Ú§Þ§Ñ§Ö§Þ§à§Ô§à §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§³§ä§Ñ§â§ä§å§Ö§Þ.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //§£§¯§ª§®§¡§¯§ª§¦!
    /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1 | 1); //§¡§Õ§â§Ö§ã + §Ü§à§Þ§Ñ§ß§Õ§Ñ Read

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §ã§Ò§â§à§ã§Ú§Þ §Ò§Ú§ä ADDR
        /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*§±§â§à§é§ä§Ö§Þ §Õ§Ñ§ß§ß§í§Ö*/
        for (uint16_t i = 0; i < Size_data; i++) {
            if (i < Size_data - 1) {
                SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //§¦§ã§Ý§Ú §Þ§í §ç§à§ä§Ú§Þ §á§â§Ú§ß§ñ§ä§î §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §Ò§Ñ§Û§ä, §ä§à §à§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ ACK

                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    //§°§Ø§Ú§Õ§Ñ§Ö§Þ, §á§à§Ü§Ñ §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö §á§à§ñ§Ó§ñ§ä§ã§ñ §Õ§Ñ§ß§ß§í§Ö
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }

                *(data + i) = I2C->DATAR; //§¹§ä§Ö§ß§Ú§Ö §Ò§Ñ§Û§ä§Ñ
            } else {
                CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //§¦§ã§Ý§Ú §Þ§í §Ù§ß§Ñ§Ö§Þ, §é§ä§à §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §á§â§Ú§ß§ñ§ä§í§Û §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§ä §á§à§ã§Ý§Ö§Õ§ß§Ú§Þ, §ä§à §à§ä§á§â§Ñ§Ó§Ú§Þ NACK

                SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    //§°§Ø§Ú§Õ§Ñ§Ö§Þ, §á§à§Ü§Ñ §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö §á§à§ñ§Ó§ñ§ä§ã§ñ §Õ§Ñ§ß§ß§í§Ö
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }
                *(data + i) = I2C->DATAR; //§¹§ä§Ö§ß§Ú§Ö §Ò§Ñ§Û§ä§Ñ
            }
        }
        return true;

    } else {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
        return false;
    }

}

/**
 **************************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §Ù§Ñ§á§Ú§ã§Ú §Ó §á§Ñ§Þ§ñ§ä§î §á§à §å§Ü§Ñ§Ù§Ñ§ß§ß§à§Þ§å §Ñ§Õ§â§Ö§ã§å
 *  @param  *I2C - §ê§Ú§ß§Ñ I2C
 *  @param  Adress_Device - §¡§Õ§â§Ö§ã §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
 *  @param  Adress_data - §¡§Õ§â§Ö§ã §Ó §á§Ñ§Þ§ñ§ä§Ú, §Ü§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö
 *  @param  Size_adress - §²§Ñ§Ù§Þ§Ö§â §Ñ§Õ§â§Ö§ã§Ñ §Ó §Ò§Ñ§Û§ä§Ñ§ç. §±§â§Ú§Þ§Ö§â: 1 - 8 §Ò§Ú§ä§ß§í§Û §Ñ§Õ§â§Ö§ã. 2 - 16 §Ò§Ú§ä§ß§í§Û §Ñ§Õ§â§Ö§ã.
 *  @param  *data - §¥§Ñ§ß§ß§í§Ö, §Ü§à§ä§à§â§í§Ö §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §Ù§Ñ§á§Ú§ã§Ú. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemWrite(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //§¦§ã§Ý§Ú §ê§Ú§ß§Ñ §Ù§Ñ§ß§ñ§ä§Ñ

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //§¦§ã§Ý§Ú §Ý§Ú§ß§Ú§ñ §ß§Ñ §ã§Ñ§Þ§à§Þ §Õ§Ö§Ý§Ö §ã§Ó§à§Ò§à§Õ§ß§Ñ, §Ñ BUSY §Ó§Ú§ã§Ú§ä
            RVMSIS_I2C_Reset(); //§â§Ö§ã§Ö§ä
            RVMSIS_I2C1_Init(); //§á§à§Ó§ä§à§â§ß§Ñ§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //§¦§ã§Ý§Ú §ã§ä§à§Ú§ä §ã§ä§Ñ§ä§å§ã, §é§ä§à §Þ§í §Ó §Þ§Ñ§ã§ä§Ö§â§Ö
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ú§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        }

        if (I2C->CTLR1 != 1) {
            //§¦§ã§Ý§Ú §Ó CR1 §é§ä§à-§ä§à §Ý§Ú§ê§ß§Ö§Ö, §ä§à §á§Ö§â§Ö§Ù§Ñ§Ô§â§å§Ù§Ú§Þ I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //§¢§Ú§ä ACK §å§á§â§Ñ§Ó§Ý§ñ§Ö§ä (N)ACK §ä§Ö§Ü§å§ë§Ö§Ô§à §Ò§Ñ§Û§ä§Ñ, §á§â§Ú§ß§Ú§Þ§Ñ§Ö§Þ§à§Ô§à §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§³§ä§Ñ§â§ä§å§Ö§Þ.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //§£§¯§ª§®§¡§¯§ª§¦!
    /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //§¡§Õ§â§Ö§ã + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §ã§Ò§â§à§ã§Ú§Þ §Ò§Ú§ä ADDR
        /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*§°§ä§á§â§Ñ§Ó§Ú§Þ §Ñ§Õ§â§Ö§ã §á§Ñ§Þ§ñ§ä§Ú*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*) &Adress_data + (Size_adress - 1 - i)); //§©§Ñ§á§Ú§ã§î §Ò§Ñ§Û§ä§Ñ
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Õ§Ñ§ß§ß§í§Ö §Ù§Ñ§Ô§â§å§Ù§ñ§ä§ã§ñ §Ó §â§Ö§Ô§Ú§ã§ä§â §ã§Õ§Ó§Ú§Ô§Ñ.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
                    return false;
                }
            }
        }

        /*§¢§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö §Ó §ñ§é§Ö§Û§Ü§å §á§Ñ§Þ§ñ§ä§Ú, §ß§Ñ§é§Ú§ß§Ñ§ñ §ã §å§Ü§Ñ§Ù§Ñ§ß§ß§à§Ô§à §Ñ§Õ§â§Ö§ã§Ñ*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i); //§©§Ñ§á§Ú§ã§î §Ò§Ñ§Û§ä§Ñ
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Õ§Ñ§ß§ß§í§Ö §Ù§Ñ§Ô§â§å§Ù§ñ§ä§ã§ñ §Ó §â§Ö§Ô§Ú§ã§ä§â §ã§Õ§Ó§Ú§Ô§Ñ.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ

        return true;

    } else {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §é§ä§Ö§ß§Ú§ñ §Ú§Ù §á§Ñ§Þ§ñ§ä§Ú §á§à §å§Ü§Ñ§Ù§Ñ§ß§ß§à§Þ§å §Ñ§Õ§â§Ö§ã§å
 *  @param  *I2C - §ê§Ú§ß§Ñ I2C
 *  @param  Adress_Device - §¡§Õ§â§Ö§ã §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
 *  @param  Adress_data - §¡§Õ§â§Ö§ã §Ó §á§Ñ§Þ§ñ§ä§Ú, §à§ä§Ü§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §ã§é§Ú§ä§í§Ó§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö
 *  @param  Size_adress - §²§Ñ§Ù§Þ§Ö§â §Ñ§Õ§â§Ö§ã§Ñ §Ó §Ò§Ñ§Û§ä§Ñ§ç. §±§â§Ú§Þ§Ö§â: 1 - 8 §Ò§Ú§ä§ß§í§Û §Ñ§Õ§â§Ö§ã. 2 - 16 §Ò§Ú§ä§ß§í§Û §Ñ§Õ§â§Ö§ã.
 *  @param  *data - §¥§Ñ§ß§ß§í§Ö, §Ó §Ü§à§ä§à§â§í§Ö §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î §ã§é§Ú§ä§Ñ§ß§ß§å§ð §Ú§ß§æ§à§â§Þ§Ñ§è§Ú§ð.
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§Þ §ã§é§Ú§ä§í§Ó§Ñ§ä§î.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §ã§é§Ú§ä§í§Ó§Ñ§ß§Ú§ñ. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemRead(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //§¦§ã§Ý§Ú §ê§Ú§ß§Ñ §Ù§Ñ§ß§ñ§ä§Ñ

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //§¦§ã§Ý§Ú §Ý§Ú§ß§Ú§ñ §ß§Ñ §ã§Ñ§Þ§à§Þ §Õ§Ö§Ý§Ö §ã§Ó§à§Ò§à§Õ§ß§Ñ, §Ñ BUSY §Ó§Ú§ã§Ú§ä
            RVMSIS_I2C_Reset(); //§â§Ö§ã§Ö§ä
            RVMSIS_I2C1_Init(); //§á§à§Ó§ä§à§â§ß§Ñ§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //§¦§ã§Ý§Ú §ã§ä§à§Ú§ä §ã§ä§Ñ§ä§å§ã, §é§ä§à §Þ§í §Ó §Þ§Ñ§ã§ä§Ö§â§Ö
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ú§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        }

        if (I2C->CTLR1 != 1) {
            //§¦§ã§Ý§Ú §Ó CR1 §é§ä§à-§ä§à §Ý§Ú§ê§ß§Ö§Ö, §ä§à §á§Ö§â§Ö§Ù§Ñ§Ô§â§å§Ù§Ú§Þ I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //§¢§Ú§ä ACK §å§á§â§Ñ§Ó§Ý§ñ§Ö§ä (N)ACK §ä§Ö§Ü§å§ë§Ö§Ô§à §Ò§Ñ§Û§ä§Ñ, §á§â§Ú§ß§Ú§Þ§Ñ§Ö§Þ§à§Ô§à §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§³§ä§Ñ§â§ä§å§Ö§Þ.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //§£§¯§ª§®§¡§¯§ª§¦!
    /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //§¡§Õ§â§Ö§ã + §Ü§à§Þ§Ñ§ß§Õ§Ñ Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §ã§Ò§â§à§ã§Ú§Þ §Ò§Ú§ä ADDR
        /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*§°§ä§á§â§Ñ§Ó§Ú§Þ §Ñ§Õ§â§Ö§ã §á§Ñ§Þ§ñ§ä§Ú*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*) &Adress_data + (Size_adress - 1 - i)); //§©§Ñ§á§Ú§ã§î §Ò§Ñ§Û§ä§Ñ
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Õ§Ñ§ß§ß§í§Ö §Ù§Ñ§Ô§â§å§Ù§ñ§ä§ã§ñ §Ó §â§Ö§Ô§Ú§ã§ä§â §ã§Õ§Ó§Ú§Ô§Ñ.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
                    return false;
                }
            }
        }

        //§±§à§Ó§ä§à§â§ß§í§Û §ã§ä§Ñ§â§ä
        SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§³§ä§Ñ§â§ä§å§Ö§Þ.

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
            //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

            if (!Timeout_counter_ms) {
                return false;
            }

        }
        //§£§¯§ª§®§¡§¯§ª§¦!
        /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
        I2C->STAR1;
        I2C->DATAR = (Adress_Device << 1 | 1); //§¡§Õ§â§Ö§ã + §Ü§à§Þ§Ñ§ß§Õ§Ñ Read

        Timeout_counter_ms = Timeout_ms;
        while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
            //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

            if (!Timeout_counter_ms) {
                return false;
            }

        }

        if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
            //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §ã§Ò§â§à§ã§Ú§Þ §Ò§Ú§ä ADDR
            /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
            I2C->STAR1;
            I2C->STAR2;

            /*§±§â§à§é§ä§Ö§Þ §Õ§Ñ§ß§ß§í§Ö, §ß§Ñ§é§Ú§ß§Ñ§ñ §ã §å§Ü§Ñ§Ù§Ñ§ß§ß§à§Ô§à §Ñ§Õ§â§Ö§ã§Ñ*/
            for (uint16_t i = 0; i < Size_data; i++) {
                if (i < Size_data - 1) {
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //§¦§ã§Ý§Ú §Þ§í §ç§à§ä§Ú§Þ §á§â§Ú§ß§ñ§ä§î §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §Ò§Ñ§Û§ä, §ä§à §à§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ ACK
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    *(data + i) = I2C->DATAR; //§¹§ä§Ö§ß§Ú§Ö §Ò§Ñ§Û§ä§Ñ
                } else {
                    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //§¦§ã§Ý§Ú §Þ§í §Ù§ß§Ñ§Ö§Þ, §é§ä§à §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §á§â§Ú§ß§ñ§ä§í§Û §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§ä §á§à§ã§Ý§Ö§Õ§ß§Ú§Þ, §ä§à §à§ä§á§â§Ñ§Ó§Ú§Þ NACK

                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    //§±§à§Õ§à§Ø§Õ§Ö§Þ, §á§à§Ü§Ñ §ã§Õ§Ó§Ú§Ô§à§Ó§í§Û §â§Ö§Ô§Ú§ã§ä§â §á§à§á§à§Ý§ß§Ú§ä§ã§ñ §ß§à§Ó§í§Þ §Ò§Ñ§Û§ä§à§Þ §Õ§Ñ§ß§ß§í§ç
                    *(data + i) = I2C->DATAR; //§¹§ä§Ö§ß§Ú§Ö §Ò§Ñ§Û§ä§Ñ
                }
            }
            return true;

        } else {
            //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
            CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
            return false;
        }

    } else {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
        return false;
    }
}

/*================================= §¯§¡§³§´§²§°§«§¬§¡ SPI ============================================*/

/**
***************************************************************************************
*  @breif Serial peripheral interface (SPI)
***************************************************************************************
*/

void RVMSIS_SPI1_init(void) {
    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ GPIO*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§í§ç §æ§å§ß§Ü§è§Ú§Û
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_SPI1); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ SPI1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §á§à§â§ä§Ñ §¡
    /*§¬§Ñ§Ü§Ú§Ö §ß§à§Ø§Ü§Ú:*/
    //PA4 - NSS
    //PA5 - SCK
    //PA6 - MISO
    //PA7 - MOSI
    //§®§í §Ò§å§Õ§Ö§Þ §ß§Ñ§ã§ä§â§Ñ§Ú§Ó§Ñ§ä§î SPI §Ó §â§Ö§Ø§Ú§Þ Master
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
     //§¯§Ñ§ã§ä§â§à§Ú§Þ §ã§Ñ§Þ§Ú §ß§à§Ø§Ü§Ú §å§Ø§Ö §á§à§ã§Ý§Ö §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§Ú SPI, §é§ä§à§Ò §á§â§Ú §ã§ä§Ñ§â§ä§Ö §ß§Ö §Ò§í§Ý§à §Ý§Ú§ê§ß§Ú§ç §ß§à§Ô§à§Õ§Ö§â§Ô§Ñ§ß§Ú§Û.

    //20.4.1 SPI Control Register 1 (SPIx_CTLR1) (x=1/2/3)
    MODIFY_REG(SPI1->CTLR1, SPI_CTLR1_BR, 0b011 << SPI_CTLR1_BR_Pos); //fPCLK/4. 72000000/32 = 2.22 MBits/s
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_CPOL); //§±§à§Ý§ñ§â§ß§à§ã§ä§î
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_CPHA); //§¶§Ñ§Ù§Ñ
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_DFF); //0: 8-bit data frame format is selected for transmission/reception
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_LSBFIRST); //0: MSB transmitted first
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_SSM); //1: Software slave management enabled
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_SSI); //1: Software slave management enabled
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_MSTR); //1: Master configuration
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_BIDIMODE); //0: 2-line unidirectional data mode selected
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_RXONLY); //0: Full duplex (Transmit and receive)

    SET_BIT(SPI1->CTLR1, SPI_CTLR1_SPE); //§£§Ü§Ý§ð§é§Ú§Þ SPI

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
    CLEAR_BIT(SPI1->I2SCFGR, SPI_I2SCFGR_I2SMOD); //§ä.§Ü. §ß§Ñ F103C6T6 §ß§Ö§ä I2S, §Ö§Ô§à §Ó§í§â§Ö§Ù§Ñ§Ý§Ú, §Ñ §â§Ö§Ô§Ú§ã§ä§â §à§ã§ä§Ñ§Ó§Ú§Ý§Ú, §ß§å§Ø§ß§à §á§â§à§ã§ä§à §à§Ò§ß§å§Ý§Ú§ä§î §Õ§Ñ§ß§ß§í§Û §â§Ö§Ô§Ú§ã§ä§â. §´§Ö§Þ §ã§Ñ§Þ§í§Þ §Ó§Ü§Ý§ð§é§Ú§Þ §â§Ö§Ø§Ú§Þ SPI mode.


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
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú §Õ§Ñ§ß§ß§í§ç §á§à §ê§Ú§ß§Ö SPI
 *  @param  *SPI - §ê§Ú§ß§Ñ SPI
 *  @param  *data - §¥§Ñ§ß§ß§í§Ö, §Ü§à§ä§à§â§í§Ö §Ò§å§Õ§Ö§Þ §á§Ö§â§Ö§Õ§Ñ§Ó§Ñ§ä§î.
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§Þ §á§Ö§â§Ö§Õ§Ñ§Ó§Ñ§ä§î.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §á§Ö§â§Ö§Õ§Ñ§é§Ú. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Transmit_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    //(§ã§Þ. Reference Manual §ã§ä§â. 712 Transmit-only procedure (BIDIMODE=0 RXONLY=0))
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //§±§â§à§Ó§Ö§â§Ú§Þ §Ù§Ñ§ß§ñ§ä§à§ã§ä§î §ê§Ú§ß§í
        SPI->DATAR = *(data); //§©§Ñ§á§Ú§ê§Ö§Þ §á§Ö§â§Ó§í§Û §ï§Ý§Ö§Þ§Ö§ß§ä §Õ§Ñ§ß§ß§í§ç §Õ§Ý§ñ §à§ä§á§â§Ñ§Ó§Ü§Ú §Ó §â§Ö§Ô§Ú§ã§ä§â SPI_DR
        //(§±§â§Ú §ï§ä§à§Þ §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §Ò§Ú§ä TXE)

        for (uint16_t i = 1; i < Size_data; i++) {
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ò§å§æ§Ö§â §ß§Ñ §á§Ö§â§Ö§Õ§Ñ§é§å §ß§Ö §à§ã§Ó§à§Ò§à§Õ§Ú§ä§ã§ñ
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            SPI->DATAR = *(data + i); //§©§Ñ§á§Ú§ê§Ö§Þ §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §ï§Ý§Ö§Þ§Ö§ß§ä §Õ§Ñ§ß§ß§í§ç.
        }
        Timeout_counter_ms = Timeout_ms;
        while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
            //§±§à§ã§Ý§Ö §Ù§Ñ§á§Ú§ã§Ú §á§à§ã§Ý§Ö§Õ§ß§Ö§Ô§à §ï§Ý§Ö§Þ§Ö§ß§ä§Ñ §Õ§Ñ§ß§ß§í§ç §Ó §â§Ö§Ô§Ú§ã§ä§â SPI_DR,
            //§á§à§Õ§à§Ø§Õ§Ö§Þ, §á§à§Ü§Ñ TXE §ã§ä§Ñ§ß§Ö§ä §â§Ñ§Ó§ß§í§Þ 1.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //§©§Ñ§ä§Ö§Þ §á§à§Õ§à§Ø§Õ§Ö§Þ, §á§à§Ü§Ñ BSY §ã§ä§Ñ§ß§Ö§ä §â§Ñ§Ó§ß§í§Þ 0.
            //§¿§ä§à §å§Ü§Ñ§Ù§í§Ó§Ñ§Ö§ä §ß§Ñ §ä§à, §é§ä§à §á§Ö§â§Ö§Õ§Ñ§é§Ñ §á§à§ã§Ý§Ö§Õ§ß§Ú§ç §Õ§Ñ§ß§ß§í§ç §Ù§Ñ§Ó§Ö§â§ê§Ö§ß§Ñ.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
    //§±§â§Ú§Þ§Ö§é§Ñ§ß§Ú§Ö:
    //§±§à§ã§Ý§Ö §á§Ö§â§Ö§Õ§Ñ§é§Ú §Õ§Ó§å§ç §ï§Ý§Ö§Þ§Ö§ß§ä§à§Ó §Õ§Ñ§ß§ß§í§ç §Ó §â§Ö§Ø§Ú§Þ§Ö "transmit-only mode" §Ó §â§Ö§Ô§Ú§ã§ä§â§Ö SPI_SR §å§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§ä§ã§ñ §æ§Ý§Ñ§Ô OVR, §ä§Ñ§Ü §Ü§Ñ§Ü §á§â§Ú§ß§ñ§ä§í§Ö §Õ§Ñ§ß§ß§í§Ö §ß§Ú§Ü§à§Ô§Õ§Ñ §ß§Ö §ã§é§Ú§ä§í§Ó§Ñ§ð§ä§ã§ñ.
}

/**
 **************************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú §Õ§Ñ§ß§ß§í§ç §á§à §ê§Ú§ß§Ö SPI
 *  @param  *SPI - §ê§Ú§ß§Ñ SPI
 *  @param  *data - §¥§Ñ§ß§ß§í§Ö, §Ü§à§ä§à§â§í§Ö §Ò§å§Õ§Ö§Þ §á§Ö§â§Ö§Õ§Ñ§Ó§Ñ§ä§î.
 *  @param  Size_data - §ã§Ü§à§Ý§î§Ü§à 16 - §Ò§Ú§ä§ß§í§ç §Õ§Ñ§ß§ß§í§ç §ç§à§ä§Ú§Þ §á§Ö§â§Ö§Õ§Ñ§ä§î. §´.§Ö. 1 = 2 §Ò§Ñ§Û§ä§Ñ.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §á§Ö§â§Ö§Õ§Ñ§é§Ú. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Transmit_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    //(§ã§Þ. Reference Manual §ã§ä§â. 712 Transmit-only procedure (BIDIMODE=0 RXONLY=0))
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //§±§â§à§Ó§Ö§â§Ú§Þ §Ù§Ñ§ß§ñ§ä§à§ã§ä§î §ê§Ú§ß§í
        SPI->DATAR = *(data); //§©§Ñ§á§Ú§ê§Ö§Þ §á§Ö§â§Ó§í§Û §ï§Ý§Ö§Þ§Ö§ß§ä §Õ§Ñ§ß§ß§í§ç §Õ§Ý§ñ §à§ä§á§â§Ñ§Ó§Ü§Ú §Ó §â§Ö§Ô§Ú§ã§ä§â SPI_DR
        //(§±§â§Ú §ï§ä§à§Þ §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §Ò§Ú§ä TXE)

        for (uint16_t i = 1; i < Size_data; i++) {
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ò§å§æ§Ö§â §ß§Ñ §á§Ö§â§Ö§Õ§Ñ§é§å §ß§Ö §à§ã§Ó§à§Ò§à§Õ§Ú§ä§ã§ñ
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            SPI->DATAR = *(data + i); //§©§Ñ§á§Ú§ê§Ö§Þ §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §ï§Ý§Ö§Þ§Ö§ß§ä §Õ§Ñ§ß§ß§í§ç.
        }
        Timeout_counter_ms = Timeout_ms;
        while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
            //§±§à§ã§Ý§Ö §Ù§Ñ§á§Ú§ã§Ú §á§à§ã§Ý§Ö§Õ§ß§Ö§Ô§à §ï§Ý§Ö§Þ§Ö§ß§ä§Ñ §Õ§Ñ§ß§ß§í§ç §Ó §â§Ö§Ô§Ú§ã§ä§â SPI_DR,
            //§á§à§Õ§à§Ø§Õ§Ö§Þ, §á§à§Ü§Ñ TXE §ã§ä§Ñ§ß§Ö§ä §â§Ñ§Ó§ß§í§Þ 1.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //§©§Ñ§ä§Ö§Þ §á§à§Õ§à§Ø§Õ§Ö§Þ, §á§à§Ü§Ñ BSY §ã§ä§Ñ§ß§Ö§ä §â§Ñ§Ó§ß§í§Þ 0.
            //§¿§ä§à §å§Ü§Ñ§Ù§í§Ó§Ñ§Ö§ä §ß§Ñ §ä§à, §é§ä§à §á§Ö§â§Ö§Õ§Ñ§é§Ñ §á§à§ã§Ý§Ö§Õ§ß§Ú§ç §Õ§Ñ§ß§ß§í§ç §Ù§Ñ§Ó§Ö§â§ê§Ö§ß§Ñ.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
    //§±§â§Ú§Þ§Ö§é§Ñ§ß§Ú§Ö:
    //§±§à§ã§Ý§Ö §á§Ö§â§Ö§Õ§Ñ§é§Ú §Õ§Ó§å§ç §ï§Ý§Ö§Þ§Ö§ß§ä§à§Ó §Õ§Ñ§ß§ß§í§ç §Ó §â§Ö§Ø§Ú§Þ§Ö "transmit-only mode" §Ó §â§Ö§Ô§Ú§ã§ä§â§Ö SPI_SR §å§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§ä§ã§ñ §æ§Ý§Ñ§Ô OVR, §ä§Ñ§Ü §Ü§Ñ§Ü §á§â§Ú§ß§ñ§ä§í§Ö §Õ§Ñ§ß§ß§í§Ö §ß§Ú§Ü§à§Ô§Õ§Ñ §ß§Ö §ã§é§Ú§ä§í§Ó§Ñ§ð§ä§ã§ñ.
}

/**
 **************************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §á§â§Ú§Ö§Þ§Ñ §Õ§Ñ§ß§ß§í§ç §á§à §ê§Ú§ß§Ö SPI
 *  @param  *SPI - §ê§Ú§ß§Ñ SPI
 *  @param  *data - §¥§Ñ§ß§ß§í§Ö, §Ü§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î §á§â§Ú§ß§ñ§ä§í§Ö §Õ§Ñ§ß§ß§í§Ö.
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §ç§à§ä§Ú§Þ §á§â§Ú§ß§ñ§ä§î.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §á§â§Ú§Ö§Þ§Ñ. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Receive_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //§±§â§à§Ó§Ö§â§Ú§Þ §Ù§Ñ§ß§ñ§ä§à§ã§ä§î §ê§Ú§ß§í

        if (READ_BIT(SPI->STATR, SPI_STATR_OVR) || READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
            //§´.§Ü. §Þ§í §Þ§à§Ø§Ö§Þ §á§â§Ú§ß§Ú§Þ§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö §Ó §Ý§ð§Ò§à§Û §Þ§à§Þ§Ö§ß§ä, §ß§Ñ§á§â§Ú§Þ§Ö§â §á§à§ã§Ý§Ö §â§Ö§Ø§Ú§Þ§Ñ "transmit-only mode"
            //§ä§à §ã§Ý§Ö§Õ§å§Ö§ä §á§â§à§Ó§Ö§â§Ú§ä§î §ã§ä§Ñ§ä§å§ã§í OVR §Ú RXNE. §¦§ã§Ý§Ú §ç§à§ä§ñ §Ò§í §à§Õ§Ú§ß §Ú§Ù §ß§Ú§ç §å§ã§ä§Ñ§ß§à§Ó§Ý§Ö§ß,
            //§ä§à §ã§Ò§â§à§ã§Ú§Þ §Ú§ç §á§â§Ú §á§à§Þ§à§ë§Ú §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ DR.
            SPI->DATAR;
        }

        //§¯§Ñ§é§ß§Ö§Þ §á§â§Ú§Ö§Þ §Õ§Ñ§ß§ß§í§ç
        for (uint16_t i = 0; i < Size_data; i++) {
            SPI->DATAR = 0; //§©§Ñ§á§å§ã§ä§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö, §é§ä§à§Ò §ã§é§Ú§ä§Ñ§ä§î 8 §Ò§Ú§ä
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ò§å§æ§Ö§â §ß§Ñ §á§â§Ú§Ö§Þ §ß§Ö §Ù§Ñ§á§à§Ý§ß§Ú§ä§ã§ñ
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            *(data + i) = SPI->DATAR; //§³§é§Ú§ä§í§Ó§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö
        }

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //§©§Ñ§ä§Ö§Þ §á§à§Õ§à§Ø§Õ§Ö§Þ, §á§à§Ü§Ñ BSY §ã§ä§Ñ§ß§Ö§ä §â§Ñ§Ó§ß§í§Þ 0.
            //§¿§ä§à §å§Ü§Ñ§Ù§í§Ó§Ñ§Ö§ä §ß§Ñ §ä§à, §é§ä§à §á§â§Ú§Ö§Þ §á§à§ã§Ý§Ö§Õ§ß§Ú§ç §Õ§Ñ§ß§ß§í§ç §Ù§Ñ§Ó§Ö§â§ê§Ö§ß.
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
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §á§â§Ú§Ö§Þ§Ñ §Õ§Ñ§ß§ß§í§ç §á§à §ê§Ú§ß§Ö SPI
 *  @param  *SPI - §ê§Ú§ß§Ñ SPI
 *  @param  *data - §¥§Ñ§ß§ß§í§Ö, §Ü§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î §á§â§Ú§ß§ñ§ä§í§Ö §Õ§Ñ§ß§ß§í§Ö.
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à 16 - §Ò§Ú§ä§ß§í§ç §Õ§Ñ§ß§ß§í§ç §ç§à§ä§Ú§Þ §á§â§Ú§ß§ñ§ä§î. §´.§Ö. 1 = 2 §Ò§Ñ§Û§ä§Ñ.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §á§â§Ú§Ö§Þ§Ñ. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Receive_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //§±§â§à§Ó§Ö§â§Ú§Þ §Ù§Ñ§ß§ñ§ä§à§ã§ä§î §ê§Ú§ß§í

        if (READ_BIT(SPI->STATR, SPI_STATR_OVR) || READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
            //§´.§Ü. §Þ§í §Þ§à§Ø§Ö§Þ §á§â§Ú§ß§Ú§Þ§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö §Ó §Ý§ð§Ò§à§Û §Þ§à§Þ§Ö§ß§ä, §ß§Ñ§á§â§Ú§Þ§Ö§â §á§à§ã§Ý§Ö §â§Ö§Ø§Ú§Þ§Ñ "transmit-only mode"
            //§ä§à §ã§Ý§Ö§Õ§å§Ö§ä §á§â§à§Ó§Ö§â§Ú§ä§î §ã§ä§Ñ§ä§å§ã§í OVR §Ú RXNE. §¦§ã§Ý§Ú §ç§à§ä§ñ §Ò§í §à§Õ§Ú§ß §Ú§Ù §ß§Ú§ç §å§ã§ä§Ñ§ß§à§Ó§Ý§Ö§ß,
            //§ä§à §ã§Ò§â§à§ã§Ú§Þ §Ú§ç §á§â§Ú §á§à§Þ§à§ë§Ú §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ DR.
            SPI->DATAR;
        }

        //§¯§Ñ§é§ß§Ö§Þ §á§â§Ú§Ö§Þ §Õ§Ñ§ß§ß§í§ç
        for (uint16_t i = 0; i < Size_data; i++) {
            SPI->DATAR = 0; //§©§Ñ§á§å§ã§ä§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö, §é§ä§à§Ò §ã§é§Ú§ä§Ñ§ä§î 16 §Ò§Ú§ä
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ò§å§æ§Ö§â §ß§Ñ §á§â§Ú§Ö§Þ §ß§Ö §Ù§Ñ§á§à§Ý§ß§Ú§ä§ã§ñ
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            *(data + i) = SPI->DATAR; //§³§é§Ú§ä§í§Ó§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö
        }

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //§©§Ñ§ä§Ö§Þ §á§à§Õ§à§Ø§Õ§Ö§Þ, §á§à§Ü§Ñ BSY §ã§ä§Ñ§ß§Ö§ä §â§Ñ§Ó§ß§í§Þ 0.
            //§¿§ä§à §å§Ü§Ñ§Ù§í§Ó§Ñ§Ö§ä §ß§Ñ §ä§à, §é§ä§à §á§â§Ú§Ö§Þ §á§à§ã§Ý§Ö§Õ§ß§Ú§ç §Õ§Ñ§ß§ß§í§ç §Ù§Ñ§Ó§Ö§â§ê§Ö§ß.
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
