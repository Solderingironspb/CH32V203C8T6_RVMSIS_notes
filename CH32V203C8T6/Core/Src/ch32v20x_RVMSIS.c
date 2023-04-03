/**
 ******************************************************************************
 *  @file ch32v20x_RVMSIS.h
 *  @brief RVMSIS �ߧ� ���ڧާ֧�� ���� CH32V203C8T6
 *  @author ����ݧܧ�� ���ݧ֧�
 *  @date 31.03.2023
 *
 ******************************************************************************
 * @attention
 *
 *  ���ڧҧݧڧ��֧ܧ� ���ާ�ԧѧ֧� ��ѧ٧�ҧ�ѧ���� �� �ҧڧҧݧڧ��֧ܧ�� RVMSIS �ߧ� ���ڧާ֧��
 *  ���� CH32V203C8T6
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/STM32F103C8T6_CMSIS_notes
 *  �������� ����: https://vk.com/solderingiron.stm32
 *  ���ѧҧ��ѧ� ��� �է�ܧ�ާ֧ߧ�ѧ�ڧ�: http://www.wch-ic.com/products/CH32V203.html?
 *
 ******************************************************************************
 */
#include "ch32v20x_RVMSIS.h"

/*================================= ������������������ DEBUG ============================================*/
/**
 ***************************************************************************************
 *  @breif Debug port mapping
 ***************************************************************************************
 */
void RVMSIS_Debug_init(void) {
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ѧ����ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ A
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //���ѧ����ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� �ѧݧ��֧�ߧѧ�ڧӧߧ�� ���ߧܧ�ڧ�
    MODIFY_REG(AFIO->PCFR1, AFIO_PCFR1_SWJ_CFG, 0b000 << AFIO_PCFR1_SWJ_CFG_Pos); //Serial wire

    /**
     *  ����� �ӧ�ҧ��� Serial wire:
     *  PA13 /JTMS/SWDIO
     *  PA14 /JTCK/SWCLK.
     *  PA15, PB3 �� PB4 ��ӧ�ҧ�էߧ�
     */
    /*���ѧҧݧ�ܧڧ��֧� �է����� �էݧ� ��֧էѧܧ�ڧ��ӧѧߧڧ� �ܧ�ߧ�ڧԧ��ѧ�ڧ� PA13 �� PA14*/

    GPIOA->LCKR = GPIO_LCKK | GPIO_LCK13 | GPIO_LCK14;
    GPIOA->LCKR = GPIO_LCK13 | GPIO_LCK14;
    GPIOA->LCKR = GPIO_LCKK | GPIO_LCK13 | GPIO_LCK14;
    GPIOA->LCKR;
}

/*============================== ������������������ RCC =======================================*/
/**
 ***************************************************************************************
 *  @breif ���ѧ����ۧܧ� ���� CH32V203C8T6 �ߧ� ��ѧ����� 144MHz ��� �ӧߧ֧�ߧ֧ԧ� �ܧӧѧ��֧ӧ�ԧ� ��֧٧�ߧѧ����
 *  P.S. �ާ�ا֧�� �ܧڧէѧ�� �� �ާ֧ߧ� �ܧѧާߧ�ާ�, �ߧ� �� ���ڧ�ѧ�, ���� ��� 144������ ���� ���ݧ�٧�, �ܧѧ�
 *  ��� �ܧ�٧ݧ� �ާ�ݧ�ܧ�) I2C1 �� I2C2 �ߧ� APB1. ���ݧ� �ߧڧ� �ާѧܧ�ڧާ�� 36 ������.
 *  ���ܧ������ FLASH �ާѧܧ�ڧާ�� 60 ������, �� �է֧ݧڧ�֧ݧ� ��ѧ� �֧��� ���ݧ�ܧ� �ߧ� 2, ���� ������
 *  �ߧ� �էѧ֧� �ߧѧ� �ӧ���ѧӧڧ�� 144 ������. �� �֧�ݧ� �ާ� ����ڧ� ��ѧҧ��ѧ�� �� ADC, ��� ������ �ا�, �ߧ� APB2
 *  �ާ� �ߧ� �ާ�ا֧� �ӧ���ѧӧڧ�� 144 ������, ��.��. �ާѧܧ�ڧާѧݧ�ߧ�� �է֧ݧڧ�֧ݧ� 8(144/8 = 18������.)
 *  �� �ާѧܧ�ڧާ�� �էݧ� ������ - 14 ������...�������ާ� �էݧ� ��ߧڧӧ֧��ѧݧ�ߧ�� ��ѧҧ��� �ӧ�֧ԧ�, ��ѧ����� �٧ѧէڧ�ѧ��
 *  �ӧ��� 72 ������ �ߧ� �ҧ�է֧�.
 *  ���ߧ֧�ߧڧ� �ܧӧѧ��֧ӧ�� ��֧٧�ߧѧ��� �ߧ� 8 MHz
 *  ADC �ߧѧ����֧� �ߧ� 12MHz
 *  USB �ߧѧ����֧� �ߧ� 48MHz
 *  MCO ���էܧݧ��֧� �� HSE �� ��ѧܧ�ڧ��֧��� ��� 8MHz
 *
 ***************************************************************************************
 */
void RVMSIS_RCC_SystemClock_144MHz(void) {
    SET_BIT(RCC->CTLR, RCC_HSION); //���ѧ����ڧ� �ӧߧ���֧ߧߧڧ� RC �ԧ֧ߧ֧�ѧ��� �ߧ� 8 ������
    while (READ_BIT(RCC->CTLR, RCC_HSIRDY) == 0);
    //����اէ֧ާ�� ���էߧ��ڧ� ��ݧѧԧ� �� �ԧ���ӧߧ����
    CLEAR_BIT(RCC->CTLR, RCC_HSEBYP);//�������� ��ҧ���ڧ� ����� �ҧڧ� �� 0(������ �ڧ٧ߧѧ�ѧݧ�ߧ� ��� �� ��ѧ� �է�ݧا֧� �ҧ��� �� 0).
    SET_BIT(RCC->CTLR, RCC_HSEON); //���ѧ����ڧ� �ӧߧ֧�ߧڧ� �ܧӧѧ��֧ӧ�� ��֧٧�ߧѧ���. ���� �� �ߧѧ� �ߧ� 8 MHz.
    while (READ_BIT(RCC->CTLR, RCC_HSERDY) == 0);
    //����اէ֧ާ�� ���էߧ��ڧ� ��ݧѧԧ� �� �ԧ���ӧߧ����
    SET_BIT(RCC->CTLR, RCC_CSSON);//���ܧݧ��ڧ� CSS
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b01 << RCC_SW_Pos); //����ҧ֧�֧� HSE �� �ܧѧ�֧��ӧ� System Clock(PLL �ݧ���� ���ܧ� �ߧ� �ӧ�ҧڧ�ѧ��, ��� �� �ߧѧ� ���ܧݧ��֧�)
    CLEAR_BIT(RCC->CTLR, RCC_PLLON); //����ܧݧ��ڧ� PLL
    MODIFY_REG(RCC->CFGR0, RCC_HPRE, RCC_HPRE_DIV2); //AHB prescaler /2
    //Note: FLASH access clock frequency cannot
    //be more than 60 MHz.
    CLEAR_BIT(FLASH->CTLR, 1 << 25U); //0: FLASH access clock frequency = SYSCLK/2.
    MODIFY_REG(RCC->CFGR0, RCC_PPRE1, RCC_PPRE1_DIV2); //APB1 Prescaler /2,  72/2 = 36 MHz.(���ߧѧ�� �� �ߧѧ� I2C �ҧ�է֧� �ߧ� �ߧѧ����ڧ��...)
    MODIFY_REG(RCC->CFGR0, RCC_PPRE2, RCC_PPRE2_DIV1); //APB2 Prescaler /1. ����է֧� 72MHz.
    MODIFY_REG(RCC->CFGR0, RCC_ADCPRE, RCC_ADCPRE_DIV6); //ADC Prescaler /6, ����� �ҧ�ݧ� 12MHz, ��.��. �ާѧܧ�ڧާѧݧ�ߧѧ� ��ѧ����� ���� 14 MHz
    CLEAR_BIT(RCC->CFGR0, RCC_PLLXTPRE); //0: HSE clock not divided.
    SET_BIT(RCC->CFGR0, RCC_PLLSRC); //�� �ܧѧ�֧��ӧ� �ӧ��էߧ�ԧ� ��ڧԧߧѧݧ� �էݧ� PLL �ӧ�ҧ֧�֧� HSE
    MODIFY_REG(RCC->CFGR0, RCC_PLLMULL, 0b1111 << RCC_PLLMULL_Pos); //��.��. �ܧӧѧ�� �� �ߧѧ� 8Mhz, �� �ߧѧ� �ߧ�اߧ� 144MHz, ��� �� PLL �ߧ�اߧ� ��է֧ݧѧ�� ��ާߧ�ا֧ߧڧ� �ߧ� 18. 8MHz * 18 = 144MHz.
    MODIFY_REG(RCC->CFGR0, RCC_USBPRE, 0b10 << RCC_USBPRE_Pos); //10: Divided by 3 (when PLLCLK=144MHz); 144/3 = 48 ������
    MODIFY_REG(RCC->CFGR0, RCC_CFGR0_MCO, RCC_CFGR0_MCO_HSE); //�� �ܧѧ�֧��ӧ� ��ѧܧ�ڧ��ӧѧߧڧ� �էݧ� MCO �ӧ�ҧ�ѧ� HSE. ����է֧� 8 MHz.
    SET_BIT(RCC->CTLR, RCC_PLLON); //���ѧ����ڧ� PLL

    //��.��. PLL ��ا� �٧ѧ���֧�, �ӧ�ҧ֧�֧� �֧ԧ� �� �ܧѧ�֧��ӧ� System Clock:
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b10 << RCC_SW_Pos);//����ҧ֧�֧� PLL �� �ܧѧ�֧��ӧ� System Clock
    while (READ_BIT(RCC->CTLR, RCC_PLLRDY) == 0);
    //����اڧէ֧ާ�� ���էߧ��ڧ� ��ݧѧԧ� �ӧܧݧ��֧ߧڧ� PLL

}

/*========================= ������������������ �������������������� �������������� ==============================*/

/**
 ***************************************************************************************
 *  @breif ���ѧ����ۧܧ� SysTick �ߧ� �ާڧܧ���֧ܧ�ߧէ�
 *  ���� ����� ��ѧۧާ֧�� �ާ� �ߧѧ����ڧ� Delay �� �ѧߧѧݧ�� HAL_GetTick()
 ***************************************************************************************
 */
void RVMSIS_SysTick_Timer_init(void) {
    SysTick->CTLR &= ~(1 << 0); //����ܧݧ��ڧ� ��ѧۧާ֧� �էݧ� ����ӧ֧է֧ߧڧ� �ߧѧ����֧�.
    SysTick->CTLR |= (1 << 1); //1: Enable counter interrupts.
    SysTick->CTLR &= ~(1 << 2); //0: HCLK for time base.144/8 =18
    SysTick->CTLR |= (1 << 3); //1: Re-counting from 0 after counting up to the comparison value, and re-counting from the comparison value after counting down to 0
    SysTick->CTLR |= (1 << 4); //0: Counting up.
    SysTick->CTLR |= (1 << 5); //1: Updated to 0 on up counts, updated to the comparison value on down counts.
    SysTick->CMP = 8999; ////���ѧ����ڧ� ���֧��ӧѧߧڧ� �ߧ� ��ѧ����� �� 1 �ܧ���(��.��. ���ѧҧ��ܧ� �ҧ�է֧� �ܧѧاէ�� �ާ�) 18000000 / 18000 = 1000����
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->CTLR |= (1 << 0); //���ѧ����ڧ� ��ѧۧާ֧�.
}

/**
 ***************************************************************************************
 *  @breif ���ѧ����ۧܧ� Delay �� �ѧߧѧݧ�� HAL_GetTick()
 ***************************************************************************************
 */
volatile uint32_t SysTimer_ms = 0; //���֧�֧ާ֧ߧߧѧ�, �ѧߧѧݧ�ԧڧ�ߧѧ� HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0; //����֧��ڧ� �էݧ� ���ߧܧ�ڧ� Delay_ms
volatile uint32_t Timeout_counter_ms = 0; //���֧�֧ާ֧ߧߧѧ� �էݧ� ��ѧۧާѧ��� ���ߧܧ�ڧ�

/**
 ******************************************************************************
 *  @breif ����֧��ӧѧߧڧ� ��� ��ݧѧԧ� CNTIF
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
 *  @param   uint32_t Milliseconds - ���ݧڧߧ� �٧ѧէ֧�اܧ� �� �ާڧݧݧڧ�֧ܧ�ߧէѧ�
 ******************************************************************************
 */
void Delay_ms(uint32_t Milliseconds) {
    Delay_counter_ms = Milliseconds;
    while (Delay_counter_ms != 0);
}

/*============================== ������������������ GPIO =======================================*/

/**
 ***************************************************************************************
 *  @breif ���ߧڧ�ڧѧݧڧ٧ѧ�ڧ� PIN PC13 �ߧ� �ӧ���� �� ��֧اڧާ� Push-Pull �� �ާѧܧ�ڧާѧݧ�ߧ�� ��ܧ������� 50 MHz
 *  ���֧�֧� �ߧѧ����ۧܧ�� (GPIOs and AFIOs) �ߧ�اߧ� �ӧܧݧ��ڧ�� ��ѧܧ�ڧ��ӧѧߧڧ� ������.
 ***************************************************************************************
 */
void RVMSIS_PC13_OUTPUT_Push_Pull_init(void) {
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //���ѧ����ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ C
    MODIFY_REG(GPIOC->CFGHR, GPIO_CFGHR_MODE13, 0b10 << GPIO_CFGHR_MODE13_Pos); //���ѧ����ۧܧ� GPIOC ������ 13 �ߧ� �ӧ���� ��� �ާѧܧ�ڧާѧݧ�ߧ�� ��ܧ������� �� 50 MHz
    MODIFY_REG(GPIOC->CFGHR, GPIO_CFGHR_CNF13, 0b00 << GPIO_CFGHR_CNF13_Pos); //���ѧ����ۧܧ� GPIOC ������ 13 �ߧ� �ӧ���� �� ��֧اڧާ� Push-Pull

}

/**
 ***************************************************************************************
 *  @breif Blink PIN PC13 �ߧ� �ӧ���� �� ��֧اڧާ� Push-Pull
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
 *  @breif ���ѧ����ۧܧ� MCO c �ӧ���է�� �ߧ� �ߧ�اܧ� PA8
 *  ���֧�֧� �ߧѧ����ۧܧ�� (GPIOs and AFIOs) �ߧ�اߧ� �ӧܧݧ��ڧ�� ��ѧܧ�ڧ��ӧѧߧڧ� ������.
 ***************************************************************************************
 */
void RVMSIS_PA8_MCO_init(void) {
    //���ѧܧ�ڧ��ӧѧߧڧ� MCO �է�ݧاߧ� �ҧ��� �ߧѧ����֧ߧ� �� ��֧ԧڧ���� RCC
    MODIFY_REG(RCC->CFGR0, RCC_CFGR0_MCO, RCC_CFGR0_MCO_SYSCLK); //�� �ܧѧ�֧��ӧ� ��ѧܧ�ڧ��ӧѧߧڧ� �էݧ� MCO �ӧ�ҧ�ѧ� HSE. ����է֧� 8 MHz.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ѧ����ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ A
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_MODE8, 0b11 << GPIO_CFGHR_MODE8_Pos); //���ѧ����ۧܧ� GPIOA ������ 8 �ߧ� �ӧ���� ��� �ާѧܧ�ڧާѧݧ�ߧ�� ��ܧ������� �� 50 MHz
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_CNF8, 0b10 << GPIO_CFGHR_CNF8_Pos); //���ѧ����ۧܧ� GPIOA ������ 8, �ܧѧ� �ѧݧ��֧�ߧѧ�ڧӧߧѧ� ���ߧܧ�ڧ�, �� ��֧اڧާ� Push-Pull
}

/*================================ ���������� EXTI =======================================*/

/**
 ***************************************************************************************
 *  @breif ���֧اڧ� EXTI.
 ***************************************************************************************
 */
void RVMSIS_EXTI0_init(void) {
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� �ѧݧ��֧�ߧѧ�ڧӧߧ�� ���ߧܧ�ڧ�
    MODIFY_REG(AFIO->EXTICR[0], AFIO_EXTICR1_EXTI0, AFIO_EXTICR1_EXTI0_PB); //AFIO_EXTICR1, EXTI0, �ӧ�ҧ�ѧ� ����� B.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOB); //���ܧݧ��ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ B
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_CNF0, 0b10 << GPIO_CFGLR_CNF0_Pos); //���ѧ����ڧ� �ߧ�اܧ� PB0 �� ��֧اڧ� Input with pull-up / pull-down
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_MODE0, 0b00 << GPIO_CFGLR_MODE0_Pos); //���ѧ����ۧܧ� �� ��֧اڧ� Input
    GPIOB->BSHR = GPIO_BSHR_BR0; //����է��اܧ� �� �٧֧ާݧ�
    SET_BIT(EXTI->INTENR, EXTI_INTENR_MR0); //���ܧݧ��ѧ֧� ���֧��ӧѧߧڧ� EXTI0 ��� �ӧ��էߧ�ާ� ��ڧԧߧѧݧ�
    SET_BIT(EXTI->RTENR, EXTI_RTENR_TR0); //���֧ѧԧڧ��ӧѧߧڧ� ��� ����ߧ�� �ӧܧ�.
    SET_BIT(EXTI->FTENR, EXTI_FTENR_TR0); //���֧ѧԧڧ��ӧѧߧڧ� ��� ���ѧէ� �ӧܧ�.
    //SET_BIT(EXTI->SWIEVR, EXTI_SWIEVR_SWIEVR0);//����� �����ӧѧ�ߧ�� �ӧܧݧ��֧ߧڧ� ���֧��ӧѧߧڧ�
    //SET_BIT(EXTI->INTFR, EXTI_INTF_INTF0); //����ާѧߧէ� �ӧ���է� �ڧ� ���֧��ӧѧߧڧ�
    NVIC_EnableIRQ(EXTI0_IRQn);

}

__WEAK void EXTI0_IRQHandler(void) {

    SET_BIT(EXTI->INTFR, EXTI_INTF_INTF0); //����ާѧߧէ� �ӧ���է� �ڧ� ���֧��ӧѧߧڧ�

}

/*================================ ���ѧۧާ֧�� �ߧ� ���ڧާ֧�� TIM3 =======================================*/

void RVMSIS_TIM3_init(void) {
    /*���ܧݧ��ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ��ѧۧާ֧�� (����ѧߧڧ�� 48)*/
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_TIM3); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� ��ѧۧާ֧�� 3
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� �ѧݧ��֧�ߧѧ�ڧӧߧ�� ���ߧܧ�ڧ�

    CLEAR_BIT(TIM3->CTLR1, TIM_UDIS); //���֧ߧ֧�ڧ��ӧѧ�� ���ҧ��ڧ� Update
    CLEAR_BIT(TIM3->CTLR1, TIM_URS); //���֧ߧ֧�ڧ��ӧѧ�� ���֧��ӧѧߧڧ�
    CLEAR_BIT(TIM3->CTLR1, TIM_OPM); //One pulse mode off(����֧��ڧ� �ߧ� ����ѧߧѧӧݧڧӧѧ֧��� ���� ��ҧߧ�ӧݧ֧ߧڧ�)
    CLEAR_BIT(TIM3->CTLR1, TIM_DIR); //����ڧ�ѧ֧� �ӧߧڧ�
    MODIFY_REG(TIM3->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos); //�����ѧӧߧڧӧѧߧڧ� ��� �ܧ�ѧ�
    SET_BIT(TIM3->CTLR1, TIM_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM3->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos); //����֧էէ֧ݧ֧ߧڧ� �ӧ�ܧݧ��֧ߧ�

    /*���ѧ����ۧܧ� ���֧��ӧѧߧڧ� (�����ѧߧڧ�� 409)*/
    SET_BIT(TIM3->DMAINTENR, TIM_UIE); //Update interrupt enable

    TIM3->PSC = 14400 - 1;
    TIM3->ATRLR = 10 - 1;

    NVIC_EnableIRQ(TIM3_IRQn); //���ѧ٧�֧�ڧ�� ���֧��ӧѧߧڧ� ��� ��ѧۧާ֧�� 3
    SET_BIT(TIM3->CTLR1, TIM_CEN); //���ѧ���� ��ѧۧާ֧��
}

__WEAK void TIM3_IRQHandler(void) {
    if (READ_BIT(TIM3->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM3->INTFR, TIM_UIF); //���ҧ���ڧ� ��ݧѧ� ���֧��ӧѧߧڧ�
    }
}

void RVMSIS_TIM3_PWM_CHANNEL1_init(void) {
    /*���ѧ����ۧܧ� �ߧ�اܧ� PA6 ���� ������*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ܧݧ��ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ ��
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF6, 0b10 << GPIO_CFGLR_CNF6_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE6, 0b11 << GPIO_CFGLR_MODE6_Pos);

    /*���ѧ����ۧܧ� ��ڧ�(���ѧߧѧ� 1)*/
    MODIFY_REG(TIM3->CHCTLR1, TIM_CC1S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC1FE); //Fast mode disable
    SET_BIT(TIM3->CHCTLR1, TIM_OC1PE); //Preload enable
    MODIFY_REG(TIM3->CHCTLR1, TIM_OC1M, 0b110 << TIM_OC1M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC1CE); //OC1Ref is not affected by the ETRF input

    /*���ѧ���� ������*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM3->CCER, TIM_CC1E);//On - OC1 signal is output on the corresponding output pin.
    SET_BIT(TIM3->CCER, TIM_CC1P); //OC1 active high.

    TIM3->CH1CVR = 5;
}

void RVMSIS_TIM3_PWM_CHANNEL2_init(void) {
    /*���ѧ����ۧܧ� �ߧ�اܧ� PA7 ���� ������*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ܧݧ��ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ ��
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF7, 0b10 << GPIO_CFGLR_CNF7_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE7, 0b11 << GPIO_CFGLR_MODE7_Pos);

    /*���ѧ����ۧܧ� ��ڧ�(���ѧߧѧ� 2)*/
    /*���ѧ����ۧܧ� ��ڧ�(���ѧߧѧ� 1)*/
    MODIFY_REG(TIM3->CHCTLR1, TIM_CC2S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC2FE); //Fast mode disable
    SET_BIT(TIM3->CHCTLR1, TIM_OC2PE); //Preload enable
    MODIFY_REG(TIM3->CHCTLR1, TIM_OC2M, 0b110 << TIM_OC2M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM3->CHCTLR1, TIM_OC2CE); //OC1Ref is not affected by the ETRF input

    /*���ѧ���� ������*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM3->CCER, TIM_CC2E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM3->CCER, TIM_CC2P); //OC1 active high.

    TIM3->CH2CVR = 5;
}

/*================================= ������������������ ADC ============================================*/

/**
 ***************************************************************************************
 *  @breif Analog-to-digital converter (ADC)
 ***************************************************************************************
 */

volatile uint16_t ADC_RAW_Data[2] = { 0, }; //���ѧ��ڧ�, �ܧ�է� �ҧ�է֧� �ܧڧէѧ�� �էѧߧߧ�� �� ������

void RVMSIS_ADC_DMA_init(void) {
    //Chapter 11 Direct Memory Access Control (DMA)
    SET_BIT(RCC->AHBPCENR, RCC_AHBPeriph_DMA1); //���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ� DMA1
    DMA1_Channel1->PADDR = (uint32_t) &(ADC1->RDATAR); //���ѧէѧ֧� �ѧէ�֧� ��֧�ڧ�֧�ڧۧߧ�ԧ� ������ۧ��ӧ�
    DMA1_Channel1->MADDR = (uint32_t) ADC_RAW_Data; //���ѧէѧ֧� �ѧէ�֧� �� ��ѧާ���, �ܧ�է� �ҧ�է֧� �ܧڧէѧ�� �էѧߧߧ��.
    DMA1_Channel1->CNTR = 2; //���ѧ����ڧ� �ܧ�ݧڧ�֧��ӧ� �էѧߧߧ�� �էݧ� ��֧�֧էѧ��. �����ݧ� �ܧѧاէ�ԧ� ��֧�ڧ�֧�ڧۧߧ�ԧ� ���ҧ��ڧ� ���� �٧ߧѧ�֧ߧڧ� �ҧ�է֧� ��ާ֧ߧ��ѧ����.
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PL, 0b00 << DMA_CFGR1_PL_Pos); //���ѧէѧէڧ� ���ڧ��ڧ�֧� �ܧѧߧѧݧ� �ߧ� �ӧ���ܧڧ�
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_DIR); //����֧ߧڧ� �ҧ�է֧� �����֧��ӧݧ��� �� ��֧�ڧ�֧�ڧ�
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_CIRC); //���ѧ����ڧ� DMA �� Circular mode
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PSIZE, 0b01 << DMA_CFGR1_PSIZE_Pos); //���ѧ٧ާ֧� �էѧߧߧ�� ��֧�ڧ�֧�ڧۧߧ�ԧ� ������ۧ��ӧ� 16 �ҧڧ�
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_MSIZE, 0b01 << DMA_CFGR1_MSIZE_Pos); //���ѧ٧ާ֧� �էѧߧߧ�� �� ��ѧާ��� 16 �ҧڧ�
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TCIE); //���ܧݧ��ڧ� ���֧��ӧѧߧڧ� ��� ���ݧߧ�� ��֧�֧էѧ��
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_HTIE); //����ܧݧ��ڧ� ���֧��ӧѧߧڧ� ��� ���ݧ�ӧڧߧߧ�� ��֧�֧էѧ��
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TEIE); //���ܧݧ��ڧ� ���֧��ӧѧߧڧ� ��� ���ڧҧܧ� ��֧�֧էѧ��.
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_MINC); //���ܧݧ��ڧ� �ڧߧܧ�֧ާ֧ߧ�ڧ��ӧѧߧڧ� ��ѧާ���
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_EN); //DMA ON
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_ADC1); //���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ADC1.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ ��.

    /*���ѧ����ۧܧ� �ߧ�ا֧� PA0 �� PA1 �ߧ� �ѧߧѧݧ�ԧ�ӧ�� �ӧ���*/
    /*Pin PA0 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF0, 0b00 << GPIO_CFGLR_CNF0_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE0, 0b00 << GPIO_CFGLR_MODE0_Pos);

    /*Pin PA1 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b00 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b00 << GPIO_CFGLR_MODE1_Pos);

    //����֧��ӧѧߧڧ� ��� ������: ��֧ԧ�ݧ��ߧ�� �ܧѧߧѧݧ� (�ӧܧ�/�ӧ�ܧ�)
    CLEAR_BIT(ADC1->CTLR1, ADC_EOCIE);//EOC interrupt enabled/disabled. An interrupt is generated when the EOC bit is set

    //����֧��ӧѧߧڧ� ��� ������: analog watchdog (�ӧܧ�/�ӧ�ܧ�)
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDIE);//Analog watchdog interrupt disabled

    //����֧��ӧѧߧڧ� ��� ������: �ڧߧا֧ܧ�ڧ��ӧѧߧߧ�� �ܧѧߧѧݧ� (�ӧܧ�/�ӧ�ܧ�)
    CLEAR_BIT(ADC1->CTLR1, ADC_JEOCIE);//JEOC interrupt disabled

    SET_BIT(ADC1->CTLR1, ADC_SCAN); //Scan mode enabled

    /* ����ڧާ֧�ѧߧڧ�:
     * ����֧��ӧѧߧڧ� EOC �ڧݧ� JEOC �ԧ֧ߧ֧�ڧ��֧��� ���ݧ�ܧ� �� �ܧ�ߧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ� ����ݧ֧էߧ֧ԧ� �ܧѧߧѧݧ�,
     * �֧�ݧ� ����ѧߧ�ӧݧ֧� �����ӧ֧���ӧ���ڧ� �ҧڧ� EOCIE �ڧݧ� JEOCIE.*/

    CLEAR_BIT(ADC1->CTLR1, ADC_AWDSGL); //Analog watchdog enabled on all channels
    CLEAR_BIT(ADC1->CTLR1, ADC_JAUTO); //Automatic injected group conversion disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_DISCEN); //Discontinuous mode on regular channels disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_JDISCEN); //Discontinuous mode on injected channels disabled
    MODIFY_REG(ADC1->CTLR1, ADC_DUALMOD, 0b0110 << ADC_DUALMOD_Pos); //0110: Regular simultaneous mode only
    CLEAR_BIT(ADC1->CTLR1, ADC_JAWDEN); //Analog watchdog disabled on injected channels
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDEN); //Analog watchdog disabled on regular channels

    //Control register 2 CTLR2
    SET_BIT(ADC1->CTLR2, ADC_ADON);//���ѧ����ڧ�� ������

    /* ����ڧާ֧�ѧߧڧ�:
     * ����ݧ� �� ����� �ا� �ާ�ާ֧ߧ� �ڧ٧ާ֧ߧ�֧��� �ܧѧܧ��-�ݧڧҧ� �է��ԧ�� �ҧڧ� �� ����� ��֧ԧڧ����,
     * �ܧ��ާ� ADON, ��� �ܧ�ߧӧ֧��ڧ� �ߧ� �٧ѧ���ܧѧ֧���.
     * ����� ��է֧ݧѧߧ� �էݧ� ���֧է��ӧ�ѧ�֧ߧڧ� ���ڧҧ��ߧ�ԧ� ���֧�ҧ�ѧ٧�ӧѧߧڧ�.*/

    SET_BIT(ADC1->CTLR2, ADC_CONT); //Continuous conversion mode(�ߧ֧��֧��ӧߧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ�)
    SET_BIT(ADC1->CTLR2, ADC_CAL); //Enable calibration
    /*����ڧާ֧�ѧߧڧ�:
     * ������ �ҧڧ� ����ѧߧѧӧݧڧӧѧ֧��� ����ԧ�ѧާާ�� �էݧ� �٧ѧ���ܧ� �ܧѧݧڧҧ��ӧܧ�.
     * ���� ��ҧ�ѧ��ӧѧ֧��� �ѧ��ѧ�ѧ�ߧ� ����ݧ� �٧ѧӧ֧��֧ߧڧ� �ܧѧݧڧҧ��ӧܧ�.*/

    while (READ_BIT(ADC1->CTLR2, ADC_CAL));
    //����է�اէ֧� ��ܧ�ߧ�ѧߧڧ� �ܧѧݧڧҧ��ӧܧ�
    //Delay_ms(10);

    SET_BIT(ADC1->CTLR2, ADC_DMA);//DMA �ӧܧݧ��֧�
    CLEAR_BIT(ADC1->CTLR2, ADC_ALIGN); //�����ѧӧߧڧӧѧߧڧ� ��� ���ѧӧ�ާ� �ܧ�ѧ�
    MODIFY_REG(ADC1->CTLR2, ADC_EXTSEL, 0b111 << ADC_EXTSEL_Pos); //���ѧ���ܧѧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ� ����ԧ�ѧާާߧ�
    CLEAR_BIT(ADC1->CTLR2, ADC_EXTTRIG); //Conversion on external event disabled
    //SET_BIT(ADC1->CTLR2, ADC_SWSTART); //���ѧ�ѧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ�
    //SET_BIT(ADC1->CTLR2, ADC_TSVREFE);//Temperature sensor and VREFINT channel enabled

    // 12.3.5 ADCx Sample Time Configuration Register 2 (ADCx_SAMPTR2) (x=1/2)
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP0, 0b111 << ADC_SMP0_Pos);//239.5 cycles
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP1, 0b111 << ADC_SMP1_Pos); //239.5 cycles
    //MODIFY_REG(ADC1->SAMPTR1, ADC_SMP17, 0b111 << ADC_SMP17_Pos); //239.5 cycles

    //12.3.9 ADCx Regular Channel Sequence Register1 (ADCx_RSQR1) (x=1/2)
    MODIFY_REG(ADC1->RSQR1, ADC_L, 0b0001 << ADC_L_Pos);//2 ���֧�ҧ�ѧ٧�ӧѧߧڧ�

    //12.3.11 ADCx Regular Channel Sequence Register 3 (ADCx_RSQR3) (x=1/2)
    MODIFY_REG(ADC1->RSQR3, ADC_SQ1, 0 << ADC_SQ1_Pos);
    MODIFY_REG(ADC1->RSQR3, ADC_SQ2, 1 << ADC_SQ2_Pos);
    //NVIC_EnableIRQ(ADC1_2_IRQn); //���ѧ٧�֧�ڧ�� ���֧��ӧѧߧڧ� ��� ������

    //SET_BIT(ADC1->CTLR2, ADC_SWSTART); //���ѧ�ѧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ�. ���� �ߧ�اߧ� �٧ѧ���ܧѧ��, �֧�ݧ� Continuous conversion mode(�ߧ֧��֧��ӧߧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ�) �ӧܧݧ��֧�

}

__WEAK void ADC1_2_IRQHandler(void) {
    if (READ_BIT(ADC1->STATR, ADC_EOC)) {
        ADC1->IDATAR1; //���ڧ�ѧ֧� �ܧѧߧѧ�, ����� ��ҧ���ڧ�� ��ݧѧ�
    }

}
__WEAK void DMA1_Channel1_IRQHandler(void) {
    if (READ_BIT(DMA1->INTFR, DMA_TCIF1)) {
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //���ҧ���ڧ� �ԧݧ�ҧѧݧ�ߧ�� ��ݧѧ�.
        /*���է֧�� �ާ�اߧ� ��ڧ�ѧ�� �ܧ��*/

    } else if (READ_BIT(DMA1->INTFR, DMA_TEIF1)) {
        /*���է֧�� �ާ�اߧ� ��է֧ݧѧ�� �ܧѧܧ��-��� ��ҧ�ѧҧ���ڧ� ���ڧҧ��*/
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //���ҧ���ڧ� �ԧݧ�ҧѧݧ�ߧ�� ��ݧѧ�.
    }
}

/*================================= ������������������ USART ============================================*/

/**
 ***************************************************************************************
 *  @breif Universal synchronous asynchronous receiver transmitter (USART)
 ***************************************************************************************
 */

struct USART_name husart1; //���ҧ��ӧݧ�֧� �����ܧ���� ��� USART.(���. ch32v203x_RVMSIS.h)
struct USART_name husart2; //���ҧ��ӧݧ�֧� �����ܧ���� ��� USART.(���. ch32v203x_RVMSIS.h)

/**
 ******************************************************************************
 *  @breif ���ѧ����ۧܧ� USART1. ���ѧ�ѧާ֧��� 9600 8 N 1
 ******************************************************************************
 */

void RVMSIS_USART1_Init(void) {

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ ��
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //���ܧݧ��֧ߧڧ� �ѧݧ��֧�ߧѧ�ڧӧߧ�� ���ߧܧ�ڧ�

    //���ݧ� �ܧ�ߧ�ڧԧ��ڧ��ӧѧߧڧ� �ߧ�ا֧� UART �էݧ� Full Duplex �ߧ�اߧ� �ڧ���ݧ�٧�ӧѧ�� Alternate function push-pull(����. ��.��. 9.1.11 GPIO configurations for device peripherals ����.111 Reference Manual)
    //Tx - Alternative Function output Push-pull(Maximum output speed 50 MHz)
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_CNF9, 0b10 << GPIO_CFGHR_CNF9_Pos);
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_MODE9, 0b11 << GPIO_CFGHR_MODE9_Pos);
    //Rx - Input floating
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_CNF10, 0b1 << GPIO_CFGHR_CNF10_Pos);
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_MODE10, 0b00 << GPIO_CFGHR_MODE10_Pos);

    //���ѧ����ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� USART1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_USART1);

    /*���ѧ��֧� Fractional baud rate generation
     �֧��� ����ާ�ݧ�:
     Tx/Rx baud = fCK/(16*USARTDIV)
     �ԧէ� fCK - Input clock to the peripheral (PCLK1 for USART2, 3, 4, 5 or PCLK2 for USART1)
     �� �ߧѧ�֧� ��ݧ��ѧ� fCK = 72000000
     �է�����ڧ� �ߧѧ� �ߧ�اߧ� ��ܧ������ 9600
     9600 = 72000000/(16*USARTDIV)
     ����ԧէ� USARTDIV = 72000000/9600*16 = 468.75
     DIV_Mantissa �� �էѧߧߧ�� ��ݧ��ѧ� �ҧ�է֧� 468, ���� �֧��� 0x1D4
     DIV_Fraction �ҧ�է֧�, �ܧѧ� 0.75*16 = 12, ���� �֧��� 0xC
     ����ԧէ� �ӧ֧�� ��֧ԧڧ��� USART->BRR �էݧ� ��ܧ������ 9600 �ҧ�է֧� �ӧ�ԧݧ�է֧��, �ܧѧ� 0x1D4C.
     �էݧ� ���ڧާ֧�� �֧�� ��ѧ٧ҧ֧�֧� ��ܧ������ 115200:
     115200 = 72000000/(16*USARTDIV)
     ����ԧէ� USARTDIV = 72000000/115200*16 = 39.0625
     DIV_Mantissa �� �էѧߧߧ�� ��ݧ��ѧ� �ҧ�է֧� 39, ���� �֧��� 0x27
     DIV_Fraction �ҧ�է֧�, �ܧѧ� 0.0625*16 = 1, ���� �֧��� 0x1
     ����ԧէ� �ӧ֧�� ��֧ԧڧ��� USART->BRR �էݧ� ��ܧ������ 115200 �ҧ�է֧� �ӧ�ԧݧ�է֧��, �ܧѧ� 0x271.
     */

    MODIFY_REG(USART1->BRR, USART_BRR_DIV_Mantissa, 0x1D4 << USART_BRR_DIV_Mantissa_Pos);
    MODIFY_REG(USART1->BRR, USART_BRR_DIV_Fraction, 0xC << USART_BRR_DIV_Mantissa_Pos);

    //18.10.4 USART Control Register1 (USARTx_CTLR1) (x=1/2/3/4/5/6/7/8)
    SET_BIT(USART1->CTLR1, USART_CTLR1_UE);//USART enable
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_M); //Word lenght 1 Start bit, 8 Data bits, n Stop bit
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_WAKE); //Wake up idle Line
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_PCE); //Partity control disabled
    //�ߧѧ����ۧܧ� ���֧��ӧѧߧڧ�
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_PEIE);//partity error interrupt disabled
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_TXEIE); //TXE interrupt is inhibited
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_TCIE); //Transmission complete interrupt disabled
    SET_BIT(USART1->CTLR1, USART_CTLR1_RXNEIE); //����֧��ӧѧߧڧ� ��� ���ڧ֧ާ� �էѧߧߧ�� �ӧܧݧ��֧ߧ�
    SET_BIT(USART1->CTLR1, USART_CTLR1_IDLEIE); //����֧��ӧѧߧڧ� ��� ��ݧѧԧ� IDLE �ӧܧݧ��֧ߧ�
    SET_BIT(USART1->CTLR1, USART_CTLR1_TE); //Transmitter is enabled
    SET_BIT(USART1->CTLR1, USART_CTLR1_RE); //Receiver is enabled and begins searching for a start bit
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_RWU);
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_SBK);

    //�����ѧݧ�ߧ�� �ߧѧ����ۧܧ�, �ߧ� �ܧѧ�ѧ������ ���ѧߧէѧ��ߧ�ԧ� USART, �ާ� ���ܧ� ����ԧѧ�� �ߧ� �ҧ�է֧�, �ߧ� �ߧ� �ӧ��ܧڧ� ��ݧ��ѧ� ��ҧߧ�ݧڧ�
    //18.10.5 USART Control Register2 (USARTx_CTLR2) (x=1/2/3/4/5/6/7/8)
    USART1->CTLR2 = 0;
    CLEAR_BIT(USART1->CTLR2, USART_CTLR2_STOP); //1 ����� �ҧڧ�.
    //18.10.6 USART Control Register 3 (USARTx_CTLR3) (x=1/2/3/4/5/6/7/8)
    USART1->CTLR3 = 0;
    //18.10.7 USART Guard Time and Prescaler Register (USARTx_GPR) (x=1/2/3/4/5/6/7/8)
    USART1->GPR = 0;

    NVIC_EnableIRQ(USART1_IRQn); //���ܧݧ��ڧ� ���֧��ӧѧߧڧ� ��� USART1
}

/**
 ******************************************************************************
 *  @breif ���ѧ����ۧܧ� USART2. ���ѧ�ѧާ֧��� 9600 8 N 1
 ******************************************************************************
 */

void CMSIS_USART2_Init(void) {

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ ��
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //���ܧݧ��֧ߧڧ� �ѧݧ��֧�ߧѧ�ڧӧߧ�� ���ߧܧ�ڧ�

    //���ݧ� �ܧ�ߧ�ڧԧ��ڧ��ӧѧߧڧ� �ߧ�ا֧� UART �էݧ� Full Duplex �ߧ�اߧ� �ڧ���ݧ�٧�ӧѧ�� Alternate function push-pull(����. ��.��. 9.1.11 GPIO configurations for device peripherals ����.111 Reference Manual)
    //Tx - Alternative Function output Push-pull(Maximum output speed 50 MHz)
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF2, 0b10 << GPIO_CFGLR_CNF2_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE2, 0b11 << GPIO_CFGLR_MODE2_Pos);
    //Rx - Input floating
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF3, 0b1 << GPIO_CFGLR_CNF3_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE3, 0b00 << GPIO_CFGLR_MODE3_Pos);

    //���ѧ����ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� USART2
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_USART2);

    /*���ѧ��֧� Fractional baud rate generation
     �֧��� ����ާ�ݧ�:
     Tx/Rx baud = fCK/(16*USARTDIV)
     �ԧէ� fCK - Input clock to the peripheral (PCLK1 for USART2, 3, 4, 5 or PCLK2 for USART1)
     �� �ߧѧ�֧� ��ݧ��ѧ� fCK = 36000000
     �է�����ڧ� �ߧѧ� �ߧ�اߧ� ��ܧ������ 9600
     9600 = 36000000/(16*USARTDIV)
     ����ԧէ� USARTDIV = 36000000/9600*16 = 234.375
     DIV_Mantissa �� �էѧߧߧ�� ��ݧ��ѧ� �ҧ�է֧� 234, ���� �֧��� 0xEA
     DIV_Fraction �ҧ�է֧�, �ܧѧ� 0.375*16 = 6, ���� �֧��� 0x6
     ����ԧէ� �ӧ֧�� ��֧ԧڧ��� USART->BRR �էݧ� ��ܧ������ 9600 �ҧ�է֧� �ӧ�ԧݧ�է֧��, �ܧѧ� 0xEA6.
     �էݧ� ���ڧާ֧�� �֧�� ��ѧ٧ҧ֧�֧� ��ܧ������ 115200: (���֧���ߧ���� ��� ��ܧ������ �ҧ�է֧� 0.15%. ���� ��֧ܧ�ާ֧ߧէ�֧���)
     115200 = 36000000/(16*USARTDIV)
     ����ԧէ� USARTDIV = 36000000/115200*16 = 19.53125
     DIV_Mantissa �� �էѧߧߧ�� ��ݧ��ѧ� �ҧ�է֧� 19, ���� �֧��� 0x13
     DIV_Fraction �ҧ�է֧�, �ܧѧ� 0.53125*16 = 8, ���� �֧��� 0x8
     ����ԧէ� �ӧ֧�� ��֧ԧڧ��� USART->BRR �էݧ� ��ܧ������ 115200 �ҧ�է֧� �ӧ�ԧݧ�է֧��, �ܧѧ� 0x138.
     */

    MODIFY_REG(USART2->BRR, USART_BRR_DIV_Mantissa, 0xEA << USART_BRR_DIV_Mantissa_Pos);
    MODIFY_REG(USART2->BRR, USART_BRR_DIV_Fraction, 0x6 << USART_BRR_DIV_Fraction_Pos);

    //18.10.4 USART Control Register1 (USARTx_CTLR1) (x=1/2/3/4/5/6/7/8)
    SET_BIT(USART2->CTLR1, USART_CTLR1_UE);//USART enable
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_M); //Word lenght 1 Start bit, 8 Data bits, n Stop bit
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_WAKE); //Wake up idle Line
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_PCE); //Partity control disabled
    //�ߧѧ����ۧܧ� ���֧��ӧѧߧڧ�
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_PEIE);//partity error interrupt disabled
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_TXEIE); //TXE interrupt is inhibited
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_TCIE); //Transmission complete interrupt disabled
    SET_BIT(USART2->CTLR1, USART_CTLR1_RXNEIE); //����֧��ӧѧߧڧ� ��� ���ڧ֧ާ� �էѧߧߧ�� �ӧܧݧ��֧ߧ�
    SET_BIT(USART2->CTLR1, USART_CTLR1_IDLEIE); //����֧��ӧѧߧڧ� ��� ��ݧѧԧ� IDLE �ӧܧݧ��֧ߧ�
    SET_BIT(USART2->CTLR1, USART_CTLR1_TE); //Transmitter is enabled
    SET_BIT(USART2->CTLR1, USART_CTLR1_RE); //Receiver is enabled and begins searching for a start bit
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_RWU);
    CLEAR_BIT(USART2->CTLR1, USART_CTLR1_SBK);

    //�����ѧݧ�ߧ�� �ߧѧ����ۧܧ�, �ߧ� �ܧѧ�ѧ������ ���ѧߧէѧ��ߧ�ԧ� USART, �ާ� ���ܧ� ����ԧѧ�� �ߧ� �ҧ�է֧�, �ߧ� �ߧ� �ӧ��ܧڧ� ��ݧ��ѧ� ��ҧߧ�ݧڧ�
    //18.10.5 USART Control Register2 (USARTx_CTLR2) (x=1/2/3/4/5/6/7/8)
    USART2->CTLR2 = 0;
    CLEAR_BIT(USART2->CTLR2, USART_CTLR2_STOP); //1 ����� �ҧڧ�.
    //18.10.6 USART Control Register 3 (USARTx_CTLR3) (x=1/2/3/4/5/6/7/8)
    USART2->CTLR3 = 0;
    //18.10.7 USART Guard Time and Prescaler Register (USARTx_GPR) (x=1/2/3/4/5/6/7/8)
    USART2->GPR = 0;

    NVIC_EnableIRQ(USART2_IRQn); //���ܧݧ��ڧ� ���֧��ӧѧߧڧ� ��� USART2
}

/**
 ******************************************************************************
 *  @breif ����֧��ӧѧߧڧ� ��� USART1
 ******************************************************************************
 */

__WEAK void USART1_IRQHandler(void) {
    if (READ_BIT(USART1->STATR, USART_STATR_RXNE)) {
        //����ݧ� ���ڧ�ݧ� �էѧߧߧ�� ��� USART
        husart1.rx_buffer[husart1.rx_counter] = USART1->DATAR; //����ڧ�ѧ֧� �էѧߧߧ�� �� �����ӧ֧���ӧ����� ���֧ۧܧ� �� rx_buffer
        husart1.rx_counter++; //���ӧ֧ݧڧ�ڧ� ���֧��ڧ� ���ڧߧ���� �ҧѧۧ� �ߧ� 1
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
 *  @breif ����֧��ӧѧߧڧ� ��� USART1
 ******************************************************************************
 */

__WEAK void USART2_IRQHandler(void) {
    if (READ_BIT(USART2->STATR, USART_STATR_RXNE)) {
        //����ݧ� ���ڧ�ݧ� �էѧߧߧ�� ��� USART
        husart2.rx_buffer[husart2.rx_counter] = USART2->DATAR; //����ڧ�ѧ֧� �էѧߧߧ�� �� �����ӧ֧���ӧ����� ���֧ۧܧ� �� rx_buffer
        husart2.rx_counter++; //���ӧ֧ݧڧ�ڧ� ���֧��ڧ� ���ڧߧ���� �ҧѧۧ� �ߧ� 1
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
 *  @breif ����ߧܧ�ڧ� �����ѧӧܧ� �էѧߧߧ�� ��� USART
 *  @param  *USART - USART, �� �ܧ�����ԧ� �ҧ�է�� �����ѧӧݧ����� �էѧߧߧ��
 *  @param  *data - �էѧߧߧ��, �ܧ������ �ҧ�է֧� �����ѧӧݧ���
 *  @param  Size - ��ܧ�ݧ�ܧ� �ҧѧۧ� ���֧ҧ�֧��� ��֧�֧էѧ��
 ******************************************************************************
 */

bool RVMSIS_USART_Transmit(USART_TypeDef* USART, uint8_t* data, uint16_t Size, uint32_t Timeout_ms) {
    for (uint16_t i = 0; i < Size; i++) {
        Timeout_counter_ms = Timeout_ms;
        //���է֧�, ���ܧ� �ݧڧߧڧ� �ߧ� ���ӧ�ҧ�էڧ���
        while (READ_BIT(USART->STATR, USART_STATR_TXE) == 0) {
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        USART->DATAR = *data++; //���ڧէѧ֧� �էѧߧߧ��
    }
    return true;
}

/*================================= ������������������ I2C ============================================*/

/**
 ***************************************************************************************
 *  @breif Inter-integrated circuit (I2C) interface
 ***************************************************************************************
 */

void RVMSIS_I2C_Reset(void) {
    //���ҧ��� �ߧѧ����֧� I2C
    //19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    SET_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST); //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST) == 0);
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST); //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST));
    /* ����ڧާ֧�ѧߧڧ�: ������ �ҧڧ� �ާ�اߧ� �ڧ���ݧ�٧�ӧѧ�� �էݧ� ���ӧ���ߧ�� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
     * ��֧�ڧ�֧�ڧۧߧ�ԧ� ������ۧ��ӧ� ����ݧ� ���ڧҧܧ� �ڧݧ� �٧ѧҧݧ�ܧڧ��ӧѧߧߧ�ԧ� �������ߧڧ�.
     * ���ѧ��ڧާ֧�, �֧�ݧ� �ҧڧ� BUSY ����ѧߧ�ӧݧ֧� �� ����ѧ֧��� �٧ѧҧݧ�ܧڧ��ӧѧߧߧ�� �ڧ�-�٧� ��ҧ�� �ߧ� ��ڧߧ�,
     * �ҧڧ� SWRST �ާ�اߧ� �ڧ���ݧ�٧�ӧѧ�� �էݧ� �ӧ���է� �ڧ� ����ԧ� �������ߧڧ�.*/
}

/**
 *************************************************************************************
 *  @breif ����ߧܧ�ڧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ� ��ڧߧ� I2C1. Sm.
 *************************************************************************************
 */

void RVMSIS_I2C1_Init(void) {
    //���ѧ����ۧܧ� ��ѧܧ�ڧ��ӧѧߧڧ�
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOB); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� ������ B
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� �ѧݧ��֧�ߧѧ�ڧӧߧ�� ���ߧܧ�ڧ�
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_I2C1); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� I2C1

    //���ѧ����ۧܧ� �ߧ�ا֧� SDA �� SCL
    //PB7 SDA (I2C Data I/O) Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_CNF7, 0b11 << GPIO_CFGLR_CNF7_Pos);//Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_MODE7, 0b11 << GPIO_CFGLR_MODE7_Pos); //Maximum output speed 50 MHz
    //PB6 SCL (I2C clock) Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_CNF6, 0b11 << GPIO_CFGLR_CNF6_Pos);//Alternate function open drain
    MODIFY_REG(GPIOB->CFGLR, GPIO_CFGLR_MODE6, 0b11 << GPIO_CFGLR_MODE6_Pos); //Maximum output speed 50 MHz

    //19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    RVMSIS_I2C_Reset();

    /*����� �ӧ�� �էݧ� �ڧߧڧ�� �ߧ� �ߧ�اߧ�. �����ݧ� ��ҧ���� �ڧ�ѧ� �ҧ�է֧� �� 0. */
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
    MODIFY_REG(I2C1->CTLR2, I2C_CTLR2_FREQ, 36 << I2C_CTLR2_FREQ_Pos); //f PCLK1 = 36 ���ԧ�

    //19.12.3 I2C Address Register 1 (I2Cx_OADDR1) (x=1/2)
    I2C1->OADDR1 = 0;
    //19.12.4 I2C Address Register2 (I2Cx_OADDR2) (x=1/2)
    I2C1->OADDR2 = 0;

    //19.12.8 I2C Clock Register (I2Cx_CKCFGR) (x=1/2)
    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS);//Standard mode I2C
    //SET_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS); //Fast mode I2C

    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_DUTY);//Fm mode tlow/thigh = 2
    //SET_BIT(I2C1->CCR, I2C_CCR_DUTY); //Fm mode tlow/thigh = 16/9 (see CCR)

    //���ѧ��֧� CCR. ���ާ���� ���ڧާ֧�� ��ѧ��֧��
    MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 180 << I2C_CKCFGR_CCR_Pos);//�էݧ� Sm mode
    //MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 30 << I2C_CKCFGR_CCR_Pos); //�էݧ� Fm mode. DUTY 0.
    //MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 4 << I2C_CKCFGR_CCR_Pos); //�էݧ� Fm mode. DUTY 1.

    //19.12.9 I2C Rise Time Register (I2Cx_RTR) (x=1/2)
    MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 37 << I2C_RTR_TRISE_Pos);//�էݧ� Sm mode
    //MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 12 << I2C_RTR_TRISE_Pos); //�էݧ� Fm mode

    SET_BIT(I2C1->CTLR1, I2C_CTLR1_PE); //I2C1 enable
}

/**
 *************************************************************************************
 *  @breif ����ߧܧ�ڧ� ��ܧѧߧڧ��ӧѧߧڧ� ������ۧ��ӧ� ��� �٧ѧէѧߧߧ�ާ� 7-�ҧڧ�ߧ�ާ� �ѧէ�֧��
 *  @param  *I2C - ��ڧߧ� I2C
 *  @param  Adress_Device - ���է�֧� ������ۧ��ӧ�
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� true - �֧�ݧ� ������ۧ��ӧ� ��� �٧ѧէѧߧߧ�ާ� �ѧէ�֧�� ����٧ӧѧݧ���,
 *           false - �֧�ݧ� ������ۧ��ӧ� ��� �٧ѧէѧߧߧ�ާ� �ѧէ�֧�� �ߧ� ���ӧ֧�ѧ֧�
 *************************************************************************************
 */
bool RVMSIS_I2C_Adress_Device_Scan(I2C_TypeDef* I2C, uint8_t Adress_Device, uint32_t Timeout_ms) {

    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //����ݧ� ��ڧߧ� �٧ѧߧ���

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //����ݧ� �ݧڧߧڧ� �ߧ� ��ѧާ�� �է֧ݧ� ��ӧ�ҧ�էߧ�, �� BUSY �ӧڧ�ڧ�
            RVMSIS_I2C_Reset(); //��֧�֧�
            RVMSIS_I2C1_Init(); //���ӧ���ߧѧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //����ݧ� ����ڧ� ���ѧ���, ���� �ާ� �� �ާѧ��֧��
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧڧ� ��ڧԧߧѧ� STOP
        }

        if (I2C->CTLR1 != 1) {
            //����ݧ� �� CR1 ����-��� �ݧڧ�ߧ֧�, ��� ��֧�֧٧ѧԧ��٧ڧ� I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //���ڧ� ACK ����ѧӧݧ�֧� (N)ACK ��֧ܧ��֧ԧ� �ҧѧۧ��, ���ڧߧڧާѧ֧ާ�ԧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ����.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //������ѧӧݧ�֧� ��ڧԧߧѧ� START

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //����������������!
    /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �էѧߧߧ�� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //���է�֧� + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧݧ�֧� ��ڧԧߧѧ� STOP
        /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
        I2C->STAR1;
        I2C->STAR2;
        return true;
    } else {
        //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧݧ�֧� ��ڧԧߧѧ� STOP
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif ����ߧܧ�ڧ� ��֧�֧էѧ�� �էѧߧߧ�� ��� I2C
 *  @param  *I2C - ��ڧߧ� I2C
 *  @param  Adress_Device - ���է�֧� ������ۧ��ӧ�
 *  @param  *data - ���ѧߧߧ��, �ܧ������ �ҧ�է֧� �����ѧӧݧ���
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� �ҧ�է֧� �����ѧӧݧ���.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� �����ѧӧܧ� �էѧߧߧ��. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Transmit(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //����ݧ� ��ڧߧ� �٧ѧߧ���

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //����ݧ� �ݧڧߧڧ� �ߧ� ��ѧާ�� �է֧ݧ� ��ӧ�ҧ�էߧ�, �� BUSY �ӧڧ�ڧ�
            RVMSIS_I2C_Reset(); //��֧�֧�
            RVMSIS_I2C1_Init(); //���ӧ���ߧѧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //����ݧ� ����ڧ� ���ѧ���, ���� �ާ� �� �ާѧ��֧��
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧڧ� ��ڧԧߧѧ� STOP
        }

        if (I2C->CTLR1 != 1) {
            //����ݧ� �� CR1 ����-��� �ݧڧ�ߧ֧�, ��� ��֧�֧٧ѧԧ��٧ڧ� I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //���ڧ� ACK ����ѧӧݧ�֧� (N)ACK ��֧ܧ��֧ԧ� �ҧѧۧ��, ���ڧߧڧާѧ֧ާ�ԧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ����.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //����ѧ���֧�.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //����������������!
    /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //���է�֧� + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���, ��ҧ���ڧ� �ҧڧ� ADDR
        /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*������ѧӧڧ� �էѧߧߧ��*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i); //���ѧ�ڧ�� �ҧѧۧ��
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //���է֧�, ���ܧ� �էѧߧߧ�� �٧ѧԧ��٧���� �� ��֧ԧڧ��� ��էӧڧԧ�.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//�����ѧߧѧӧݧڧӧѧ֧�
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�

        return true;

    } else {
        //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif ����ߧܧ�ڧ� ���ڧ֧ާ� �էѧߧߧ�� ��� I2C
 *  @param  *I2C - ��ڧߧ� I2C
 *  @param  Adress_Device - ���է�֧� ������ۧ��ӧ�
 *  @param  *data - ����է� �ҧ�է֧� �٧ѧ�ڧ��ӧѧ�� ���ڧߧ���� �էѧߧߧ��
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� �ҧ�է֧� ���ڧߧڧާѧ��.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� ���ڧ֧ާ� �էѧߧߧ��. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Receive(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //����ݧ� ��ڧߧ� �٧ѧߧ���

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //����ݧ� �ݧڧߧڧ� �ߧ� ��ѧާ�� �է֧ݧ� ��ӧ�ҧ�էߧ�, �� BUSY �ӧڧ�ڧ�
            RVMSIS_I2C_Reset(); //��֧�֧�
            RVMSIS_I2C1_Init(); //���ӧ���ߧѧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //����ݧ� ����ڧ� ���ѧ���, ���� �ާ� �� �ާѧ��֧��
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧڧ� ��ڧԧߧѧ� STOP
        }

        if (I2C->CTLR1 != 1) {
            //����ݧ� �� CR1 ����-��� �ݧڧ�ߧ֧�, ��� ��֧�֧٧ѧԧ��٧ڧ� I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //���ڧ� ACK ����ѧӧݧ�֧� (N)ACK ��֧ܧ��֧ԧ� �ҧѧۧ��, ���ڧߧڧާѧ֧ާ�ԧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ����.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //����ѧ���֧�.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //����������������!
    /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1 | 1); //���է�֧� + �ܧ�ާѧߧէ� Read

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���, ��ҧ���ڧ� �ҧڧ� ADDR
        /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*�������֧� �էѧߧߧ��*/
        for (uint16_t i = 0; i < Size_data; i++) {
            if (i < Size_data - 1) {
                SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //����ݧ� �ާ� ����ڧ� ���ڧߧ��� ��ݧ֧է���ڧ� �ҧѧۧ�, ��� �����ѧӧݧ�֧� ACK

                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    //���اڧէѧ֧�, ���ܧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ���� ����ӧ���� �էѧߧߧ��
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }

                *(data + i) = I2C->DATAR; //����֧ߧڧ� �ҧѧۧ��
            } else {
                CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //����ݧ� �ާ� �٧ߧѧ֧�, ���� ��ݧ֧է���ڧ� ���ڧߧ���� �ҧѧۧ� �ҧ�է֧� ����ݧ֧էߧڧ�, ��� �����ѧӧڧ� NACK

                SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    //���اڧէѧ֧�, ���ܧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ���� ����ӧ���� �էѧߧߧ��
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }
                *(data + i) = I2C->DATAR; //����֧ߧڧ� �ҧѧۧ��
            }
        }
        return true;

    } else {
        //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
        return false;
    }

}

/**
 **************************************************************************************************
 *  @breif ����ߧܧ�ڧ� �٧ѧ�ڧ�� �� ��ѧާ��� ��� ��ܧѧ٧ѧߧߧ�ާ� �ѧէ�֧��
 *  @param  *I2C - ��ڧߧ� I2C
 *  @param  Adress_Device - ���է�֧� ������ۧ��ӧ�
 *  @param  Adress_data - ���է�֧� �� ��ѧާ���, �ܧ�է� �ҧ�է֧� �٧ѧ�ڧ��ӧѧ�� �էѧߧߧ��
 *  @param  Size_adress - ���ѧ٧ާ֧� �ѧէ�֧�� �� �ҧѧۧ�ѧ�. ����ڧާ֧�: 1 - 8 �ҧڧ�ߧ�� �ѧէ�֧�. 2 - 16 �ҧڧ�ߧ�� �ѧէ�֧�.
 *  @param  *data - ���ѧߧߧ��, �ܧ������ �ҧ�է֧� �٧ѧ�ڧ��ӧѧ��
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� �ҧ�է֧� �٧ѧ�ڧ��ӧѧ��.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� �٧ѧ�ڧ��. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemWrite(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //����ݧ� ��ڧߧ� �٧ѧߧ���

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //����ݧ� �ݧڧߧڧ� �ߧ� ��ѧާ�� �է֧ݧ� ��ӧ�ҧ�էߧ�, �� BUSY �ӧڧ�ڧ�
            RVMSIS_I2C_Reset(); //��֧�֧�
            RVMSIS_I2C1_Init(); //���ӧ���ߧѧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //����ݧ� ����ڧ� ���ѧ���, ���� �ާ� �� �ާѧ��֧��
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧڧ� ��ڧԧߧѧ� STOP
        }

        if (I2C->CTLR1 != 1) {
            //����ݧ� �� CR1 ����-��� �ݧڧ�ߧ֧�, ��� ��֧�֧٧ѧԧ��٧ڧ� I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //���ڧ� ACK ����ѧӧݧ�֧� (N)ACK ��֧ܧ��֧ԧ� �ҧѧۧ��, ���ڧߧڧާѧ֧ާ�ԧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ����.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //����ѧ���֧�.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //����������������!
    /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //���է�֧� + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���, ��ҧ���ڧ� �ҧڧ� ADDR
        /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*������ѧӧڧ� �ѧէ�֧� ��ѧާ���*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*) &Adress_data + (Size_adress - 1 - i)); //���ѧ�ڧ�� �ҧѧۧ��
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //���է֧�, ���ܧ� �էѧߧߧ�� �٧ѧԧ��٧���� �� ��֧ԧڧ��� ��էӧڧԧ�.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//�����ѧߧѧӧݧڧӧѧ֧�
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
                    return false;
                }
            }
        }

        /*����է֧� �٧ѧ�ڧ��ӧѧ�� �էѧߧߧ�� �� ���֧ۧܧ� ��ѧާ���, �ߧѧ�ڧߧѧ� �� ��ܧѧ٧ѧߧߧ�ԧ� �ѧէ�֧��*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i); //���ѧ�ڧ�� �ҧѧۧ��
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //���է֧�, ���ܧ� �էѧߧߧ�� �٧ѧԧ��٧���� �� ��֧ԧڧ��� ��էӧڧԧ�.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//�����ѧߧѧӧݧڧӧѧ֧�
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�

        return true;

    } else {
        //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif ����ߧܧ�ڧ� ���֧ߧڧ� �ڧ� ��ѧާ��� ��� ��ܧѧ٧ѧߧߧ�ާ� �ѧէ�֧��
 *  @param  *I2C - ��ڧߧ� I2C
 *  @param  Adress_Device - ���է�֧� ������ۧ��ӧ�
 *  @param  Adress_data - ���է�֧� �� ��ѧާ���, ���ܧ�է� �ҧ�է֧� ���ڧ��ӧѧ�� �էѧߧߧ��
 *  @param  Size_adress - ���ѧ٧ާ֧� �ѧէ�֧�� �� �ҧѧۧ�ѧ�. ����ڧާ֧�: 1 - 8 �ҧڧ�ߧ�� �ѧէ�֧�. 2 - 16 �ҧڧ�ߧ�� �ѧէ�֧�.
 *  @param  *data - ���ѧߧߧ��, �� �ܧ������ �ҧ�է֧� �٧ѧ�ڧ��ӧѧ�� ���ڧ�ѧߧߧ�� �ڧߧ���ާѧ�ڧ�.
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� �ҧ�է֧� ���ڧ��ӧѧ��.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� ���ڧ��ӧѧߧڧ�. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemRead(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //����ݧ� ��ڧߧ� �٧ѧߧ���

        if ((READ_BIT(GPIOB->INDR, GPIO_INDR_IDR6)) && (READ_BIT(GPIOB->INDR, GPIO_INDR_IDR7))) {
            //����ݧ� �ݧڧߧڧ� �ߧ� ��ѧާ�� �է֧ݧ� ��ӧ�ҧ�էߧ�, �� BUSY �ӧڧ�ڧ�
            RVMSIS_I2C_Reset(); //��֧�֧�
            RVMSIS_I2C1_Init(); //���ӧ���ߧѧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //����ݧ� ����ڧ� ���ѧ���, ���� �ާ� �� �ާѧ��֧��
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧڧ� ��ڧԧߧѧ� STOP
        }

        if (I2C->CTLR1 != 1) {
            //����ݧ� �� CR1 ����-��� �ݧڧ�ߧ֧�, ��� ��֧�֧٧ѧԧ��٧ڧ� I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //���ڧ� ACK ����ѧӧݧ�֧� (N)ACK ��֧ܧ��֧ԧ� �ҧѧۧ��, ���ڧߧڧާѧ֧ާ�ԧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ����.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //����ѧ���֧�.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //����������������!
    /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //���է�֧� + �ܧ�ާѧߧէ� Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���, ��ҧ���ڧ� �ҧڧ� ADDR
        /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*������ѧӧڧ� �ѧէ�֧� ��ѧާ���*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*) &Adress_data + (Size_adress - 1 - i)); //���ѧ�ڧ�� �ҧѧۧ��
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //���է֧�, ���ܧ� �էѧߧߧ�� �٧ѧԧ��٧���� �� ��֧ԧڧ��� ��էӧڧԧ�.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//�����ѧߧѧӧݧڧӧѧ֧�
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
                    return false;
                }
            }
        }

        //����ӧ���ߧ�� ���ѧ��
        SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //����ѧ���֧�.

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
            //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

            if (!Timeout_counter_ms) {
                return false;
            }

        }
        //����������������!
        /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
        I2C->STAR1;
        I2C->DATAR = (Adress_Device << 1 | 1); //���է�֧� + �ܧ�ާѧߧէ� Read

        Timeout_counter_ms = Timeout_ms;
        while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
            //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

            if (!Timeout_counter_ms) {
                return false;
            }

        }

        if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
            //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���, ��ҧ���ڧ� �ҧڧ� ADDR
            /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
            I2C->STAR1;
            I2C->STAR2;

            /*�������֧� �էѧߧߧ��, �ߧѧ�ڧߧѧ� �� ��ܧѧ٧ѧߧߧ�ԧ� �ѧէ�֧��*/
            for (uint16_t i = 0; i < Size_data; i++) {
                if (i < Size_data - 1) {
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //����ݧ� �ާ� ����ڧ� ���ڧߧ��� ��ݧ֧է���ڧ� �ҧѧۧ�, ��� �����ѧӧݧ�֧� ACK
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    *(data + i) = I2C->DATAR; //����֧ߧڧ� �ҧѧۧ��
                } else {
                    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //����ݧ� �ާ� �٧ߧѧ֧�, ���� ��ݧ֧է���ڧ� ���ڧߧ���� �ҧѧۧ� �ҧ�է֧� ����ݧ֧էߧڧ�, ��� �����ѧӧڧ� NACK

                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    //����է�اէ֧�, ���ܧ� ��էӧڧԧ�ӧ�� ��֧ԧڧ��� �����ݧߧڧ��� �ߧ�ӧ�� �ҧѧۧ��� �էѧߧߧ��
                    *(data + i) = I2C->DATAR; //����֧ߧڧ� �ҧѧۧ��
                }
            }
            return true;

        } else {
            //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
            CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
            return false;
        }

    } else {
        //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
        return false;
    }
}

/*================================= ������������������ SPI ============================================*/

/**
***************************************************************************************
*  @breif Serial peripheral interface (SPI)
***************************************************************************************
*/

void RVMSIS_SPI1_init(void) {
    /*���ѧ����ۧܧ� GPIO*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //���ܧݧ��֧ߧڧ� �ѧݧ��֧�ߧѧ�ڧӧߧ�� ���ߧܧ�ڧ�
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_SPI1); //���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ� SPI1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ ��
    /*���ѧܧڧ� �ߧ�اܧ�:*/
    //PA4 - NSS
    //PA5 - SCK
    //PA6 - MISO
    //PA7 - MOSI
    //���� �ҧ�է֧� �ߧѧ���ѧڧӧѧ�� SPI �� ��֧اڧ� Master
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
     //���ѧ����ڧ� ��ѧާ� �ߧ�اܧ� ��ا� ����ݧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ� SPI, ����� ���� ���ѧ��� �ߧ� �ҧ�ݧ� �ݧڧ�ߧڧ� �ߧ�ԧ�է֧�ԧѧߧڧ�.

    //20.4.1 SPI Control Register 1 (SPIx_CTLR1) (x=1/2/3)
    MODIFY_REG(SPI1->CTLR1, SPI_CTLR1_BR, 0b011 << SPI_CTLR1_BR_Pos); //fPCLK/4. 72000000/32 = 2.22 MBits/s
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_CPOL); //����ݧ��ߧ����
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_CPHA); //���ѧ٧�
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_DFF); //0: 8-bit data frame format is selected for transmission/reception
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_LSBFIRST); //0: MSB transmitted first
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_SSM); //1: Software slave management enabled
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_SSI); //1: Software slave management enabled
    SET_BIT(SPI1->CTLR1, SPI_CTLR1_MSTR); //1: Master configuration
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_BIDIMODE); //0: 2-line unidirectional data mode selected
    CLEAR_BIT(SPI1->CTLR1, SPI_CTLR1_RXONLY); //0: Full duplex (Transmit and receive)

    SET_BIT(SPI1->CTLR1, SPI_CTLR1_SPE); //���ܧݧ��ڧ� SPI

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
    CLEAR_BIT(SPI1->I2SCFGR, SPI_I2SCFGR_I2SMOD); //��.��. �ߧ� F103C6T6 �ߧ֧� I2S, �֧ԧ� �ӧ��֧٧ѧݧ�, �� ��֧ԧڧ��� ����ѧӧڧݧ�, �ߧ�اߧ� ������� ��ҧߧ�ݧڧ�� �էѧߧߧ�� ��֧ԧڧ���. ���֧� ��ѧާ�� �ӧܧݧ��ڧ� ��֧اڧ� SPI mode.


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
 *  @breif ����ߧܧ�ڧ� ��֧�֧էѧ�� �էѧߧߧ�� ��� ��ڧߧ� SPI
 *  @param  *SPI - ��ڧߧ� SPI
 *  @param  *data - ���ѧߧߧ��, �ܧ������ �ҧ�է֧� ��֧�֧էѧӧѧ��.
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� �ҧ�է֧� ��֧�֧էѧӧѧ��.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� ��֧�֧էѧ��. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Transmit_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    //(���. Reference Manual ����. 712 Transmit-only procedure (BIDIMODE=0 RXONLY=0))
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //�����ӧ֧�ڧ� �٧ѧߧ������ ��ڧߧ�
        SPI->DATAR = *(data); //���ѧ�ڧ�֧� ��֧�ӧ�� ��ݧ֧ާ֧ߧ� �էѧߧߧ�� �էݧ� �����ѧӧܧ� �� ��֧ԧڧ��� SPI_DR
        //(����� ����� ���ڧ�ѧ֧��� �ҧڧ� TXE)

        for (uint16_t i = 1; i < Size_data; i++) {
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
                //���է֧�, ���ܧ� �ҧ��֧� �ߧ� ��֧�֧էѧ�� �ߧ� ���ӧ�ҧ�էڧ���
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            SPI->DATAR = *(data + i); //���ѧ�ڧ�֧� ��ݧ֧է���ڧ� ��ݧ֧ާ֧ߧ� �էѧߧߧ��.
        }
        Timeout_counter_ms = Timeout_ms;
        while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
            //�����ݧ� �٧ѧ�ڧ�� ����ݧ֧էߧ֧ԧ� ��ݧ֧ާ֧ߧ�� �էѧߧߧ�� �� ��֧ԧڧ��� SPI_DR,
            //���է�اէ֧�, ���ܧ� TXE ���ѧߧ֧� ��ѧӧߧ�� 1.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //���ѧ�֧� ���է�اէ֧�, ���ܧ� BSY ���ѧߧ֧� ��ѧӧߧ�� 0.
            //����� ��ܧѧ٧�ӧѧ֧� �ߧ� ���, ���� ��֧�֧էѧ�� ����ݧ֧էߧڧ� �էѧߧߧ�� �٧ѧӧ֧��֧ߧ�.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
    //����ڧާ֧�ѧߧڧ�:
    //�����ݧ� ��֧�֧էѧ�� �էӧ�� ��ݧ֧ާ֧ߧ��� �էѧߧߧ�� �� ��֧اڧާ� "transmit-only mode" �� ��֧ԧڧ���� SPI_SR ����ѧߧѧӧݧڧӧѧ֧��� ��ݧѧ� OVR, ��ѧ� �ܧѧ� ���ڧߧ���� �էѧߧߧ�� �ߧڧܧ�ԧէ� �ߧ� ���ڧ��ӧѧ����.
}

/**
 **************************************************************************************************
 *  @breif ����ߧܧ�ڧ� ��֧�֧էѧ�� �էѧߧߧ�� ��� ��ڧߧ� SPI
 *  @param  *SPI - ��ڧߧ� SPI
 *  @param  *data - ���ѧߧߧ��, �ܧ������ �ҧ�է֧� ��֧�֧էѧӧѧ��.
 *  @param  Size_data - ��ܧ�ݧ�ܧ� 16 - �ҧڧ�ߧ�� �էѧߧߧ�� ����ڧ� ��֧�֧էѧ��. ��.��. 1 = 2 �ҧѧۧ��.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� ��֧�֧էѧ��. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Transmit_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    //(���. Reference Manual ����. 712 Transmit-only procedure (BIDIMODE=0 RXONLY=0))
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //�����ӧ֧�ڧ� �٧ѧߧ������ ��ڧߧ�
        SPI->DATAR = *(data); //���ѧ�ڧ�֧� ��֧�ӧ�� ��ݧ֧ާ֧ߧ� �էѧߧߧ�� �էݧ� �����ѧӧܧ� �� ��֧ԧڧ��� SPI_DR
        //(����� ����� ���ڧ�ѧ֧��� �ҧڧ� TXE)

        for (uint16_t i = 1; i < Size_data; i++) {
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
                //���է֧�, ���ܧ� �ҧ��֧� �ߧ� ��֧�֧էѧ�� �ߧ� ���ӧ�ҧ�էڧ���
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            SPI->DATAR = *(data + i); //���ѧ�ڧ�֧� ��ݧ֧է���ڧ� ��ݧ֧ާ֧ߧ� �էѧߧߧ��.
        }
        Timeout_counter_ms = Timeout_ms;
        while (!READ_BIT(SPI->STATR, SPI_STATR_TXE)) {
            //�����ݧ� �٧ѧ�ڧ�� ����ݧ֧էߧ֧ԧ� ��ݧ֧ާ֧ߧ�� �էѧߧߧ�� �� ��֧ԧڧ��� SPI_DR,
            //���է�اէ֧�, ���ܧ� TXE ���ѧߧ֧� ��ѧӧߧ�� 1.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //���ѧ�֧� ���է�اէ֧�, ���ܧ� BSY ���ѧߧ֧� ��ѧӧߧ�� 0.
            //����� ��ܧѧ٧�ӧѧ֧� �ߧ� ���, ���� ��֧�֧էѧ�� ����ݧ֧էߧڧ� �էѧߧߧ�� �٧ѧӧ֧��֧ߧ�.
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }
    //����ڧާ֧�ѧߧڧ�:
    //�����ݧ� ��֧�֧էѧ�� �էӧ�� ��ݧ֧ާ֧ߧ��� �էѧߧߧ�� �� ��֧اڧާ� "transmit-only mode" �� ��֧ԧڧ���� SPI_SR ����ѧߧѧӧݧڧӧѧ֧��� ��ݧѧ� OVR, ��ѧ� �ܧѧ� ���ڧߧ���� �էѧߧߧ�� �ߧڧܧ�ԧէ� �ߧ� ���ڧ��ӧѧ����.
}

/**
 **************************************************************************************************
 *  @breif ����ߧܧ�ڧ� ���ڧ֧ާ� �էѧߧߧ�� ��� ��ڧߧ� SPI
 *  @param  *SPI - ��ڧߧ� SPI
 *  @param  *data - ���ѧߧߧ��, �ܧ�է� �ҧ�է֧� �٧ѧ�ڧ��ӧѧ�� ���ڧߧ���� �էѧߧߧ��.
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� ����ڧ� ���ڧߧ���.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� ���ڧ֧ާ�. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Receive_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //�����ӧ֧�ڧ� �٧ѧߧ������ ��ڧߧ�

        if (READ_BIT(SPI->STATR, SPI_STATR_OVR) || READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
            //��.��. �ާ� �ާ�ا֧� ���ڧߧڧާѧ�� �էѧߧߧ�� �� �ݧ�ҧ�� �ާ�ާ֧ߧ�, �ߧѧ��ڧާ֧� ����ݧ� ��֧اڧާ� "transmit-only mode"
            //��� ��ݧ֧է�֧� ����ӧ֧�ڧ�� ���ѧ���� OVR �� RXNE. ����ݧ� ����� �ҧ� ��էڧ� �ڧ� �ߧڧ� ����ѧߧ�ӧݧ֧�,
            //��� ��ҧ���ڧ� �ڧ� ���� ���ާ��� ���֧ߧڧ� ��֧ԧڧ���� DR.
            SPI->DATAR;
        }

        //���ѧ�ߧ֧� ���ڧ֧� �էѧߧߧ��
        for (uint16_t i = 0; i < Size_data; i++) {
            SPI->DATAR = 0; //���ѧ����ڧ� ��ѧܧ�ڧ��ӧѧߧڧ�, ����� ���ڧ�ѧ�� 8 �ҧڧ�
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
                //���է֧�, ���ܧ� �ҧ��֧� �ߧ� ���ڧ֧� �ߧ� �٧ѧ��ݧߧڧ���
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            *(data + i) = SPI->DATAR; //����ڧ��ӧѧ֧� �էѧߧߧ��
        }

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //���ѧ�֧� ���է�اէ֧�, ���ܧ� BSY ���ѧߧ֧� ��ѧӧߧ�� 0.
            //����� ��ܧѧ٧�ӧѧ֧� �ߧ� ���, ���� ���ڧ֧� ����ݧ֧էߧڧ� �էѧߧߧ�� �٧ѧӧ֧��֧�.
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
 *  @breif ����ߧܧ�ڧ� ���ڧ֧ާ� �էѧߧߧ�� ��� ��ڧߧ� SPI
 *  @param  *SPI - ��ڧߧ� SPI
 *  @param  *data - ���ѧߧߧ��, �ܧ�է� �ҧ�է֧� �٧ѧ�ڧ��ӧѧ�� ���ڧߧ���� �էѧߧߧ��.
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� 16 - �ҧڧ�ߧ�� �էѧߧߧ�� ����ڧ� ���ڧߧ���. ��.��. 1 = 2 �ҧѧۧ��.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� ���ڧ֧ާ�. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_SPI_Data_Receive_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    if (!READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
        //�����ӧ֧�ڧ� �٧ѧߧ������ ��ڧߧ�

        if (READ_BIT(SPI->STATR, SPI_STATR_OVR) || READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
            //��.��. �ާ� �ާ�ا֧� ���ڧߧڧާѧ�� �էѧߧߧ�� �� �ݧ�ҧ�� �ާ�ާ֧ߧ�, �ߧѧ��ڧާ֧� ����ݧ� ��֧اڧާ� "transmit-only mode"
            //��� ��ݧ֧է�֧� ����ӧ֧�ڧ�� ���ѧ���� OVR �� RXNE. ����ݧ� ����� �ҧ� ��էڧ� �ڧ� �ߧڧ� ����ѧߧ�ӧݧ֧�,
            //��� ��ҧ���ڧ� �ڧ� ���� ���ާ��� ���֧ߧڧ� ��֧ԧڧ���� DR.
            SPI->DATAR;
        }

        //���ѧ�ߧ֧� ���ڧ֧� �էѧߧߧ��
        for (uint16_t i = 0; i < Size_data; i++) {
            SPI->DATAR = 0; //���ѧ����ڧ� ��ѧܧ�ڧ��ӧѧߧڧ�, ����� ���ڧ�ѧ�� 16 �ҧڧ�
            Timeout_counter_ms = Timeout_ms;
            while (!READ_BIT(SPI->STATR, SPI_STATR_RXNE)) {
                //���է֧�, ���ܧ� �ҧ��֧� �ߧ� ���ڧ֧� �ߧ� �٧ѧ��ݧߧڧ���
                if (!Timeout_counter_ms) {
                    return false;
                }
            }
            *(data + i) = SPI->DATAR; //����ڧ��ӧѧ֧� �էѧߧߧ��
        }

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(SPI->STATR, SPI_STATR_BSY)) {
            //���ѧ�֧� ���է�اէ֧�, ���ܧ� BSY ���ѧߧ֧� ��ѧӧߧ�� 0.
            //����� ��ܧѧ٧�ӧѧ֧� �ߧ� ���, ���� ���ڧ֧� ����ݧ֧էߧڧ� �էѧߧߧ�� �٧ѧӧ֧��֧�.
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
