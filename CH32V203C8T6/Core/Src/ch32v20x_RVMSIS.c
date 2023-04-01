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
 *  ���ߧ֧�ߧڧ� �ܧӧѧ��֧ӧ�� ��֧٧�ߧѧ��� �ߧ� 8 MHz
 *  ADC �ߧѧ����֧� �ߧ� 12MHz
 *  USB �ߧѧ����֧� �ߧ� 48MHz
 *  MCO ���էܧݧ��֧� �� HSE �� ��ѧܧ�ڧ��֧��� ��� 8MHz
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
    MODIFY_REG(RCC->CFGR0, RCC_HPRE, RCC_HPRE_DIV1); //AHB prescaler /1
    //Note: FLASH access clock frequency cannot
    //be more than 60 MHz.
    CLEAR_BIT(FLASH->CTLR, 1 << 25U); //0: FLASH access clock frequency = SYSCLK/2.
    MODIFY_REG(RCC->CFGR0, RCC_PPRE1, RCC_PPRE1_DIV1); //APB1 Prescaler /1,  ����է֧� 144MHz.
    MODIFY_REG(RCC->CFGR0, RCC_PPRE2, RCC_PPRE2_DIV2); //APB2 Prescaler /2. ����է֧� 72MHz.
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
    SysTick->CMP = 17999; ////���ѧ����ڧ� ���֧��ӧѧߧڧ� �ߧ� ��ѧ����� �� 1 �ܧ���(��.��. ���ѧҧ��ܧ� �ҧ�է֧� �ܧѧاէ�� �ާ�) 18000000 / 18000 = 1000����
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

    SET_BIT(ADC1->CTLR2, ADC_DMA); //DMA �ӧܧݧ��֧�
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

    }
    else if (READ_BIT(DMA1->INTFR, DMA_TEIF1)) {
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

