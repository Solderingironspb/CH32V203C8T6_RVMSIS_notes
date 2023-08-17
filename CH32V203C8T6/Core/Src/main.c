#include "main.h"
#include "ch32v20x_RVMSIS.h"

uint8_t Priority = 0; //Попробуем узнать приоритет прерывания

typedef struct
    __attribute__((packed)) {
        uint8_t Data1;
        uint16_t Data2;
        uint32_t Data3;
        float Data4;
    } Flash_struct;
    Flash_struct Flash_data_CH32;
    Flash_struct Flash_data_CH32_read;

    int main(void) {
        RVMSIS_Debug_init(); //Настройка дебага
        RVMSIS_RCC_SystemClock_144MHz(); //Настройка системной частоты
        RVMSIS_SysTick_Timer_init(); //Настройка системного таймера
        RVMSIS_GPIO_init(GPIOC, 13, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_50_MHZ); //Настройка PC13 в режим Push-pull
        Flash_data_CH32.Data1 = 255;
        Flash_data_CH32.Data2 = 0x4567;
        Flash_data_CH32.Data3 = 0x89101112;
        Flash_data_CH32.Data4 = 3.14159f;
        RVMSIS_FLASH_Page_write(0x0800F000, (uint8_t*) &Flash_data_CH32, sizeof(Flash_data_CH32));
        RVMSIS_FLASH_Read_data(0x0800F000, (uint8_t*) &Flash_data_CH32_read, sizeof(Flash_data_CH32_read));
        Priority = NVIC_GetPriority(SysTicK_IRQn);

        while(1) {
            GPIOC->BSHR = GPIO_BSHR_BS13;
            Delay_ms(100);
            GPIOC->BSHR = GPIO_BSHR_BR13;
            Delay_ms(100);
        }
    }
