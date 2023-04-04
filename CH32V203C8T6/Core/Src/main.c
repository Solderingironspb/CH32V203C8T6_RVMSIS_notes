#include "main.h"
#include "ch32v20x_RVMSIS.h"


typedef struct __attribute__((packed)) {
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
    RVMSIS_PC13_OUTPUT_Push_Pull_init(); //Настройка PC13 в режим Push-pull
    Flash_data_CH32.Data1 = 255;
    Flash_data_CH32.Data2 = 0x4567;
    Flash_data_CH32.Data3 = 0x89101112;
    Flash_data_CH32.Data4 = 3.14159f;
    RVMSIS_FLASH_Page_write(0x0800F000, (uint8_t*)&Flash_data_CH32, sizeof(Flash_data_CH32));
    RVMSIS_FLASH_Read_data(0x0800F000, (uint8_t*)&Flash_data_CH32_read, sizeof(Flash_data_CH32_read));


    while(1) {
        GPIOC->BSHR = GPIO_BSHR_BS13;
        Delay_ms(1000);
        GPIOC->BSHR = GPIO_BSHR_BR13;
        Delay_ms(1000);

    }
}
