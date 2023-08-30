/*
 * Описание проекта:
 * Датчик HDC1080:
 * I2C1 Sm. PB8 - SCL, PB9 - SDA (Remap включен)
 * SPI1     PA4 - NSS, PA5 - SCK, PA6 - MISO, PA7 - MOSI.
 * Скорость 36МГц. CPOL 1 CPHA 1
 * Так же для дисплея используются доп. ноги: PA2 - DC, PA3 - RST
 * TIM1 24 кГц. Управление подсветкой дисплея при помощи ШИМ сигнала с ноги PA8
 * (управление осуществляется через мосфет N типа)
 * TIM3 20 Гц. В прерывании происходит опрос кнопок управления.
 * Кнопки управления Button0 - A0, Button1 - A1, Button2 - B0, Button3 - B1
 * TIM4 0,1666666666666667 Гц. В прерывании заполняются графики.
 * Данная частота соответствует 10 итеррациям, как 1 минута.
 *
 * */

#include "main.h"
#include <stdio.h>
#include <stdbool.h>

//#define DEBUG_USE   //Использовать DEBUG по USART

/*HDC1080*/
bool Task1 = false;
uint32_t I2C1_Polling_Timer; //Таймер для опроса HDC1080
extern uint8_t I2C1_tx_buffer[4];  //Исходящий буфер
extern uint8_t I2C1_rx_buffer[10];  //Входящий буфер
extern float Temperature, Humidity;

/*Фильтрация данных*/
#define SMA_FILTER_ORDER                32
float SMA_Filter_Buffer_Temperature[SMA_FILTER_ORDER] = { 0, };
float SMA_Filter_Buffer_Humidity[SMA_FILTER_ORDER] = { 0, };
float Temperature_SMA;
float Humidity_SMA;

/*GMG12864*/
extern unsigned char tx_buffer[128]; //Буфер для отправки текста на дисплей
extern uint8_t cnt; //счетчик накопления значений в окне графика 1
extern const uint8_t size_array; //размер массива. В нашем случае ширина 100 точек(График 100*50 пикселей)
extern uint8_t arr[100]; //значения на графике 1. Заполняются в определенный момент времени(каждый шаг сдвига графика влево)
extern bool array_is_full; //График заполнился, теперь смещается справо налево
uint8_t y_min = 0;  //График 1
uint8_t y_max = 60; //График 1
uint8_t cnt1; //счетчик накопления значений в окне графика 2
uint8_t arr1[100]; //значения на графике 2. Заполняются в определенный момент времени(каждый шаг сдвига графика влево)
bool array_is_full_1 = false; //График заполнился, теперь смещается справо налево
uint8_t y_min1 = 0; //График 2
uint8_t y_max1 = 100; //График 2
int8_t Graph_number = 0; //Выбор страницы или графиков
bool Task2 = false;
uint32_t GMG12864_Timer; //Таймер для работы дисплея
int32_t Backlight = 50; //Процент подсветки дисплея
//Экспоненциальная зависимость подсветки дисплея от 0 до 100%
uint16_t Curves_exp[101] = { 0, 31, 33, 34, 36, 37, 39, 41, 43, 45, 47, 49, 52, 54, 57, 60, 62, 65, 68, 72, 75, 78, 82, 86, 90, 94, 99, 104, 108, 114, 119, 125, 131, 137, 143, 150, 157, 165, 172, 180,
        189, 198, 207, 217, 228, 238, 249, 261, 273, 286, 300, 314, 329, 345, 361, 378, 396, 414, 434, 454, 476, 498, 522, 546, 572, 599, 628, 657, 688, 721, 755, 790, 828, 867, 908, 951, 995, 1042,
        1092, 1143, 1197, 1253, 1312, 1374, 1439, 1507, 1578, 1653, 1731, 1812, 1898, 1987, 2081, 2179, 2282, 2390, 2502, 2620, 2744, 2874, 3000 };

/*--------------------------Для работы с Flash (Сохранение настроек)-----------------------*/
#define TIME_WAIT_BEFORE_WRITE_FLASH                1000 //1000 мс
uint32_t TIMER_wait_before_write_flash; //Ожидаем перед записью данных во флеш
bool write_flash = false; //Можно писать во флеш

typedef struct
    __attribute__((packed)) {
        int8_t Graph_number;
        int32_t Backlight;
    } Flash_struct;

    Flash_struct Flash_data;
    /*--------------------------Для работы с Flash (Сохранение настроек)-----------------------*/

    /*--------------------------------Float_transform--------------------------------------*/
    extern uint8_t sign_number;
    extern int integer_number;
    extern uint32_t fractional_number;
    /*--------------------------------Float_transform--------------------------------------*/

    /*---------------------------Прототипы функций-----------------------------------------*/
    void Task1_Pooling_HDC1080(void);
    void Task2_GMG12864_output(void);
    float SMA_FILTER_Get_Value(float *SMA_Filter_buffer, float *RAW_Data);
    void GPIO_Initialization(void);
    void Flash_init(void);
    /*---------------------------Прототипы функций-----------------------------------------*/

    /*Функции:*/

    void GPIO_Initialization(void) {
        /*Настройка ножек для CS, RST и DC(Для GMG12864)*/
        RVMSIS_GPIO_init(GPIOA, 2, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_50_MHZ); //DC
        RVMSIS_GPIO_init(GPIOA, 3, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_50_MHZ); //RST
        RVMSIS_GPIO_init(GPIOA, 4, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_50_MHZ); //CS
        RVMSIS_GPIO_init(GPIOA, 0, GPIO_INPUT, GPIO_INPUT_FLOATING, GPIO_SPEED_RESERVED); //Button0
        RVMSIS_GPIO_init(GPIOA, 1, GPIO_INPUT, GPIO_INPUT_FLOATING, GPIO_SPEED_RESERVED); //Button1
        RVMSIS_GPIO_init(GPIOB, 0, GPIO_INPUT, GPIO_INPUT_FLOATING, GPIO_SPEED_RESERVED); //Button2
        RVMSIS_GPIO_init(GPIOB, 1, GPIO_INPUT, GPIO_INPUT_FLOATING, GPIO_SPEED_RESERVED); //Button3
    }

    void Flash_init(void) {
        RVMSIS_FLASH_Read_data(0x0800F000, (uint8_t*) &Flash_data, sizeof(Flash_data));
        if (Flash_data.Graph_number > 2 || Flash_data.Graph_number < 0) {
            Graph_number = 0;
        } else {
            Graph_number = Flash_data.Graph_number;
        }
        if (Flash_data.Backlight > 100 || Flash_data.Backlight < 0) {
            Backlight = 50;
            TIM1->CH1CVR = Curves_exp[Backlight];
        } else {
            Backlight = Flash_data.Backlight;
            TIM1->CH1CVR = Curves_exp[Backlight];
        }
    }

    void Task1_Pooling_HDC1080(void) {
        //Команда опроса температуры и влажности
        I2C1_tx_buffer[0] = 0;
        I2C1_tx_buffer[1] = 0;
        RVMSIS_I2C_Data_Transmit(I2C1, HDC1080ADDR, I2C1_tx_buffer, 1, 100);
        Delay_ms(20);
        //Cчитаем данные
        RVMSIS_I2C_Data_Receive(I2C1, HDC1080ADDR, I2C1_rx_buffer, 4, 100);
        Temperature = (((float) ((uint16_t) I2C1_rx_buffer[0] << 8 | I2C1_rx_buffer[1]) * 165) / 65536) - 40;
        Temperature = Temperature - 1;
        Humidity = ((float) ((uint16_t) I2C1_rx_buffer[2] << 8 | I2C1_rx_buffer[3]) * 100) / 65536;
        Humidity = Humidity + 5;
        Temperature_SMA = SMA_FILTER_Get_Value(SMA_Filter_Buffer_Temperature, &Temperature);
        Humidity_SMA = SMA_FILTER_Get_Value(SMA_Filter_Buffer_Humidity, &Humidity);
    }

    void Task2_GMG12864_output(void) {

        GMG12864_Clean_Frame_buffer();

        /*Показ страницы с данными или графиком*/
        switch (Graph_number) {
        case 0:
            GMG12864_Clean_Frame_buffer();
            sprintf(tx_buffer, "HDC 1080");
            GMG12864_Decode_UTF8(40, 0, FONT_5x7, INVERSION_OFF, tx_buffer);

            Float_transform(Temperature_SMA, 1, (uint8_t*) &sign_number, (int*) &integer_number, (uint32_t*) &fractional_number);
            Float_print_to_buf(tx_buffer, "Температура: %3d.%.01ld°С", (uint8_t*) &sign_number, (int*) &integer_number, (uint32_t*) &fractional_number);
            GMG12864_Decode_UTF8(0, 20, FONT_5x7, INVERSION_OFF, tx_buffer);
            Float_transform(Humidity_SMA, 1, (uint8_t*) &sign_number, (int*) &integer_number, (uint32_t*) &fractional_number);
            Float_print_to_buf(tx_buffer, "Влажность:    %3d.%.01ld%%", (uint8_t*) &sign_number, (int*) &integer_number, (uint32_t*) &fractional_number);
            GMG12864_Decode_UTF8(0, 30, FONT_5x7, INVERSION_OFF, tx_buffer);
            sprintf(tx_buffer, "Яркость дисплея: %3d%%", Backlight);
            GMG12864_Decode_UTF8(0, 50, FONT_5x7, INVERSION_OFF, tx_buffer);
            break;
        case 1:
            GMG12864_Generate_a_Graph(&cnt, arr, size_array, &array_is_full, y_min, y_max, 1, TIME_INTERVAL_MINUTE, GRID_ON);
            Float_transform(Temperature_SMA, 1, (uint8_t*) &sign_number, (int*) &integer_number, (uint32_t*) &fractional_number);
            Float_print_to_buf(tx_buffer, "Температура: %d.%.01ld°С", (uint8_t*) &sign_number, (int*) &integer_number, (uint32_t*) &fractional_number);
            GMG12864_Decode_UTF8(0, 58, FONT_3x5, INVERSION_OFF, tx_buffer);
            break;
        case 2:
            GMG12864_Generate_a_Graph(&cnt1, arr1, size_array, &array_is_full_1, y_min1, y_max1, 1, TIME_INTERVAL_MINUTE, GRID_ON);
            Float_transform(Humidity_SMA, 1, (uint8_t*) &sign_number, (int*) &integer_number, (uint32_t*) &fractional_number);
            Float_print_to_buf(tx_buffer, "Влажность: %d.%.01ld%%", (uint8_t*) &sign_number, (int*) &integer_number, (uint32_t*) &fractional_number);
            GMG12864_Decode_UTF8(0, 58, FONT_3x5, INVERSION_OFF, tx_buffer);
            break;
        }
        GMG12864_Update();
    }

    /*-------------------Фильтрация сигнала с датчика HDC1080------------------------*/
    float SMA_FILTER_Get_Value(float *SMA_Filter_buffer, float *RAW_Data) {
        /* Создадим переменную для суммы сырых значений */
        float SMA_Filter_Result = 0;
        /* Начнем заполнять массив сырыми значениями с конца */
        SMA_Filter_buffer[SMA_FILTER_ORDER - 1] = *RAW_Data;
        /* Просуммируем все элементы массива */
        for (uint8_t i = 0; i < SMA_FILTER_ORDER; i++) {
            SMA_Filter_Result += SMA_Filter_buffer[i];
        }

        /*Найдем среднее арифметическое значение:*/
        SMA_Filter_Result = (float) SMA_Filter_Result / (float) SMA_FILTER_ORDER;
        /* Сдвинем все элементы массива влево на 1 */
        for (uint8_t i = 0; i < SMA_FILTER_ORDER; i++) {
            SMA_Filter_buffer[i] = SMA_Filter_buffer[i + 1];
        }
        return (float) SMA_Filter_Result; //Вернем среднее арифметическое значение
    }
    /*-------------------Фильтрация сигнала с датчика HDC1080------------------------*/


    /*---------------------------Работа с кнопками-----------------------------------*/
    void Button0_Callback(void) {
        Backlight--;
        if (Backlight <= 0) {
            Backlight = 0;
        }
        Flash_data.Backlight = Backlight;
        TIM1->CH1CVR = Curves_exp[Backlight];
        write_flash = true;
        TIMER_wait_before_write_flash = TIME_WAIT_BEFORE_WRITE_FLASH; //Зададим таймаут, по истечении которого настройки сохранятся во Flash

    }
    void Button1_Callback(void) {
        Backlight++;
        if (Backlight >= 100) {
            Backlight = 100;
        }
        Flash_data.Backlight = Backlight;
        TIM1->CH1CVR = Curves_exp[Backlight];
        write_flash = true;
        TIMER_wait_before_write_flash = TIME_WAIT_BEFORE_WRITE_FLASH; //Зададим таймаут, по истечении которого настройки сохранятся во Flash
    }
    void Button2_Callback(void) {
        Graph_number--;
        if (Graph_number <= 0) {
            Graph_number = 0;
        }
        Flash_data.Graph_number = Graph_number;
        write_flash = true;
        TIMER_wait_before_write_flash = TIME_WAIT_BEFORE_WRITE_FLASH; //Зададим таймаут, по истечении которого настройки сохранятся во Flash
    }
    void Button3_Callback(void) {
        Graph_number++;
        if (Graph_number >= 2) {
            Graph_number = 2;
        }
        Flash_data.Graph_number = Graph_number;
        write_flash = true;
        TIMER_wait_before_write_flash = TIME_WAIT_BEFORE_WRITE_FLASH; //Зададим таймаут, по истечении которого настройки сохранятся во Flash
    }
    /*---------------------------Работа с кнопками-----------------------------------*/

    /*--------------------------Таймера для заполнения графика-----------------------*/
    void TIM4_IRQHandler(void) {
        if (READ_BIT(TIM4->INTFR, TIM_UIF)) {
            CLEAR_BIT(TIM4->INTFR, TIM_UIF); //Сбросим флаг прерывания
        }
        /*--------------------------Заполнение графиков--------------------------*/
        uint8_t value = GMG12864_Value_for_Plot(y_min, y_max, Temperature_SMA);
        GMG12864_Fill_the_array_Plot(&cnt, arr, size_array, &array_is_full, value);

        uint8_t value1 = GMG12864_Value_for_Plot(y_min1, y_max1, Humidity_SMA);
        GMG12864_Fill_the_array_Plot(&cnt1, arr1, size_array, &array_is_full_1, value1);
        /*--------------------------Заполнение графиков--------------------------*/
    }
    /*--------------------------Таймера для заполнения графика-----------------------*/

    int main(void) {
        RVMSIS_Debug_init(); //Настройка дебага
        RVMSIS_RCC_SystemClock_144MHz(); //Настройка системной частоты
        RVMSIS_SysTick_Timer_init(); //Настройка системного таймера
        RVMSIS_I2C1_Init(); //Подключим датчик HDC1080(Включен ремап на I2C/PB8, PB9)
        HDC1080_init(); //Инициализация HDC1080
        GPIO_Initialization(); //Настройка GPIO
        ButtonsInit(); //Инициализация структуры кнопок
        RVMSIS_TIM3_init(); //Таймер для кнопок
        RVMSIS_TIM1_init(); //Таймер для подсветки дисплея
        RVMSIS_TIM1_PWM_CHANNEL1_init(); //Настройка шим (для подсветки дисплея)
        RVMSIS_SPI1_init(); //настройка SPI1 (для работы с дисплеем)
        Flash_init(); //Инициализация настроек устройства
        GMG12864_Init(); //Инициализируем дисплей
        RVMSIS_TIM4_init(); //Таймер для заполнения графиков
        GMG12864_logo_demonstration(); //Покажем лого при старте

#ifdef DEBUG_USE
        RVMSIS_USART3_Init(); //См. файл syscalls.c USART3 115200 8N1 выбран для отладки через printf. Ножка PB10
        printf("Start USART Debug...\r\n");
#endif

        while(1) {
            /*Опрос HDC1080 по I2C1*/
            if (Task1 && !I2C1_Polling_Timer) {
                Task1_Pooling_HDC1080();
                Software_timer((uint32_t*) &I2C1_Polling_Timer, 50);
                Task1 = false;
            }

            /*Работа с GMG12864 по SPI1*/
            if (Task2 && !GMG12864_Timer) {
                Task2_GMG12864_output();
                Software_timer((uint32_t*)&GMG12864_Timer, 100); //Вывод на дисплей информации каждые 100 мс
                Task2 = false;
            }

            /*Сохранение настроек во Flash при достижении таймаута, если была команда сохраниться*/
            if (TIMER_wait_before_write_flash == 0 && write_flash) {
                write_flash = false;
                RVMSIS_FLASH_Page_write(0x0800F000, (uint8_t*) &Flash_data, sizeof(Flash_data));
            }

        }
    }
