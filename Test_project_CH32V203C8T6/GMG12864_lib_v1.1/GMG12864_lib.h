/*
 * GMG12864_lib.h
 *
 *  Версия 1.2.(Общая графическая библиотека Волкова Олега)
 *  Переделан вывод символов на дисплей. В версии 1.2. имеется возможность вывода текста в любой точке по x и y.
 *
 *  Created on: Aug 28, 2023
 *      Author: Oleg Volkov
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md
 *  Группа ВК: https://vk.com/solderingiron.stm32
 *
 *  Кодировка UTF-8 Basic Latin: https://www.utf8-chartable.de/unicode-utf8-table.pl
 *  Кодировка UTF-8 Cyrillic: https://www.utf8-chartable.de/unicode-utf8-table.pl?start=1024&names=-&unicodeinhtml=hex
 *  Программа для конвертации изображения.bmp в bitmap: http://en.radzio.dxp.pl/bitmap_converter/
 */

#ifndef INC_GMG12864_LIB_H_
#define INC_GMG12864_LIB_H_

#include <main.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>



/*Внимание! Для совместимости с контроллерами STM32F4хх и другими, для выключения ножки, используем регистр BSRR, т.к. регистр BRR в них уже отсутствует*/
/*Собственно, ниже, так и сделано:                                                                                                           */
#define CS_SET()            GPIOA->BSHR = GPIO_BSHR_BR4;    //CS_притягиваем к земле
#define CS_RESET()          GPIOA->BSHR = GPIO_BSHR_BS4;    //CS притягиваем к 3.3v
#define RST_SET()           GPIOA->BSHR = GPIO_BSHR_BR3;    //RST притягиваем к земле
#define RST_RESET()         GPIOA->BSHR = GPIO_BSHR_BS3;    //RST притягиваем к 3.3v
#define DC_SET()            GPIOA->BSHR = GPIO_BSHR_BR2;    //DC притягиваем к земле
#define DC_RESET()          GPIOA->BSHR = GPIO_BSHR_BS2;    //DC притягиваем к 3.3v


#define ST7565_SetX(x) 		GMG12864_Send_command(((x) & 0xf0) >> 4 | 0x10); GMG12864_Send_command((x) & 0x0f)
#define ST7565_SetY(y) 		GMG12864_Send_command( ( (y) & 0x07 ) | 0xB0 )

#define FONT_3x5                0
#define FONT_5x7                1
#define INVERSION_OFF           0
#define INVERSION_ON            1
#define TIME_INTERVAL_SECOND    0
#define TIME_INTERVAL_MINUTE    1
#define TIME_INTERVAL_HOUR      2
#define GRID_OFF                0
#define GRID_ON                 1

/*================= Демонстрационное лого. Можно вырезать. =====================*/
void GMG12864_logo_demonstration(void);
/*================= Демонстрационное лого. Можно вырезать. =====================*/

void GMG12864_Init(void);
void GMG12864_Clean_Frame_buffer(void);
void GMG12864_Update(void);
void GMG12864_DrawBitmap(const uint8_t *bitmap, int8_t x, int8_t y, int8_t w, int8_t h);
void GMG12864_Draw_pixel(int16_t x, int16_t y, uint8_t color);
void GMG12864_Print_symbol_5x7(uint8_t x, uint8_t y, uint16_t symbol, uint8_t inversion);
void GMG12864_Print_symbol_3x5(uint8_t x, uint8_t y, uint16_t symbol, uint8_t inversion);
void GMG12864_Decode_UTF8(uint8_t x, uint8_t y, uint8_t font, bool inversion, unsigned char *tx_buffer);
void GMG12864_Inversion(uint16_t x_start, uint16_t x_end);
uint8_t GMG12864_Value_for_Plot(int y_min, int y_max, float value);
void GMG12864_Fill_the_array_Plot(uint8_t *counter, uint8_t *array, uint8_t size_array, bool *array_is_full, uint8_t value);
void GMG12864_Generate_a_Graph(uint8_t *counter, uint8_t *array, uint8_t size_array, bool * array_is_full, int y_min, int y_max, uint8_t x_grid_time, uint8_t time_interval,bool grid);
void GMG12864_Draw_line(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end, uint8_t color);
void GMG12864_Draw_rectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t color);
void GMG12864_Draw_rectangle_filled(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t color);
void GMG12864_Draw_circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color);
void GMG12864_Draw_circle_filled(int16_t x, int16_t y, int16_t radius, uint8_t color);
void GMG12864_Draw_triangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t color);
void GMG12864_Draw_triangle_filled(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t color);


#endif /* INC_GMG12864_LIB_H_ */
