/*
 * Buttons.h
 *
 *  Created on: 25 авг. 2023 г.
 *      Author: Solderingiron
 */


/*Пример работы:*/
//int32_t Backlight;
//int32_t Volume;
//
//extern Buttons Button0;
//extern Buttons Button1;
//extern Buttons Button2;
//extern Buttons Button3;
//
//void TIM3_IRQHandler(void) {
//	if (READ_BIT(TIM3->SR, TIM_SR_UIF)) {
//		CLEAR_BIT(TIM3->SR, TIM_SR_UIF); //Сбросим флаг прерывания
//	}
//
//	ButtonAction((uint8_t*) &Button0.StateOld, (uint8_t*) &Button0.StateNew, (uint8_t*) &Button0.TimeHold, BUTTON0_STATE, (void*) Button0_Callback);
//	ButtonAction((uint8_t*) &Button1.StateOld, (uint8_t*) &Button1.StateNew, (uint8_t*) &Button1.TimeHold, BUTTON1_STATE, (void*) Button1_Callback);
//	ButtonAction((uint8_t*) &Button2.StateOld, (uint8_t*) &Button2.StateNew, (uint8_t*) &Button2.TimeHold, BUTTON2_STATE, (void*) Button2_Callback);
//	ButtonAction((uint8_t*) &Button3.StateOld, (uint8_t*) &Button3.StateNew, (uint8_t*) &Button3.TimeHold, BUTTON3_STATE, (void*) Button3_Callback);
//
//}
//
//void Button0_Callback(void) {
//	Backlight--;
//	if (Backlight <= 0) {
//		Backlight = 0;
//	}
//
//}
//void Button1_Callback(void) {
//	Backlight++;
//	if (Backlight >= 100) {
//		Backlight = 100;
//	}
//}
//
//void Button2_Callback(void) {
//	Volume--;
//	if (Volume <= 0) {
//		Volume = 0;
//	}
//
//}
//void Button3_Callback(void) {
//	Volume++;
//	if (Volume >= 100) {
//		Volume = 100;
//	}
//}

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

#include "main.h"
#include <stdbool.h>

#define TIME_BUTTONS_HOLD					10

#define BUTTON0_STATE						READ_BIT(GPIOA->INDR, GPIO_INDR_IDR0)
#define BUTTON1_STATE						READ_BIT(GPIOA->INDR, GPIO_INDR_IDR1)
#define BUTTON2_STATE						READ_BIT(GPIOB->INDR, GPIO_INDR_IDR0)
#define BUTTON3_STATE						READ_BIT(GPIOB->INDR, GPIO_INDR_IDR1)



typedef struct {
	bool StateOld;
	bool StateNew;
	uint8_t TimeHold;
} Buttons;

void ButtonsInit(void);
void ButtonAction(uint8_t *ButtonOldState, uint8_t *ButtonNewState, uint8_t *ButtonTimeHold, bool ButtonState, void * ButtonCallback());
void Button0_Callback(void);
void Button1_Callback(void);
void Button2_Callback(void);
void Button3_Callback(void);

#endif /* INC_BUTTONS_H_ */
