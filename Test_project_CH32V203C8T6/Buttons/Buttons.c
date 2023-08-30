/*
 * Buttons.c
 *
 *  Created on: 25 авг. 2023 г.
 *      Author: Solderingiron
 */

/*
 * Очень простая библиотека для работы с кнопками, без триггера Шмитта
 * Пока реализация только с подтяжкой к питанию, т.к. в основном так делаю.
 * Подтяжка 10кОм, на кнопке конденсатор 1 мкФ.
 * Таймером опрашиваю состояния кнопок с частотой 50 Гц.
 *
 */
#include "Buttons.h"

Buttons Button0;
Buttons Button1;
Buttons Button2;
Buttons Button3;

void ButtonsInit(void){
	Button0.StateOld = false;
	Button0.StateNew = false;
	Button0.TimeHold = 0;

	Button1.StateOld = false;
	Button1.StateNew = false;
	Button1.TimeHold = 0;

	Button2.StateOld = false;
	Button2.StateNew = false;
	Button2.TimeHold = 0;

	Button3.StateOld = false;
	Button3.StateNew = false;
	Button3.TimeHold = 0;
}

void ButtonAction(uint8_t *ButtonOldState, uint8_t *ButtonNewState, uint8_t *ButtonTimeHold, bool ButtonState, void * ButtonCallback()) {
	*ButtonNewState = ButtonState;
	if (!(*ButtonNewState) && (*ButtonOldState)) {
		 ButtonCallback();
		*ButtonTimeHold = 0;
	} else {
		if (!(*ButtonNewState)) {
			*ButtonTimeHold = *ButtonTimeHold + 1;
			if ((*ButtonTimeHold) >= TIME_BUTTONS_HOLD) {
				*ButtonTimeHold = TIME_BUTTONS_HOLD;
				 ButtonCallback();
			}
		}
	}
	*ButtonOldState = *ButtonNewState;
}

__WEAK void Button0_Callback(void) {

}
__WEAK void Button1_Callback(void) {

}
__WEAK void Button2_Callback(void) {

}
__WEAK void Button3_Callback(void) {

}

