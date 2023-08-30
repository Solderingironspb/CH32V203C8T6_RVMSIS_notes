/*
 * SoftwareTimer.h
 *
 *  Created on: 14 авг. 2023 г.
 *      Author: Solderingiron
 *
 *  Допустим нужно моргнуть светодиодом по какому-то событию.
 *  Дилей использовать нельзя. Тогда мы заведем таймер для светодиода.
 *  Запускаем светодиод, далее воспользуемся функцией Software_timer.
 *  Укажем переменную для таймера светодиода и время,
 *  на сколько запустить таймер. Допустим 100.
 *  В аппаратном таймере или SysTick проверяем наш таймер при
 *  помощи функции Software_timer_check. Если таймер вернет false -
 *  можно погасить светодиод.
 *  Тем самым при наступлении события мы сделали видимую индикацию в 100 мс.
 *  Диод - это пример. Можно так и функции запускать.
 */

/*
 * Пример.
 * Где-то в функции:
 *  GPIOC->BSRR = GPIO_BSRR_BS13;
 *  Software_timer(&LED1_timer, 50);
 *
 *  */

/*
 * В то же время в SysTick:
 *
 * if (!Software_timer_check(&LED1_timer)){
 *      GPIOC->BSRR = GPIO_BSRR_BR13;
 *  }
 *
 * */
#include <stdint.h>
#include <stdbool.h>
void Software_timer(uint32_t *Timer, uint32_t Time);
bool Software_timer_check(uint32_t *Timer);
