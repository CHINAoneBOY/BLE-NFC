#ifndef _LOCK_GPIO_H_
#define _LOCK_GPIO_H_
#include <stdint.h>
#include "nrf_delay.h"

#define LED1 		4            //
#define LED2 		0
#define MOTO_NSLEEP		14
#define MOTO_NFAULT		15
#define MOTO_EN1      8
#define MOTO_EN2      6
#define QK_BEEP          30
#define INPUT_SR      12

#define IRDA_PUT      7
#define IRDA_IN       9

#define VOL_EN        1

void  beep_gpio_init(void);
void  moto_gpio_init(void);
void  irda_gpio_init(void);
void  led_gpio_init(void);
void qk_lock_init(void);
void beep_test(void);
#endif