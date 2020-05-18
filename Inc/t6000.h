/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __T6000_H
#define __T6000_H

#include "stm32f4xx_hal.h"

// Размер буфера данных для одного канала, в светодиодах
#define DATA_BUFFER_LED_CAPACITY 1024
// Размер буфера данных для одного канала, в байтах (3 байта на 1 светодиод)
#define DATA_BUFFER_BYTE_CAPACITY (DATA_BUFFER_LED_CAPACITY * 3)

/* Список каналов контроллера */
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

#define CHANNELS_TIMER_1 0
#define CHANNELS_TIMER_8 1

#define PWM_1 0
#define PWM_2 1
#define PWM_3 2
#define PWM_4 3

#define CHANNEL_COUNT 8

#endif /* __T6000_H */
