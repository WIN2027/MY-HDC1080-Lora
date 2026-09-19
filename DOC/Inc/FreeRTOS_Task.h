//
// Created by WIN on 2026/9/19.
//

#ifndef FreeRTOS_Task_H
#define FreeRTOS_Task_H
#include "cmsis_os2.h"

extern osThreadId_t LEDTaskHandle;

void MX_FREERTOS_Init(void);

#endif //FreeRTOS_Task_H
