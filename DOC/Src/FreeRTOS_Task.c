//
// Created by WIN on 2026/9/19.
//
#include "main.h"
#include "FreeRTOS_Task.h"
#include "cmsis_os2.h"



osThreadId_t LEDTaskHandle;
osThreadId_t HDC1080TaskHandle;
osThreadId_t USARTTaskHandle;

static osThreadAttr_t LEDTask_attributes = {.name = "LEDTask",.stack_size = 512,.priority = osPriorityLow,};
static osThreadAttr_t HDC1080Task_attributes = {.name = "HDC1080",.stack_size = 1024,.priority = osPriorityNormal,};
static osThreadAttr_t USARTTask_attributes = {.name = "USART",.stack_size = 2024,.priority = osPriorityHigh,};

static void LEDTask(void *pvParameters)
{
    while (1)
    {
        osThreadFlagsWait(0X01,osFlagsWaitAny,osWaitForever);

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        osThreadFlagsSet(HDC1080TaskHandle,0X01);
    }
}

static void HDC1080Task(void *pvParameters)
{
    while (1)
    {
        osThreadFlagsWait(0X01,osFlagsWaitAny,osWaitForever);
    }
}

static void USARTTask(void *pvParameters)
{
    while (1)
    {
        osThreadFlagsWait(0X01,osFlagsWaitAny,osWaitForever);
    }
}

void MX_FREERTOS_Init(void)
{
    LEDTaskHandle = osThreadNew(LEDTask, NULL, &LEDTask_attributes);
    HDC1080TaskHandle = osThreadNew(HDC1080Task, NULL, &HDC1080Task_attributes);
    USARTTaskHandle = osThreadNew(USARTTask, NULL, &USARTTask_attributes);
}