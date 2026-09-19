//
// Created by WIN on 2026/9/19.
//


#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS_Task.h"
#include "MYUSART.h"

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM10)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */
    /* TIM4：1 秒周期节拍（Prescaler=9999 / Period=9999 @ APB1 定时器时钟 100MHz）。
     * 在 MX_TIM4_Init() 里以 HAL_TIM_Base_Start_IT() 启动。
     * 中断里只置线程标志，采集/发送在 HDC1080Task / USARTTask 中执行。
     * 注：TIM4 早于 MX_FREERTOS_Init() 就已启动，首拍时句柄可能仍为 NULL，
     *     osThreadFlagsSet() 内部判空返回错误码，不会崩，仅丢弃该次节拍。 */
    if (htim->Instance == TIM4)
    {
        osThreadFlagsSet(HDC1080TaskHandle,BIT_SENSOR_TICK);
    }
    /* USER CODE END Callback 1 */
}

/* ================= 串口模块（MYUSART）中断胶水 =================
 * 模块自身不定义 HAL 弱回调，由工程侧统一转发一次（见模块 README 第 4 步）。
 * 这三个回调运行在 USART1 / DMA2_Stream 中断上下文中，NVIC 优先级为 5，
 * 不低于 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)，
 * 因此模块内部调用 osThreadFlagsSet 合法（configUSE_OS2_EVENTFLAGS_FROM_ISR = 1）。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    SerialModule_OnRxEvent(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    SerialModule_OnTxCplt(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    SerialModule_OnError(huart);
}