//
// Created by WIN on 2026/9/19.
//


#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS_Task.h"

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
     * 中断里只释放信号量，采集/发射/上报在 SensorTask 中执行。 */
    if (htim->Instance == TIM4)
    {
        osThreadFlagsSet(LEDTaskHandle,0X01);
    }
    /* USER CODE END Callback 1 */
}