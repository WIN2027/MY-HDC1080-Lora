/**
 ******************************************************************************
 * @file    bsp_port_check.c
 * @brief   bsp_port.h 与驱动头文件的一致性 —— 编译期强制校验（不产生任何符号）
 *
 * @details 本文件把 sx1280.h / hdc1080.h 与 bsp_port.h 放进同一个编译单元：
 *          1) 同名移植函数被声明两次，若 bsp_port.h 里的原型与驱动不一致，
 *             GCC 直接报 "conflicting types for ..."（编译失败即暴露问题）；
 *          2) 静态断言校验 bsp_port.h 中的返回值宏与驱动错误码数值一致。
 *
 * @note    本文件只有声明与静态断言，不定义任何函数/变量，
 *          因此不会与 bsp_port.c 里的强符号定义冲突。
 ******************************************************************************
 */

#include "sx1280.h"
#include "hdc1080.h"
#include "bsp_port.h"

/* ---- 返回值数值一致性 ---- */
_Static_assert(SX1280_OK == BSP_RET_OK,
               "BSP_RET_OK 必须等于 SX1280_OK");
_Static_assert(SX1280_ERR_SPI == BSP_SX1280_ERR_SPI,
               "BSP_SX1280_ERR_SPI 必须等于 SX1280_ERR_SPI");
_Static_assert(HDC1080_OK == BSP_RET_OK,
               "BSP_RET_OK 必须等于 HDC1080_OK");
_Static_assert(HDC1080_ERR_I2C == BSP_HDC1080_ERR_I2C,
               "BSP_HDC1080_ERR_I2C 必须等于 HDC1080_ERR_I2C");

/* ---- 帧长上限：驱动声明的单帧上限为 259 字节（读缓冲 255 + 4） ---- */
_Static_assert(BSP_SX1280_FRAME_MAX >= 259U,
               "BSP_SX1280_FRAME_MAX 必须覆盖驱动声明的 259 字节单帧上限");

/* 文件结束 */
