#ifndef __MAIN_H
#define __MAIN_H
#include "OLED.hpp"
#include "Setup.h"
#include "Types.h"
#include <stdint.h>
/*
 * ============================================================================
 * 【文件导读】main.hpp —— 跨任务共享的全局声明
 * ----------------------------------------------------------------------------
 * 这里集中声明四个任务入口（startGUITask/startPIDTask/startMOVTask/
 * startPOWTask）与跨文件变量，避免各 .cpp 互相 extern 造成隐式耦合。
 *   currentTempTargetDegC  volatile：GUI 写、PID 读（单次 16 位写是原子的）
 *   pidTaskNotification    ADC ISR → PID 任务的直达通知句柄
 *   powerSupplyWattageLimit PD 协商得到的瓦数上限（单位 W）
 *   lastMovementTime       MOV 任务写、GUI 读，休眠判定的核心时间戳
 * AccelType 枚举同时被启动告警（是否提示"无加速度计"）和 MOV 任务使用。
 * ============================================================================
 */
extern volatile TemperatureType_t currentTempTargetDegC;
extern bool                       settingsWereReset;
extern bool                       usb_pd_available;
#ifdef __cplusplus
extern "C" {
#endif

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

// Threads
void                startGUITask(void const *argument);
void                startPIDTask(void const *argument);
void                startMOVTask(void const *argument);
void                startPOWTask(void const *argument);
extern TaskHandle_t pidTaskNotification;
extern int32_t      powerSupplyWattageLimit;
extern uint8_t      accelInit;
extern TickType_t   lastMovementTime;
#ifdef __cplusplus
}
// Accelerometer type
enum class AccelType {
  Scanning  = 0,
  None      = 1,
  MMA       = 2,
  LIS       = 3,
  BMA       = 4,
  MSA       = 5,
  SC7       = 6,
  GPIO      = 7,
  LIS_CLONE = 8,
};
extern AccelType DetectedAccelerometerVersion;

#endif
#endif /* __MAIN_H */
