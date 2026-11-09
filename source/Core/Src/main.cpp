/*
 * ============================================================================
 * 【中文导读】main.cpp —— IronOS 电烙铁固件入口
 * ----------------------------------------------------------------------------
 * start(): 硬件早期初始化 → 读按钮/加速度计决定休眠 → 创建 4 个 FreeRTOS
 * 任务（GUI/运动/PID/电源）→ 启动调度器。四任务划分见 Threads/ 目录：
 *   GUIThread: 界面绘制与按键；PIDThread: 控温闭环（核心算法）；
 *   MOVThread: 加速度计手势（休眠唤醒）；POWThread: 电源/USB-PD 协商。
 * ============================================================================
 */

// By Ben V. Brown - V2.0 of the TS100 firmware

/*
 * Main.cpp bootstraps the device and then hands over to FreeRTOS and the threads
 */

#include "main.hpp"
#include "BSP.h"
#include "Settings.h"
#include "cmsis_os.h"
#include "power.hpp"
AccelType DetectedAccelerometerVersion = AccelType::Scanning;
bool      settingsWereReset            = false;
// FreeRTOS variables

osThreadId          GUITaskHandle;
static const size_t GUITaskStackSize = 1024 / 2;
uint32_t            GUITaskBuffer[GUITaskStackSize];
osStaticThreadDef_t GUITaskControlBlock;

osThreadId          PIDTaskHandle;
static const size_t PIDTaskStackSize = 1024 / 2;
uint32_t            PIDTaskBuffer[PIDTaskStackSize];
osStaticThreadDef_t PIDTaskControlBlock;

osThreadId          MOVTaskHandle;
static const size_t MOVTaskStackSize = 1024 / 2;
uint32_t            MOVTaskBuffer[MOVTaskStackSize];
osStaticThreadDef_t MOVTaskControlBlock;

osThreadId          POWTaskHandle;
static const size_t POWTaskStackSize = 512 / 2;
uint32_t            POWTaskBuffer[POWTaskStackSize];
osStaticThreadDef_t POWTaskControlBlock;

// End FreeRTOS
// Main sets up the hardware then hands over to the FreeRTOS kernel
int main(void) {
  preRToSInit();

  resetWatchdog();
  // Testing for which accelerometer is mounted
  settingsWereReset = loadSettings(); // load the settings from flash

  setTipX10Watts(0); // force tip off

  resetWatchdog();
  /* Create the thread(s) */

  /* definition and creation of PIDTask - Heating control*/
  osThreadStaticDef(PIDTask, startPIDTask, osPriorityRealtime, 0, PIDTaskStackSize, PIDTaskBuffer, &PIDTaskControlBlock);
  PIDTaskHandle = osThreadCreate(osThread(PIDTask), NULL);

  /* definition and creation of POWTask - Power management for QC / PD */
  osThreadStaticDef(POWTask, startPOWTask, osPriorityAboveNormal, 0, POWTaskStackSize, POWTaskBuffer, &POWTaskControlBlock);
  POWTaskHandle = osThreadCreate(osThread(POWTask), NULL);

  /* definition and creation of MOVTask - Accelerometer management */
  osThreadStaticDef(MOVTask, startMOVTask, osPriorityNormal, 0, MOVTaskStackSize, MOVTaskBuffer, &MOVTaskControlBlock);
  MOVTaskHandle = osThreadCreate(osThread(MOVTask), NULL);

  /* definition and creation of GUITask - The OLED control & update*/
  osThreadStaticDef(GUITask, startGUITask, osPriorityBelowNormal, 0, GUITaskStackSize, GUITaskBuffer, &GUITaskControlBlock);
  GUITaskHandle = osThreadCreate(osThread(GUITask), NULL);

  resetWatchdog();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  for (;;) {
  }
}
