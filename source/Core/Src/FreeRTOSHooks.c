/*
 * 【中文导读】FreeRTOSHooks.c —— FreeRTOS 钩子（B级导读）
 * 栈溢出钩子/空闲任务钩子/断言失败处理——产品级固件的兜底逻辑。
 */

/*
 * FreeRTOSHooks.c
 *
 *  Created on: 29 May 2020
 *      Author: Ralim
 */

#include "FreeRTOSHooks.h"
#include "BSP.h"
/*
 * ============================================================================
 * 【文件导读】FreeRTOSHooks.c —— FreeRTOS 钩子函数
 * ----------------------------------------------------------------------------
 *   vApplicationIdleHook()            空闲任务里喂硬件看门狗
 *   vApplicationGetIdleTaskMemory()   为静态分配的 Idle 任务提供 TCB/栈
 *   vApplicationStackOverflowHook()   栈溢出时直接 reboot()（无法安全恢复）
 * 设计要点：看门狗在"空闲任务"里喂，意味着任何任务长期霸占 CPU（不让 Idle
 * 运行）都会触发复位；这与 PID 任务在 setOutputx10WattsViaFilters 里也喂一次
 * 形成双保险：控制环活着 或 系统空闲，二者必居其一。
 * ============================================================================
 */
void vApplicationIdleHook(void) { resetWatchdog(); }

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t  xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer   = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
  /* place for user code */
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  (void)pcTaskName;

  // We dont have a good way to handle a stack overflow at this point in time
  reboot();
}
