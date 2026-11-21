/*
 * 【中文导读】POWThread.cpp —— 电源管理任务
 * 电压监测（电池截止保护）+ QC/PD 快充协议协商（配合 QC3.cpp），
 * 动态请求高电压让烙铁快加热。
 */

/*
 * POWThread.cpp
 *
 *  Created on: 16 Jan 2021
 *      Author: Ralim
 */

#include "BSP.h"
#include "FS2711.hpp"
#include "FreeRTOS.h"
#include "HUB238.hpp"
#include "QC3.h"
#include "Settings.h"
#include "USBPD.h"
#include "cmsis_os.h"
#include "configuration.h"
#include "main.hpp"
#include "stdbool.h"
#include "stdlib.h"
#include "task.h"

// Small worker thread to handle power (PD + QC) related steps

/**
 * @brief 电源任务：驱动 USB-PD 状态机、处理 FUSB302 中断、必要时跑 QC/其它 PD 芯片
 * @param argument 未使用
 * @note  优先级 osPriorityAboveNormal（低于 PID、高于 MOV/GUI）。它的节拍不是固定
 *        周期，而是"等 FUSB302 的 IRQ 通知，最多等 50 ms"：PD 协议要求收到消息后
 *        在 ~20 ms 内回 GoodCRC，所以不能睡太久；同时用 50 ms 超时保证即使 IRQ
 *        丢失也能继续推进状态机（软件定时器式轮询）。
 */
void startPOWTask(void const *argument __unused) {

  // Init any other misc sensors
  // postRToSInit 里做"必须在调度器启动后才能做"的板级初始化（如 BLE/外设时钟）。
  // 放在本任务而不是 main，是为了让 GUI 先亮屏，用户感知不到这段延迟。
  postRToSInit();
  // 等 PID 任务把 preStartChecks 跑完（MHP30 的加热板电阻测量）。
  // 3 ms 轮询是折中：足够快以赶上 PD 的 tSenderResponse，又不至于空转抢 CPU。
  while (preStartChecksDone() == 0) {
    osDelay(3);
  }
  // You have to run this once we are willing to answer PD messages
  // Setting up too early can mean that we miss the ~20ms window to respond on some chargers
#ifdef POW_PD
  // 关键时序：start() 内部会做 FUSB302 初始化并打开 IRQ。太早初始化会错过
  // 上电后充电器发 Source_Capabilities 的窗口（部分充电器只发一次），
  // 所以必须等调度器起来、VBUS 稳定后再开始。
  USBPowerDelivery::start();
  // Crank the handle at boot until we are stable and waiting for IRQ
  USBPowerDelivery::step();
#endif
#if POW_PD_EXT == 2
  FS2711::start();
  FS2711::negotiate();
#endif

  BaseType_t res;
  for (;;) {
    // res 用 pdFALSE 初始化：如果 IRQ 已经是低电平，就不进等待，直接处理。
    // 这里的循环没有 vTaskDelay，靠下面 xTaskNotifyWait 的 50 ms 超时让出 CPU。
    res = pdFALSE;
    // While the interrupt is low, dont delay
    /*This is due to a possible race condition, where:
     * IRQ fires
     * We read interrupt register but dont see the Good CRC
     * Then Good CRC is set while reading it out (racing on I2C read)
     * Then we would sleep as nothing to do, but 100ms> 20ms power supply typical timeout
     */
    // 注释里描述的竞态：IRQ 触发 → 读中断寄存器时没看到 GoodCRC → 读的瞬间
    // GoodCRC 才置位 → 若此时去睡 100 ms，就超过充电器 20 ms 的重发超时。
    // 所以"IRQ 低电平"时只用 50 ms 超时等待，且醒来后无条件再 step 一次。
    if (!getFUS302IRQLow()) {
      res = xTaskNotifyWait(0x0, 0xFFFFFF, NULL, TICKS_100MS / 2);
    }

#ifdef POW_PD
    if (res != pdFALSE || getFUS302IRQLow()) {
      USBPowerDelivery::IRQOccured();
    }
    USBPowerDelivery::PPSTimerCallback();
    USBPowerDelivery::step();

#else
    (void)res;
#endif
#if POW_PD_EXT == 1
    hub238_check_negotiation();
#endif
#if POW_PD_EXT == 2
    FS2711::negotiate();
#endif
    // 板级电源检查：Pinecil 用它决定"PD 失败后是否退到 QC"；
    // 该函数必须能被高频调用且无阻塞（见 BSP/<board>/Power.cpp）。
    power_check();
  }
}
