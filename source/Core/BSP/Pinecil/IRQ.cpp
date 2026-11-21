/*
 * IRQ.c
 *
 *  Created on: 30 May 2020
 *      Author: Ralim
 */

#include "IRQ.h"
#include "Pins.h"
#include "configuration.h"
volatile uint8_t  i2c_read_process  = 0;
volatile uint8_t  i2c_write_process = 0;
volatile uint8_t  i2c_slave_address = 0;
volatile uint8_t  i2c_error_code    = 0;
volatile uint8_t *i2c_write;
volatile uint8_t *i2c_read;
volatile uint16_t i2c_nbytes;
volatile uint16_t i2c_write_dress;
volatile uint16_t i2c_read_dress;
volatile uint8_t  i2c_process_flag = 0;
static bool       fastPWM;
static void       switchToSlowPWM(void);
static void       switchToFastPWM(void);
/**
 * @brief ADC 注入组转换完成中断：每个 PWM 周期的采样窗口结束时触发
 * @note  这里是温控环的"心跳"：只做清标志 + 唤醒 PID 任务两件事，
 *        温度换算与 PID 计算都留在线程里，ISR 保持几十条指令量级。
 */
void              ADC0_1_IRQHandler(void) {

  adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
  // unblock the PID controller thread
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pidTaskNotification) {
      vTaskNotifyGiveFromISR(pidTaskNotification, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

/*
 * PWM 硬件模型（Pinecil / GD32VF103，TIMER1 主定时器 + TIMER2 输出门控）：
 *   一个周期 = powerPWM(255) 个加热 tick + tempMeasureTicks(14) 个采样 tick
 *              + holdoffTicks(10) 个死区 tick
 *   TIMER1_CH0 的比较值决定"本周期加热多少 tick"（PWM1 模式，低有效），
 *   到达后 CH1 中断把 TIMER2 的输出强制拉 0（硬件级关断，不依赖 CPU）；
 *   周期末的 UP 中断装载下一周期的占空并处理快/慢模式切换。
 * PWMSafetyTimer 是"软件看门狗"：setTipPWM 每次把它置 10，周期中断里自减；
 * 若 PID 任务停止调用 setTipPWM，10 个周期后输出自动关闭——即使任务卡死，
 * 烙铁也不会持续全功率加热。
 */
volatile uint16_t PWMSafetyTimer = 0;
volatile uint8_t  pendingPWM     = 0;
/**
 * @brief 主 PWM 定时器中断：周期起点装载占空，CH1 时刻强制关断输出
 * @note  两个中断共用一个 handler，必须分别查标志：
 *        UP（周期溢出）→ 递减安全计数、按需切换快慢模式、装载占空；
 *        CH1（比较匹配）→ 立即把 TIMER2 输出清零（结束本周期加热）。
 *        这里不做任何浮点/除法，保持中断短小；占空换算早在 PID 任务里完成。
 */
void              TIMER1_IRQHandler(void) {
  static bool lastPeriodWasFast = false;

  if (timer_interrupt_flag_get(TIMER1, TIMER_INT_UP) == SET) {
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_UP);
    // rollover turn on output if required
    // 安全计数为 0 时不再更新占空，且保持输出关闭：这是"PID 停转"的兜底。
    // 只有 setTipPWM 重新置 10，输出才会恢复。
    if (PWMSafetyTimer) {
      PWMSafetyTimer--;
      if (lastPeriodWasFast != fastPWM) {
        if (fastPWM) {
          switchToFastPWM();
        } else {
          switchToSlowPWM();
        }
      }
      // pendingPWM 是本周期要输出的比较值；为 0 时直接关断 TIMER2，
      // 避免写比较值 0 在 PWM1 模式下产生一个 tick 的窄脉冲。
      if (pendingPWM) {
        timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, pendingPWM);
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, 50);
      } else {
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, 0);
      }
    }
  }
  if (timer_interrupt_flag_get(TIMER1, TIMER_INT_CH1) == SET) {
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_CH1);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, 0);
  }
}

/**
 * @brief 切到快 PWM：约 1 kHz、周期 279 tick
 * @note  预分频 18000，计数时钟 = 主频/18000；总周期 powerPWM+采样+死区 = 279，
 *        因此频率 ≈ 时钟/279。快模式适合小占空比：周期短 → 温度纹波小。
 *        CH0 比较值 = powerPWM+holdoffTicks，即"加热段结束、进入采样窗口"的时刻。
 */
void switchToFastPWM(void) {
  fastPWM           = true;
  totalPWM          = powerPWM + tempMeasureTicks + holdoffTicks;
  TIMER_CAR(TIMER1) = (uint32_t)totalPWM;

  // ~10Hz
  TIMER_CH0CV(TIMER1) = powerPWM + holdoffTicks;
  // 1 kHz tick rate
  TIMER_PSC(TIMER1) = 18000;
}

/**
 * @brief 切到慢 PWM：约 500 Hz、周期减半（采样与死区窗口也减半）
 * @note  大占空比时用慢模式，降低 MOSFET 开关次数与驱动损耗；
 *        采样窗口仍够 ADC 完成 4 次注入转换（硬件保证），所以不影响测温。
 */
void switchToSlowPWM(void) {
  // 5Hz
  fastPWM             = false;
  totalPWM            = powerPWM + tempMeasureTicks / 2 + holdoffTicks / 2;
  TIMER_CAR(TIMER1)   = (uint32_t)totalPWM;
  TIMER_CH0CV(TIMER1) = powerPWM + holdoffTicks / 2;
  TIMER_PSC(TIMER1)   = 36000;
}
/**
 * @brief 下发本周期的占空与快慢模式（由 PID 任务调用）
 * @param pulse              比较值 0..powerPWM
 * @param shouldUseFastModePWM true = 快 PWM
 * @note  只写两个 volatile 变量，真正的寄存器写入放到周期中断里做，
 *        保证占空在周期边界生效，不会切出畸形脉冲（避免炸管/电流尖峰）。
 *        同时重置 PWMSafetyTimer=10，作为"控制环还活着"的心跳。
 */
void setTipPWM(const uint8_t pulse, const bool shouldUseFastModePWM) {
  // 10 个周期 ≈ 10~20 ms 的容忍窗口：PID 正常 8 Hz 调用，余量充足；
  // 一旦任务被长时间阻塞（例如 I2C 卡死），输出会在 ~20 ms 内自动归零。
  PWMSafetyTimer = 10; // This is decremented in the handler for PWM so that the tip pwm is
                       // disabled if the PID task is not scheduled often enough.
  pendingPWM = pulse;
  fastPWM    = shouldUseFastModePWM;
}
extern osThreadId POWTaskHandle;

/**
 * @brief FUSB302 中断线（PB5）下降沿：通知 POWTask 处理 PD 事件
 * @note  用 xTaskNotifyFromISR(eSetBits) 而不是队列：只需"有事件"这一位信息，
 *        通知比队列轻量（无需分配内存），也不会因为事件堆积而阻塞。
 *        POWTaskHandle 可能尚未创建（启动早期），所以先判空。
 */
void EXTI5_9_IRQHandler(void) {
#ifdef POW_PD
  if (RESET != exti_interrupt_flag_get(EXTI_5)) {
    exti_interrupt_flag_clear(EXTI_5);

    if (POWTaskHandle != nullptr) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xTaskNotifyFromISR(POWTaskHandle, 1, eSetBits, &xHigherPriorityTaskWoken);
      /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
      The macro used to do this is dependent on the port and may be called
      portEND_SWITCHING_ISR. */
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
#endif
}

// FUSB302 的 IRQ 是低有效：返回 true 表示"还有未处理的事件"。
// POWTask 用它决定是否要继续 step，而不是马上去睡。
bool getFUS302IRQLow() {
  // Return true if the IRQ line is still held low
  return (RESET == gpio_input_bit_get(FUSB302_IRQ_GPIO_Port, FUSB302_IRQ_Pin));
}
// These are unused for now
void I2C0_EV_IRQHandler(void) {}

void I2C0_ER_IRQHandler(void) {}
