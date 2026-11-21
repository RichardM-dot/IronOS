/*
 * ============================================================================
 * 【中文导读】PIDThread.cpp —— 控温 PID 闭环（本项目灵魂，精读！）
 * ----------------------------------------------------------------------------
 * 控制回路：ADC 采样烙铁头热电偶 → 温度换算（校准系数）→ PID 计算 →
 * PWM 驱动加热。亮点：采样与加热分时复用 ADC（加热时热电偶读数无效），
 * PID 参数按烙铁头型号校准，输入滤波+输出限幅都是工程级写法。
 * ============================================================================
 */

/*
 * PIDThread.cpp
 *
 *  Created on: 29 May 2020
 *      Author: Ralim
 */

#include "BSP.h"
#include "FreeRTOS.h"
#include "Settings.h"
#include "TipThermoModel.h"
#include "cmsis_os.h"
#include "configuration.h"
#include "history.hpp"
#include "main.hpp"
#include "power.hpp"
#include "task.h"

#ifdef POW_PD
#if POW_PD == 1
#include "USBPD.h"
#endif
#endif

/*
 * 全局状态（被中断与其它任务共享，volatile 只保护"单次读写的可见性"，不保护一致性）：
 *   pidTaskNotification         ADC ISR 用来唤醒本任务的句柄，创建任务后立刻保存
 *   currentTempTargetDegC       当前设定温度（C），由 GUI 模式的各函数写入
 *   powerSupplyWattageLimit     电源协商得到的功率上限（单位 W），由 USBPD 写入；
 *                               使用时再 ×10 换算成 x10 W 与其它限值比较
 *   heaterThermalRunawayCounter 热失控"积分"计数，>8 时强制断输出（见下）
 * KeepAwakePulse 的时间基准：设置项只存 1..9 的倍率，实际等待 2.5 s*倍率、
 * 脉宽 250 ms*倍率（见 setOutputx10WattsViaFilters）；TICKS_100MS=100 ticks（1 tick=1 ms）。
 */
static TickType_t          powerPulseWaitUnit          = 25 * TICKS_100MS;      // 2.5 s
static TickType_t          powerPulseDurationUnit      = (5 * TICKS_100MS) / 2; // 250 ms
TaskHandle_t               pidTaskNotification         = NULL;
volatile TemperatureType_t currentTempTargetDegC       = 0; // Current temperature target in C
int32_t                    powerSupplyWattageLimit     = 0;
uint8_t                    heaterThermalRunawayCounter = 0;

static int32_t getPIDResultX10Watts(TemperatureType_t set_point, TemperatureType_t current_value);
static void    detectThermalRunaway(const TemperatureType_t currentTipTempInC, const uint32_t x10WattsOut);
static void    setOutputx10WattsViaFilters(int32_t x10Watts);
int32_t        getX10WattageLimits();

/**
 * @brief PID 控温任务：阻塞等 ADC 中断 → 读温度 → 算功率 → 下发 PWM
 * @param argument 未使用（osThreadStaticDef 要求的签名）
 * @note  优先级 osPriorityRealtime（全系统最高），实时性由它兜底。它不轮询：
 *        ADC 注入组在 PWM 周期采样窗口结束时进 ADC0_1_IRQHandler，ISR 里
 *        vTaskNotifyGiveFromISR 唤醒本任务，所以控制周期由硬件定时器决定，
 *        软件调度抖动不会改变采样相位。
 *        共享数据：currentTempTargetDegC（GUI 写）、powerSupplyWattageLimit
 *        （POWTask/USBPD 写）、heaterThermalRunawayCounter（本任务写、GUI 读）。
 */
/* StartPIDTask function */
void startPIDTask(void const *argument __unused) {
  /*
   * We take the current tip temperature & evaluate the next step for the tip
   * control PWM.
   */
  setTipX10Watts(0); // disable the output at startup

  currentTempTargetDegC = 0; // Force start with no output (off). If in sleep / soldering this will
                             // be over-ridden rapidly

  pidTaskNotification = xTaskGetCurrentTaskHandle();

  // 预热 32 次采样：TipThermoModel 与输入电压都带滑动滤波，空滤波器初值为 0，
  // 直接进闭环会算出巨大误差。每次最多等 5 tick，等不到也继续，属于"尽力预填充"。
  TemperatureType_t PIDTempTarget = 0;
  // Pre-seed the adc filters
  for (int i = 0; i < 32; i++) {
    ulTaskNotifyTake(pdTRUE, 5);
    TipThermoModel::getTipInC(true);
    getInputVoltageX10(getSettingValue(SettingsOptions::VoltageDiv), 1);
  }

  // 板级自检（MHP30 用它测加热板电阻）；2 s 超时兜底，避免自检永不返回导致不加热。
  while (preStartChecks() == 0) {
    resetWatchdog();
    ulTaskNotifyTake(pdTRUE, 2000);
  }
// Wait for PD if its in the middle of negotiation
#ifdef POW_PD
#if POW_PD == 1
  // This is an FUSB based PD capable device
  // Wait up to 3 seconds for USB-PD to settle
  while (USBPowerDelivery::negotiationInProgress() && xTaskGetTickCount() < (TICKS_SECOND * 3)) {
    resetWatchdog();
    ulTaskNotifyTake(pdTRUE, TICKS_100MS);
  }
#endif
#endif

#ifdef MCU_TEMP_CUTOFF_C
  // Reference the die temperature sensor to the handle NTC now that the ADC has samples (cold boot: same temperature)
  calibrateMCUTemperature(getHandleTemperature(0) / 10);
#endif

  int32_t    x10WattsOut             = 0;
  TickType_t lastThermalRunawayDecay = xTaskGetTickCount();

  for (;;) {
    x10WattsOut = 0;
    // This is a call to block this thread until the ADC does its samples
    if (ulTaskNotifyTake(pdTRUE, TICKS_SECOND * 2)) {
      // Do the reading here to keep the temp calculations churning along
      TemperatureType_t currentTipTempInC = TipThermoModel::getTipInC(true);

      PIDTempTarget = currentTempTargetDegC;
#ifdef POW_DC
      // Never drive the tip below the DC minimum input voltage (9 V, or the battery cell cut-off). The soldering
      // mode exits on undervoltage as well, but only after its 2 s ADC settling delay; a 5 V USB host port would
      // already have been overloaded and dropped VBUS by then.
      if (PIDTempTarget > 0 && getIsPoweredByDCIN() && getInputVoltageX10(getSettingValue(SettingsOptions::VoltageDiv), 0) < lookupVoltageLevel()) {
        PIDTempTarget = 0;
      }
#endif
#ifdef MCU_TEMP_CUTOFF_C
      // Last line of defence for the output stage: if the MCU die gets this hot, stop heating and raise the
      // thermal runaway warning (the handle NTC derate should have kicked in long before this)
      if (getMCUTemperatureC() > MCU_TEMP_CUTOFF_C) {
        PIDTempTarget               = 0;
        heaterThermalRunawayCounter = 255;
      }
#endif
      if (PIDTempTarget > 0) {
        // Cap the max set point to 450C
        if (PIDTempTarget > 450) {
          // Maximum allowed output
          PIDTempTarget = 450;
        }
        // Safety check that not aiming higher than current tip can measure
        if (PIDTempTarget > TipThermoModel::getTipMaxInC()) {
          PIDTempTarget = TipThermoModel::getTipMaxInC();
        }

      // 注意：目标为 0 时不调用 PID，而是直接把功率按 0 处理并喂给热失控检测，
      // 这样"关机状态下温度仍上升"也能被 detectThermalRunaway 捕获。
        x10WattsOut = getPIDResultX10Watts(PIDTempTarget, currentTipTempInC);
        detectThermalRunaway(currentTipTempInC, x10WattsOut);
      } else {
        detectThermalRunaway(currentTipTempInC, 0);
      }
      setOutputx10WattsViaFilters(x10WattsOut);
    } else {
      // ADC interrupt timeout
      // 拿不到新温度就绝不允许继续加热：宁可掉温，也不能用陈旧读数闭环。
      setTipPWM(0, false);
    }
#ifdef DEBUG_UART_OUTPUT
    log_system_state(x10WattsOut);
#endif
    // 热失控计数每秒衰减 1：必须"持续异常"才能累积到触发阈值（>8，见过滤函数）。
    if (xTaskGetTickCount() - lastThermalRunawayDecay > TICKS_SECOND) {
      lastThermalRunawayDecay = xTaskGetTickCount();
      if (heaterThermalRunawayCounter > 0) {
        heaterThermalRunawayCounter--;
      }
    }
  }
}

#ifdef TIP_CONTROL_PID
/**
 * @brief 编译期参数化的位置式 PID（Kp/Ki/Kd 来自板级 configuration.h）
 * @tparam T 运算类型，本工程实例化为 TemperatureType_t（int16_t）
 * @note  与下面的 Integrator 二选一编译（TIP_CONTROL_PID 开关）：PID 要整定三个
 *        参数，Integrator 只需"热容/惯性"两个物理量。
 */
template <class T, T Kp, T Ki, T Kd, T integral_limit_scale> struct PID {
  T previous_error_term;
  T integration_running_sum;

  T update(const T set_point, const T new_reading, const TickType_t interval_ms, const T max_output) {
    const T target_delta = set_point - new_reading;

    // Proportional term
    const T kp_result = Kp * target_delta;

    // 积分项按实际间隔归一化：采样率不固定（ADC 周期会随快/慢 PWM 切换变化），
    // 所以乘 interval_ms 再除 1000 得到"每秒"等效增量，避免切换 PWM 模式时积分突变。
    // Integral term as we use mixed sampling rates, we cant assume a constant sample interval
    // Thus we multiply this out by the interval time to ~= dv/dt
    // Then the shift by 1000 is ms -> Seconds

    integration_running_sum += (target_delta * (T)interval_ms * Ki) / 1000;

    // 抗积分饱和：夹在 ±(5×最大输出)。冷启动误差几百 C，没有这个夹子积分会瞬间
    // 涨到天文数字，到温后要很久才能退饱和 → 典型的大幅超调。
    // We constrain integration_running_sum to limit windup
    // This is not overly required for most use cases but can prevent large overshoot in constrained implementations
    if (integration_running_sum > integral_limit_scale * max_output) {
      integration_running_sum = integral_limit_scale * max_output;
    } else if (integration_running_sum < -integral_limit_scale * max_output) {
      integration_running_sum = -integral_limit_scale * max_output;
    }
    // Calculate the integral term, we use a shift 100 to get precision in integral as we often need small amounts
    T ki_result = integration_running_sum / 100;

    // Derivative term
    T derivative = (target_delta - previous_error_term);
    T kd_result  = ((Kd * derivative) / (T)(interval_ms));

    // Summation of the outputs
    T output = kp_result + ki_result + kd_result;

    // Restrict to max / 0
    if (output > max_output) {
      output = max_output;
    } else if (output < 0) {
      output = 0;
    }

    // Save target_delta to previous target_delta
    previous_error_term = target_delta;

    return output;
  }
};
#else
/**
 * @brief 自衰减积分器（IronOS 默认控制器）：以"热容 × 温差"作为需求功率
 * @note  不用标准 PID 的原因：热电偶在两次加热间隙采样，读数滞后且噪声大，
 *        微分项会被噪声放大；纯积分在大误差时又太慢。这里的写法是"带泄漏的
 *        积分 + 比例型增益"：温差大时接近 P，接近设定点时逐渐退化为纯 I
 *        （getPIDResultX10Watts 上方有原作者的长篇推导）。
 */
template <class T = TemperatureType_t> struct Integrator {
  T sum;

  T update(const T val, const int32_t inertia, const int32_t gain, const int32_t rate, const int32_t limit) {
    // Decay the old value. This is a simplified formula that still works with decent results
    // Ideally we would have used an exponential decay but the computational effort required
    // by exp function is just not justified here in respect to the outcome
    sum = (sum * (100 - (inertia / rate))) / 100;
    // Add the new value x integration interval ( 1 / rate)
    sum += (gain * val) / rate;

    // constrain the output between +- our max power output, this limits windup when doing the inital heatup or when solding something large
    if (sum > limit) {
      sum = limit;
    } else if (sum < -limit) {
      sum = -limit;
    }

    return sum;
  }

  // 直接重置积分状态；切换工作模式（如进入 Boost）时可用它清历史。
  // get(positiveOnly) 默认只返回正向累积，避免"负功率"被当成制冷指令。
  void set(T const val) { sum = val; }

  T get(bool positiveOnly = true) const { return (positiveOnly) ? ((sum > 0) ? sum : 0) : sum; }
};
#endif
/**
 * @brief 计算本周期输出功率（x10 W）
 * @param set_point       目标温度（C）
 * @param current_reading 实测温度（C）
 * @return 0 .. getX10WattageLimits() 之间的功率指令
 * @note  用 static 变量记录上次调用时刻求真实间隔，所以不要求调用频率恒定；
 *        但间隔必须 > 0 tick，否则 rate 会除零（实际由 ADC 周期保证数十 ms）。
 */
int32_t getPIDResultX10Watts(TemperatureType_t set_point, TemperatureType_t current_reading) {
  static TickType_t lastCall = 0;

#ifdef TIP_CONTROL_PID
  // PID 实例与 lastCall 都是函数内 static：每个文件只有一份控制器状态，
  // 且只能在 PID 任务里调用（非线程安全）。
  static PID<TemperatureType_t, TIP_PID_KP, TIP_PID_KI, TIP_PID_KD, 5> pid = {0, 0};

  const TickType_t interval = (xTaskGetTickCount() - lastCall);

#else
  static Integrator<TemperatureType_t> powerStore = {0};
  const TickType_t                     rate       = TICKS_SECOND / (xTaskGetTickCount() - lastCall);
#endif
  lastCall = xTaskGetTickCount();
  // Sandman note:
  // PID Challenge - we have a small thermal mass that we to want heat up as fast as possible but we don't
  // want to overshot excessively (if at all) the set point temperature. In the same time we have 'imprecise'
  // instant temperature measurements. The nature of temperature reading imprecision is not necessarily
  // related to the sensor (thermocouple) or DAQ system, that otherwise are fairly decent. The real issue	is
  // the thermal inertia. We basically read the temperature in the window between two heating sessions when
  // the output is off. However, the heater temperature does not dissipate instantly into the tip mass so
  // at any moment right after heating, the thermocouple would sense a temperature significantly higher than
  // moments later. We could use longer delays but that would slow the PID loop and that would lead to other
  // negative side effects. As a result, we can only rely on the I term but with a twist. Instead of a simple
  // integrator we are going to use a self decaying integrator that acts more like a dual I term / P term
  // rather than a plain I term. Depending on the circumstances, like when the delta temperature is large,
  // it acts more like a P term whereas on closing to set point it acts increasingly closer to a plain I term.
  // So in a sense, we have a bit of both.
  //																		 So there we go...

  // P = (Thermal Mass) x (Delta Temperature ) / 1sec, where thermal mass is in X10 J / °C and
  // delta temperature is in °C. The result is the power in X10 W needed to raise (or decrease!) the
  // tip temperature with (Delta Temperature ) °C in 1 second.
  // Note on powerStore. On update, if the value is provided in X10 (W) units then inertia shall be provided
  // in X10 (J / °C) units as well.

#ifdef TIP_CONTROL_PID
  return pid.update(set_point, current_reading, interval, getX10WattageLimits());
#else
  // 四个入参的物理含义：
  //   val   = 热容(x10 J/C) × 温差(C)  → 1 秒内补齐温差所需的 x10 W
  //   inertia = 惯性，越小旧值权重越大（平滑但响应慢）
  //   gain  = 2，固定增益（原作者调参结果）
  //   rate  = 控制频率 Hz（TICKS_SECOND/间隔），用于把"每秒"换算成"每周期"
  //   limit = 当前可用功率上限（电源/用户/硬件/手柄降额的综合结果）
  return powerStore.update(((TemperatureType_t)getTipThermalMass()) * (set_point - current_reading), // the required power
                           getTipInertia(),                                                          // Inertia, smaller numbers increase dominance of the previous value
                           2,                                                                        // gain
                           rate,                                                                     // PID cycle frequency
                           getX10WattageLimits());
#endif
}

/*
 * Detection of thermal runaway
 * The goal of this is to handle cases where something has gone wrong
 * 1. The tip MOSFET is broken, so power is being constantly applied to the tip
 * a. This can show as temp being stuck at max
 * b. Or temp rising when the heater is off
 * 2. Broken temperature sense
 * a. Temp is stuck at a value
 * These boil down to either a constantly rising temperature or a temperature that is stuck at a value
 * These are both covered; but looking at the eye/delta between min and max temp seen
 */
/**
 * @brief 热失控检测：加热一段时间后温度没有"应有的上升幅度"就累加告警计数
 * @param currentTipTempInC 当前烙铁头温度（C）
 * @param x10WattsOut       本周期输出功率（x10 W），>0 视为"正在加热"
 * @note  覆盖两类故障：① 加热 MOSFET 击穿 → 关断时温度仍上升或顶到 ADC 满量程；
 *        ② 测温链路损坏 → 温度卡在某个值，加热 20 s 后温差仍小于 3 C。
 *        计数由 PID 主循环每秒衰减 1，只有持续异常才会累积到 >8 触发关机。
 */
void detectThermalRunaway(const TemperatureType_t currentTipTempInC, const uint32_t x10WattsOut) {

  // 这些 static 是跨调用的"观测窗口"，一次检测横跨数十秒，不能放在栈上。
  // haveSeenDelta 一旦置位就永久跳过后续统计（每次上电只判定一次）。
  static TemperatureType_t tiptempMin         = 0xFFFF; // Min tip temp seen
  static TemperatureType_t tipTempMax         = 0;      // Max tip temp seen while heater is on
  bool                     thisCycleIsHeating = x10WattsOut > 0;
  static TickType_t        heatCycleStart     = 0;

  static bool haveSeenDelta = false;

  // Check for readings being pegged at the top of the ADC while the heater is off
  // 场景 ①b：加热已关断，原始 ADC 却贴顶（满量程 32768，留 8 计数噪声余量）
  // → 说明功率管可能击穿、热电偶仍在被加热，直接累加计数（每秒最多 +1）。
  if (!thisCycleIsHeating && (getTipRawTemp(0) > (ADC_MAX_READING - 8)) && heaterThermalRunawayCounter < 255) {
    heaterThermalRunawayCounter++;
  }

  if (haveSeenDelta) {
    return;
  }

  if (currentTipTempInC < tiptempMin) {
    tiptempMin = currentTipTempInC;
  }
  if (thisCycleIsHeating && currentTipTempInC > tipTempMax) {
    tipTempMax = currentTipTempInC;
  }
  if (thisCycleIsHeating) {
    if (heatCycleStart == 0) {
      heatCycleStart = xTaskGetTickCount();
    }
  } else {
    heatCycleStart = 0;
  }

  // 场景 ②：连续加热满 THERMAL_RUNAWAY_TIME_SEC（默认 20 s）后评估温升。
  // 正常烙铁 20 s 至少升温几十度；温差 < THERMAL_RUNAWAY_TEMP_C（3 C）说明测温已失效。
  if ((xTaskGetTickCount() - heatCycleStart) > (THERMAL_RUNAWAY_TIME_SEC * TICKS_SECOND)) {
    if (tipTempMax > tiptempMin) {
      // Have been heating for min seconds, check if the delta is large enough
      TemperatureType_t delta = tipTempMax - tiptempMin;
      haveSeenDelta           = true;

      if (delta < THERMAL_RUNAWAY_TEMP_C && heaterThermalRunawayCounter < 255) {
        heaterThermalRunawayCounter++;
      }
    }
  }
}

/**
 * @brief 汇总所有功率限制，返回本周期允许的最大输出（x10 W）
 * @return 可用功率上限
 * @note  取四级限制的最小值：电源能力(availableW10) → 用户 PowerLimit →
 *        PD/QC 协商的电源瓦数 → 硬件额定(ENFORCE_HARDWARE_MAX_WATTAGE)，
 *        最后再按手柄 NTC 温度做线性降额（HANDLE_DERATE_*）。
 *        每次调用都会读 ADC（availableW10 内部采样），所以不要在 ISR 里调用。
 */
int32_t getX10WattageLimits() {
  int32_t limit = availableW10(0);

  if (getSettingValue(SettingsOptions::PowerLimit) && limit > (getSettingValue(SettingsOptions::PowerLimit) * 10)) {
    limit = getSettingValue(SettingsOptions::PowerLimit) * 10;
  }
  if (powerSupplyWattageLimit && limit > powerSupplyWattageLimit * 10) {
    limit = powerSupplyWattageLimit * 10;
  }
#ifdef ENFORCE_HARDWARE_MAX_WATTAGE
  // Never exceed what the output stage is rated for, even on DC input where there is no supply limit
  if (limit > HARDWARE_MAX_WATTAGE_X10) {
    limit = HARDWARE_MAX_WATTAGE_X10;
  }
#endif
#ifdef HANDLE_DERATE_START_C
  // Derate the output when the handle is getting hot; the MOSFET shares the handle with the NTC.
  // Linear ramp from 100% at HANDLE_DERATE_START_C down to HANDLE_DERATE_MIN_PERCENT at HANDLE_DERATE_END_C.
  {
    const int32_t handleC = getHandleTemperature(0) / 10;
    if (handleC > HANDLE_DERATE_START_C) {
      int32_t percent = 100 - (((handleC - HANDLE_DERATE_START_C) * (100 - HANDLE_DERATE_MIN_PERCENT)) / (HANDLE_DERATE_END_C - HANDLE_DERATE_START_C));
      if (percent < HANDLE_DERATE_MIN_PERCENT) {
        percent = HANDLE_DERATE_MIN_PERCENT;
      }
      limit = (limit * percent) / 100;
    }
  }
#endif
  return limit;
}

/**
 * @brief 输出前的最后一道"过滤 + 保护"，然后真正写 PWM
 * @param x10WattsOut PID 计算出的功率指令（x10 W）
 * @note  依次做：KeepAwakePulse 抬底 → ADC 贴顶保护 → 热失控计数保护 →
 *        SLEW_LIMIT 斜率限制 → setTipX10Watts。这里也是唯一喂看门狗的地方，
 *        意味着"控制环停转"必然导致看门狗复位，而不是悄悄停止加热。
 */
void setOutputx10WattsViaFilters(int32_t x10WattsOut) {
  static TickType_t lastPowerPulseStart = 0;
  static TickType_t lastPowerPulseEnd   = 0;
#ifdef SLEW_LIMIT
  static int32_t x10WattsOutLast = 0;
#endif

  // If the user turns on the option of using an occasional pulse to keep the power bank on
  if (getSettingValue(SettingsOptions::KeepAwakePulse)) {
    const TickType_t powerPulseWait = powerPulseWaitUnit * getSettingValue(SettingsOptions::KeepAwakePulseWait);
    if (xTaskGetTickCount() - lastPowerPulseStart > powerPulseWait) {
      const TickType_t powerPulseDuration = powerPulseDurationUnit * getSettingValue(SettingsOptions::KeepAwakePulseDuration);
      lastPowerPulseStart                 = xTaskGetTickCount();
      lastPowerPulseEnd                   = lastPowerPulseStart + powerPulseDuration;
    }

    // If current PID is less than the pulse level, check if we want to constrain to the pulse as the floor
    if (x10WattsOut < getSettingValue(SettingsOptions::KeepAwakePulse) && xTaskGetTickCount() < lastPowerPulseEnd) {
      x10WattsOut = getSettingValue(SettingsOptions::KeepAwakePulse);
    }
  }

  // 与 detectThermalRunaway 里的贴顶判断互补：那里累计计数，这里立即断输出。
  // 阈值 0x7FFF-32 表示已进入满量程最后 32 个计数（约 0.1% 量程）。
  // Secondary safety check to forcefully disable header when within ADC noise of top of ADC
  if (getTipRawTemp(0) > (0x7FFF - 32)) {
    x10WattsOut = 0;
  }
  // 计数 >8 才真正切断：单次异常（例如拔插烙铁头造成的瞬时贴顶）不会误关机，
  // 连续 9 秒以上的异常才判定为热失控。
  if (heaterThermalRunawayCounter > 8) {
    x10WattsOut = 0;
  }
#ifdef SLEW_LIMIT
  if (x10WattsOut - x10WattsOutLast > SLEW_LIMIT) {
    x10WattsOut = x10WattsOutLast + SLEW_LIMIT;
  }
  if (x10WattsOut < 0) {
    x10WattsOut = 0;
  }
  x10WattsOutLast = x10WattsOut;
#endif
  // setTipX10Watts 内部会做 W→PWM 的电压补偿换算；此处调用频率 = ADC 周期，
  // 也是整条链路里唯一"写硬件"的动作。
  setTipX10Watts(x10WattsOut);
  // 看门狗只在成功走完一次控制周期后重装：任何一步卡死都会让 IWDG 溢出复位。
  resetWatchdog();
}
