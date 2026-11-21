/*
 * 【中文导读】power.cpp —— 电源原语（B级导读）
 * ADC 采样换算、电压分压计算等底层函数。
 */

/*
 * power.cpp
 *
 *  Created on: 28 Oct, 2018
 *     Authors: Ben V. Brown, David Hilton <- Mostly David
 */

#include <BSP.h>
#include <Settings.h>
#include <power.hpp>

static int32_t PWMToX10Watts(uint8_t pwm, uint8_t sample);
/*
 * 功率-占空比换算层：上层只关心"我要 x10 瓦"，本文件负责换算成 PWM 计数。
 * 关键常量：
 *   powerPWM=255        可用占空计数上限（Pinecil，BSP 定义）
 *   fastPWMChangeoverPoint=128  快/慢 PWM 模式切换点（约 50% 占空）
 *   fastPWMChangeoverTolerance=16 迟滞带宽，防止在切换点附近来回抖
 * 换算链：x10W → PWM(0..255) → 硬件定时器比较值；反向再估回真实功率存入
 * x10WattHistory，供 GUI 显示与功率统计使用。
 */
const int      fastPWMChangeoverPoint     = 128;
const int      fastPWMChangeoverTolerance = 16;

expMovingAverage<uint32_t, wattHistoryFilter> x10WattHistory = {0};

/**
 * @brief 依据占空比决定使用"快 PWM(1 kHz)"还是"慢 PWM(500 Hz)"
 * @param pwmTicks 目标占空计数（0..powerPWM）
 * @return true = 快模式
 * @note  大占空比时用慢模式：周期变长，采样窗口占比更小，MOSFET 开关损耗低；
 *        小占空比时用快模式让温度纹波更小。用 lastPWMWasFast 做迟滞，避免在
 *        阈值附近每周期来回切换（切换本身要重写定时器 preload，代价不小）。
 */
bool shouldBeUsingFastPWMMode(const uint8_t pwmTicks) {
  // Determine if we should use slow or fast PWM mode
  // Crossover between modes set around the midpoint of the PWM control point
  static bool lastPWMWasFast = true;
  if (pwmTicks > (fastPWMChangeoverPoint + fastPWMChangeoverTolerance) && lastPWMWasFast) {
    lastPWMWasFast = false;
  } else if (pwmTicks < (fastPWMChangeoverPoint - fastPWMChangeoverTolerance) && !lastPWMWasFast) {
    lastPWMWasFast = true;
  }
  return lastPWMWasFast;
}

/**
 * @brief 对外唯一的"设定功率"入口：换算 PWM → 下发硬件 → 回读实际功率存档
 * @param mw 目标功率（x10 W，命名沿用历史 mW 说法，实际单位是 0.1 W）
 * @note  调用方是 PID 任务（setOutputx10WattsViaFilters）。这里会额外做一次
 *        ADC 采样（PWMToX10Watts → availableW10），所以不要在中断里调用。
 *        回读值进 24 点滑动平均 x10WattHistory，GUI 的功率条用的就是它。
 */
void setTipX10Watts(int32_t mw) {
  int32_t    outputPWMLevel   = X10WattsToPWM(mw, 1);
  const bool shouldUseFastPWM = shouldBeUsingFastPWMMode(outputPWMLevel);
  setTipPWM(outputPWMLevel, shouldUseFastPWM);
  uint32_t actualMilliWatts = PWMToX10Watts(outputPWMLevel, 0);

  x10WattHistory.update(actualMilliWatts);
}

/**
 * @brief 估算当前电源在 100% 占空下能给出的功率（x10 W）
 * @param sample 0=用最近一次 ADC 结果，1=强制重新采样
 * @return 可用功率（x10 W）
 * @note  推导：P=V²/R。V 以 0.1 V 为单位、R 以 0.1 Ω 为单位，
 *        则 V²/R 的量纲正好是 0.1 W，所以不需要额外缩放。
 *        乘 powerPWM/totalPWM 是因为每个周期都有一段"关断采样窗口"，
 *        占空无法真正到 100%。tipResistance==0 时返回 100（10 W）作为
 *        保守兜底，避免除零并把输出限制在安全范围。
 */
uint32_t availableW10(uint8_t sample) {
  // P = V^2 / R, v*v = v^2 * 100
  //				R = R*10
  // P therefore is in V^2*100/R*10 = W*10.
  uint32_t v             = getInputVoltageX10(getSettingValue(SettingsOptions::VoltageDiv), sample); // 100 = 10v
  uint32_t tipResistance = getTipResistanceX10();
  // 电阻为 0 只可能来自未校准/异常配置；返回 10 W 让后续限幅把输出压到很低，
  // 属于"失效安全"而不是精确值。
  if (tipResistance == 0) {
    return 100; // say 100 watt to force scale down
  }
  uint32_t availableWattsX10 = (v * v) / tipResistance;
  // However, 100% duty cycle is not possible as there is a dead time while the ADC takes a reading
  // Therefore need to scale available milliwats by this

  // avMw=(AvMw*powerPWM)/totalPWM.
  // 占空修正：总周期 = 加热段 powerPWM + 采样窗口 tempMeasureTicks + 死区 holdoffTicks，
  // 因此实际可交付功率要按 powerPWM/totalPWM 打折（Pinecil 上约 255/279 ≈ 91%）。
  availableWattsX10 = availableWattsX10 * powerPWM;
  availableWattsX10 /= totalPWM;

  // availableMilliWattsX10 is now an accurate representation
  return availableWattsX10;
}
/**
 * @brief 把目标功率换算成 PWM 比较值
 * @param x10Watts 目标功率（x10 W），<=0 直接返回 0
 * @param sample   ADC 采样标志，透传给 availableW10
 * @return 0..powerPWM 的占空计数
 * @note  即使请求 0 瓦也要读一次输入电压，让电压滤波器持续更新——否则从
 *        待机切到加热的瞬间会用到很旧的电压值，造成功率跳变。
 */
uint8_t X10WattsToPWM(int32_t x10Watts, uint8_t sample) {
  // Scale input x10Watts to the pwm range available
  if (x10Watts <= 0) {
    // keep the battery voltage updating the filter
    getInputVoltageX10(getSettingValue(SettingsOptions::VoltageDiv), sample);
    return 0;
  }

  // Calculate desired x10Watts as a percentage of availableW10
  uint32_t pwm;
  // 线性映射：占空 = 需求/可用 × 满量程。整数运算先乘后除，避免精度损失；
  // availableW10 已含占空修正，所以这里得到的是"比较值"而非物理占空比。
  pwm = (powerPWM * x10Watts) / availableW10(sample);
  if (pwm > powerPWM) {
    // constrain to max PWM counter
    pwm = powerPWM;
  }
  return pwm;
}

/**
 * @brief 反算：当前 PWM 占空实际对应多少功率（x10 W）
 * @param pwm    占空计数
 * @param sample ADC 采样标志
 * @return 估算功率（x10 W）
 * @note  用于 x10WattHistory 的"实际输出"统计；它假设电源电压在采样瞬间
 *        稳定，因此只适合做显示/统计，不适合作为控制反馈。
 */
static int32_t PWMToX10Watts(uint8_t pwm, uint8_t sample) {
  uint32_t maxMW = availableW10(sample); // Get the milliwatts for the max pwm period
  // Then convert pwm into percentage of powerPWM to get the percentage of the max mw
  return (((uint32_t)pwm) * maxMW) / powerPWM;
}
