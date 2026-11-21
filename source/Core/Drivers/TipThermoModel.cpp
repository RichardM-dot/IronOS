/*
 * TipThermoModel.cpp
 *
 *  Created on: 7 Oct 2019
 *      Author: ralim
 */

#include "TipThermoModel.h"
#include "BSP.h"
#include "Settings.h"
#include "Types.h"
#include "Utils.hpp"
#include "configuration.h"
#include "main.hpp"
#include "power.hpp"
/*
 * The hardware is laid out  as a non-inverting op-amp
 * There is a pullup of 39k(TS100) from the +ve input to 3.9V (1M pulup on TS100)
 *
 * The simplest case to model this, is to ignore the pullup resistors influence, and assume that its influence is mostly constant
 * -> Tip resistance *does* change with temp, but this should be much less than the rest of the system.
 *
 * When a thermocouple is equal temperature at both sides (hot and cold junction), then the output should be 0uV
 * Therefore, by measuring the uV when both are equal, the measured reading is the offset value.
 * This is a mix of the pull-up resistor, combined with tip manufacturing differences.
 *
 * All of the thermocouple readings are based on this expired patent
 * - > https://patents.google.com/patent/US6087631A/en
 *
 * This was bought to my attention by <Kuba Sztandera>
 */
/*
 * 温度换算链（每一级都可单独测试）：
 *   raw ADC  12 位 ×8 过采样 → 0..32768
 *     ↓ convertTipRawADCTouV()   按 ADC_VDD_MV/满量程换算成 mV，再 /运放增益 → uV，
 *                                并减去用户校准偏置 CalibrationOffset（uV）
 *     ↓ convertuVToDegC()        查 Hakko K 型热电偶表 + 线性插值（见 BSP/ThermoModel.cpp）
 *     ↓ + 手柄 NTC 的冷端温度    热电偶测的是"热端相对冷端"的温差，必须加冷端
 *     = 烙铁头绝对温度（C）
 * lastuv 保存最近一次 uV 结果，供调试界面显示，不参与控制。
 */
volatile uint32_t lastuv = 0;
/**
 * @brief 原始 ADC 码 → 运放输入端的电压（uV）
 * @param rawADC        ADC 原始值（0..ADC_MAX_READING，12 位 ×8 过采样）
 * @param skipCalOffset true = 不扣校准偏置（校准时用来读"未修正"的原始偏置）
 * @return 电压（uV）
 * @note  为什么要有校准偏置：热电偶两端同温时输出应为 0 uV，但运放失调、
 *        上拉电阻分压、烙铁头制造差异会带来一个固定偏移；CJC 校准就是把
 *        这个偏移量出来存进 CalibrationOffset。减法做了"不能为负"的保护。
 */
uint32_t          TipThermoModel::convertTipRawADCTouV(uint16_t rawADC, bool skipCalOffset) {
  // This takes the raw ADC samples, converts these to uV
  // Then divides this down by the gain to convert to the uV on the input to the op-amp (A+B terminals)
  // Then remove the calibration value that is stored as a tip offset
  // 先把基准电压放大 10 倍再运算，避免整数除法把 mV 精度截断；
  // 注释也说明：稳压器本身 ±2%，所以这里纠结更高精度没有意义。
  uint32_t vddRailmVX10 = ADC_VDD_MV * 10; // The vreg is +-2%, but we have no higher accuracy available
  // 4096 * 8 readings for full scale
  // Convert the input ADC reading back into mV times 10 format.
  // 线性映射：raw/满量程 × Vref。ADC_MAX_READING = 4096×8 = 32768，
  // 对应硬件 12 位 ADC + 8 倍过采样（BSP 的注入组连续采 8 次求和）。
  uint32_t rawInputmVX10 = (rawADC * vddRailmVX10) / (ADC_MAX_READING);

  uint32_t valueuV = rawInputmVX10 * 100; // shift into uV
  // Now to divide this down by the gain
  // 运放是同相放大：Gain = 1 + Rf/Rin。Pinecil 上 Rf=750k、Rin=2.37k → 约 317 倍。
  // 所以 ADC 看到的 mV 要除以 317 才是热电偶实际输出的 uV 量级。
  valueuV /= OP_AMP_GAIN_STAGE;

  if (getSettingValue(SettingsOptions::CalibrationOffset) && skipCalOffset == false) {
    // Remove uV tipOffset
    if (valueuV > getSettingValue(SettingsOptions::CalibrationOffset)) {
      valueuV -= getSettingValue(SettingsOptions::CalibrationOffset);
    } else {
      valueuV = 0;
    }
  }
  lastuv = valueuV;
  return valueuV;
}

// 三个便捷重载：ADC→C、ADC→F、uV→F。它们只是把上面的链路串起来，
// 真正查表插值在 convertuVToDegC（板级 ThermoModel.cpp 提供）。
// 注意 F 的换算始终"先转 C 再转 F"，避免两条独立的标定曲线。
TemperatureType_t TipThermoModel::convertTipRawADCToDegC(uint16_t rawADC) { return convertuVToDegC(convertTipRawADCTouV(rawADC)); }
TemperatureType_t TipThermoModel::convertTipRawADCToDegF(uint16_t rawADC) { return convertuVToDegF(convertTipRawADCTouV(rawADC)); }

TemperatureType_t TipThermoModel::convertuVToDegF(uint32_t tipuVDelta) { return convertCtoF(convertuVToDegC(tipuVDelta)); }

/**
 * @brief 摄氏 → 华氏（整数运算）
 * @note  公式 (C×9/5)+32；先乘后除保证精度。设置项里存的是用户单位，
 *        PID 内部一律用 C，所以进出边界都要过这一对函数。
 */
TemperatureType_t TipThermoModel::convertCtoF(TemperatureType_t degC) {
  //(Y °C × 9/5) + 32 =Y°F
  return (32 + ((degC * 9) / 5));
}

// 华氏 → 摄氏；低于 32 F 直接归 0，避免负数温度进入 PID（会算出负功率）。
// 这也是"拔掉烙铁头时读数接近 0"不会被误判为有效低温的原因之一。
TemperatureType_t TipThermoModel::convertFtoC(TemperatureType_t degF) {
  //(Y°F − 32) × 5/9 = Y°C
  if (degF < 32) {
    return 0;
  }
  return ((degF - 32) * 5) / 9;
}
/**
 * @brief 取当前烙铁头温度（C），已叠加冷端补偿
 * @param sampleNow true = 强制重新采样，false = 用最近一次 ADC 结果
 * @return 温度（C），负值被夹到 0
 * @note  冷端补偿：热电偶输出只反映"热端-冷端"温差，手柄 NTC 测的是冷端，
 *        所以必须加上 getHandleTemperature()/10（该函数返回 0.1 C 单位）。
 *        这里也是 PID 每周期调用的入口，内部会触发一次 ADC 读取 + 滤波。
 */
TemperatureType_t TipThermoModel::getTipInC(bool sampleNow) {
  TemperatureType_t currentTipTempInC = TipThermoModel::convertTipRawADCToDegC(getTipRawTemp(sampleNow));
  currentTipTempInC += getHandleTemperature(sampleNow) / 10; // Add handle offset

  if (currentTipTempInC < 0) {
    return 0;
  }
  return currentTipTempInC;
}

TemperatureType_t TipThermoModel::getTipInF(bool sampleNow) {
  TemperatureType_t currentTipTempInF = getTipInC(sampleNow);
  currentTipTempInF                   = convertCtoF(currentTipTempInF);
  return currentTipTempInF;
}

/**
 * @brief 当前硬件/校准条件下"能量到"的最高温度
 * @return 温度（C），已含冷端补偿
 * @note  用 ADC 满量程减 1 反算，再减 1 C 留余量。它的用途：
 *        ① 限制设定点（PIDThread 里夹 PIDTempTarget）；
 *        ② 判断烙铁头是否拔出（读数贴近满量程 → isTipDisconnected）。
 *        CUSTOM_MAX_TEMP_C 的机型直接返回自定义上限（例如高温头）。
 */
TemperatureType_t TipThermoModel::getTipMaxInC() {
#ifdef CUSTOM_MAX_TEMP_C
  return getCustomTipMaxInC();
#else
  TemperatureType_t maximumTipTemp = TipThermoModel::convertTipRawADCToDegC(ADC_MAX_READING - 1);
  maximumTipTemp += getHandleTemperature(0) / 10; // Add handle offset
  return maximumTipTemp - 1;
#endif
}
