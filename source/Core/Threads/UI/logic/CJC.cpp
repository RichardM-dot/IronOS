#include "OperatingModes.h"
#include "ui_drawing.hpp"
/*
 * ============================================================================
 * 【文件导读】CJC.cpp —— 冷端补偿（Cold Junction Compensation）校准
 * ----------------------------------------------------------------------------
 * 原理：热电偶两端同温时输出应为 0 uV。上电初期烙铁头与手柄温度接近，
 *       此时连续采 16 次原始 ADC，取平均后换算成 uV，就是本机+本烙铁头的
 *       固定偏置，写入 SettingsOptions::CalibrationOffset。
 * 流程：等 preStartChecks 完成 → 确认烙铁头已接且温差 <10 C → 每 100 ms 采
 *       一次共 16 次 → 求平均并写设置 → 提示完成 → 清除 CalibrateCJC 标志。
 * 条件不满足（没插烙铁头/还没凉下来）则跳到 StartupWarnings，避免写入垃圾值。
 * ============================================================================
 */

OperatingMode performCJCC(const ButtonState buttons, guiContext *cxt) {
  // Calibrate Cold Junction Compensation directly at boot, before internal components get warm.

  // While we wait for the pre-start checks to finish, we cant run CJC (as the pre-start checks control the tip)
  if (preStartChecks() == 0) {
    OLED::setCursor(0, 0);
    OLED::print(translatedString(Tr->CJCCalibrating), FontStyle::SMALL);
    return OperatingMode::CJCCalibration;
  }

  if (!isTipDisconnected() && abs(int(TipThermoModel::getTipInC() - getHandleTemperature(0) / 10)) < 10) {
    // Take 16 samples, only sample
    if (cxt->scratch_state.state1 < 16) {
      if ((xTaskGetTickCount() - cxt->scratch_state.state4) > TICKS_100MS) {
        cxt->scratch_state.state3 += getTipRawTemp(1);
        cxt->scratch_state.state1++;
        cxt->scratch_state.state4 = xTaskGetTickCount();
      }
      ui_draw_cjc_sampling(cxt->scratch_state.state1 / 4);
      return OperatingMode::CJCCalibration;
    }

    // If the thermo-couple at the end of the tip, and the handle are at
    // equilibrium, then the output should be zero, as there is no temperature
    // differential.

    uint16_t setOffset = TipThermoModel::convertTipRawADCTouV(cxt->scratch_state.state3 / 16, true);
    setSettingValue(SettingsOptions::CalibrationOffset, setOffset);
    if (warnUser(translatedString(Tr->CalibrationDone), buttons)) {
      // Preventing to repeat calibration at boot automatically (only one shot).
      setSettingValue(SettingsOptions::CalibrateCJC, 0);
      saveSettings();
      return OperatingMode::InitialisationDone;
    }
    return OperatingMode::CJCCalibration;
  }
  // Cant run calibration without the tip and for temps to be close
  return OperatingMode::StartupWarnings;
}
