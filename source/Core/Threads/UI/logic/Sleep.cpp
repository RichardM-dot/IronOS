#include "OperatingModes.h"
#include "ui_drawing.hpp"
/*
 * ============================================================================
 * 【文件导读】Sleep.cpp —— 睡眠 / 休眠模式
 * ----------------------------------------------------------------------------
 * 两种状态共用本函数，用 cxt->scratch_state.state4 区分：
 *   Sleeping     目标温度 = min(SleepTemp, SolderingTemp)，仍保持低温待命
 *   Hibernating  目标温度 = 0，完全断电，只能靠按键/运动唤醒
 * 每帧检查 shouldBeSleeping()（运动+霍尔超时）与 shouldShutdown()（关机超时），
 * 任一满足就切回 HomeScreen（关机）或回到 previousMode（被唤醒）。
 * 注意：NO_SLEEP_MODE 机型直接返回 Soldering，本文件其余代码不参与编译。
 * ============================================================================
 */
OperatingMode gui_SolderingSleepingMode(const ButtonState buttons, guiContext *cxt) {
#ifdef NO_SLEEP_MODE
  return OperatingMode::Soldering;
#endif
  // Drop to sleep temperature and display until movement or button press

  // user moved or pressed a button, go back to soldering
  // If in the first two seconds we disable this to let accelerometer warm up

#ifdef POW_DC
  if (checkForUnderVoltage()) {
    return OperatingMode::HomeScreen; // return non-zero on error
  }
#endif

  if (cxt->scratch_state.state4) {
    // Hibernating mode
    currentTempTargetDegC = 0;
  } else {
    if (getSettingValue(SettingsOptions::TemperatureInF)) {
      currentTempTargetDegC = TipThermoModel::convertFtoC(min(getSettingValue(SettingsOptions::SleepTemp), getSettingValue(SettingsOptions::SolderingTemp)));
    } else {
      currentTempTargetDegC = min(getSettingValue(SettingsOptions::SleepTemp), getSettingValue(SettingsOptions::SolderingTemp));
    }
  }
  // draw the lcd
  uint16_t tipTemp = getSettingValue(SettingsOptions::TemperatureInF) ? TipThermoModel::getTipInF() : TipThermoModel::getTipInC();

  if (getSettingValue(SettingsOptions::DetailedSoldering)) {
    ui_draw_soldering_detailed_sleep(tipTemp);
  } else {
    ui_draw_soldering_basic_sleep(tipTemp);
  }

  if (!shouldBeSleeping()) {
    return cxt->previousMode;
  }

  if (shouldShutdown()) {
    // shutdown
    currentTempTargetDegC = 0;
    return OperatingMode::HomeScreen;
  }
  if (cxt->scratch_state.state4) {
    return OperatingMode::Hibernating;
  } else {
    return OperatingMode::Sleeping;
  }
}
