
#include "Buttons.hpp"
#include "OperatingModes.h"
#include "ui_drawing.hpp"
/*
 * ============================================================================
 * 【文件导读】HomeScreen.cpp —— 待机主界面与入口分发
 * ----------------------------------------------------------------------------
 *   drawHomeScreen()    每帧把目标温度强制为 0（确保不加热），读取输入电压与
 *                       烙铁温度，按 DetailedIDLE 选择详细/简洁布局
 *   handleHomeButtons() 按键分发：
 *                         前短按 → Soldering（烙铁头未拔出时）
 *                         前长按 → SolderingProfile（或 TemperatureAdjust）
 *                         后短按 → SettingsMenu
 *                         后长按 → DebugMenuReadout
 * state1 用作"首次进入忽略按键"的闩锁：从别的模式切回来时，用户可能还按着
 * 键，先吃掉这一帧的按键，避免误入菜单。
 * ============================================================================
 */

bool showExitMenuTransition = false;

OperatingMode handleHomeButtons(const ButtonState buttons, guiContext *cxt) {
  if (buttons != BUTTON_NONE && cxt->scratch_state.state1 == 0) {
    return OperatingMode::HomeScreen; // Ignore button press
  } else {
    cxt->scratch_state.state1 = 1;
  }
  switch (buttons) {
  case BUTTON_NONE:
    // Do nothing
    break;
  case BUTTON_BOTH:
    break;

  case BUTTON_B_LONG:
    cxt->transitionMode = TransitionAnimation::Down;
    return OperatingMode::DebugMenuReadout;
    break;
  case BUTTON_F_LONG:
#ifdef PROFILE_SUPPORT
    if (!isTipDisconnected()) {
      cxt->transitionMode = TransitionAnimation::Left;
      return OperatingMode::SolderingProfile;
    } else {
      return OperatingMode::HomeScreen;
    }
#else
    cxt->transitionMode = TransitionAnimation::Left;
    return OperatingMode::TemperatureAdjust;
#endif
    break;
  case BUTTON_F_SHORT:
    if (!isTipDisconnected()) {
      bool detailedView   = getSettingValue(SettingsOptions::DetailedIDLE) && getSettingValue(SettingsOptions::DetailedSoldering);
      cxt->transitionMode = detailedView ? TransitionAnimation::None : TransitionAnimation::Left;
      return OperatingMode::Soldering;
    }
    break;
  case BUTTON_B_SHORT:
    cxt->transitionMode = TransitionAnimation::Right;
    return OperatingMode::SettingsMenu;
    break;
  default:
    break;
  }
  return OperatingMode::HomeScreen;
}

OperatingMode drawHomeScreen(const ButtonState buttons, guiContext *cxt) {

  currentTempTargetDegC = 0; // ensure tip is off
  getInputVoltageX10(getSettingValue(SettingsOptions::VoltageDiv), 0);
  uint32_t tipTemp = TipThermoModel::getTipInC();

  // Setup LCD Cursor location
  if (OLED::getRotation()) {
    OLED::setCursor(50, 0);
  } else {
    OLED::setCursor(-1, 0);
  }
  if (getSettingValue(SettingsOptions::DetailedIDLE)) {
    ui_draw_homescreen_detailed(tipTemp);
  } else {
    ui_draw_homescreen_simplified(tipTemp);
  }
  return handleHomeButtons(buttons, cxt);
}
