#include "FS2711.hpp"
#include "HUB238.hpp"
#include "OperatingModes.h"
#include "ui_drawing.hpp"
/*
 * ============================================================================
 * 【文件导读】ShowStartupWarnings.cpp —— 上电告警向导
 * ----------------------------------------------------------------------------
 * 按 state1 顺序推进的四道检查，每道都要用户按键确认：
 *   0 设置被重置（settingsWereReset）→ 提示用户重新配置
 *   1 器件校验失败（DEVICE_HAS_VALIDATION_SUPPORT）→ 可能非正品
 *   2 加速度计缺失 → 每次上电最多提示 2 次（计数器存 flash）
 *   3 PD 芯片缺失（FUSB302/HUB238/FS2711 按机型分支）→ 同样最多 2 次
 * 走完返回 StartupLogo。计数器的意义：老化的机器每次上电都弹窗会很烦，
 * 但完全不提示又会让"休眠失灵"难以排查，两次是折中。
 * ============================================================================
 */
OperatingMode showWarnings(const ButtonState buttons, guiContext *cxt) {
  // Display alert if settings were reset

  switch (cxt->scratch_state.state1) {
  case 0: // Settings reset warning
    if (settingsWereReset) {
      if (warnUser(translatedString(Tr->SettingsResetMessage), buttons)) {
        settingsWereReset         = false;
        cxt->scratch_state.state1 = 1;
      }
    } else {
      cxt->scratch_state.state1 = 1;
    }
    break;
  case 1: // Device validations
#ifdef DEVICE_HAS_VALIDATION_SUPPORT
    if (getDeviceValidationStatus()) {
      // Warn user this device might be counterfeit
      if (warnUser(translatedString(Tr->DeviceFailedValidationWarning), buttons)) {
        cxt->scratch_state.state1 = 2;
      }
    } else {
      cxt->scratch_state.state1 = 2;
    }
#else
    cxt->scratch_state.state1 = 2;
#endif
    break;
  case 2: // Accelerometer detection
#ifdef NO_ACCEL
    cxt->scratch_state.state1 = 3;
#else
    if (DetectedAccelerometerVersion == AccelType::Scanning) {
      break;
    }
    // Display alert if accelerometer is not detected
    if (DetectedAccelerometerVersion == AccelType::None) {
      if (getSettingValue(SettingsOptions::AccelMissingWarningCounter) < 2) {

        if (warnUser(translatedString(Tr->NoAccelerometerMessage), buttons)) {
          cxt->scratch_state.state1 = 3;
          nextSettingValue(SettingsOptions::AccelMissingWarningCounter);
          saveSettings();
        }
      } else {
        cxt->scratch_state.state1 = 3;
      }
    } else {
      cxt->scratch_state.state1 = 3;
    }
#endif

    break;
  case 3:

#ifdef POW_PD
    // We expect pd to be present
    if (!USBPowerDelivery::fusbPresent()) {
      if (getSettingValue(SettingsOptions::PDMissingWarningCounter) < 2) {
        if (warnUser(translatedString(Tr->NoPowerDeliveryMessage), buttons)) {
          nextSettingValue(SettingsOptions::PDMissingWarningCounter);
          saveSettings();
          cxt->scratch_state.state1 = 4;
        }
      } else {
        cxt->scratch_state.state1 = 4;
      }
    } else {
      cxt->scratch_state.state1 = 4;
    }
#else
#if POW_PD_EXT == 1
    if (!hub238_probe()) {
      if (getSettingValue(SettingsOptions::PDMissingWarningCounter) < 2) {
        if (warnUser(translatedString(Tr->NoPowerDeliveryMessage), buttons)) {
          cxt->scratch_state.state1 = 4;
          nextSettingValue(SettingsOptions::PDMissingWarningCounter);
          saveSettings();
        }
      } else {
        cxt->scratch_state.state1 = 4;
      }
    } else {
      cxt->scratch_state.state1 = 4;
    }
#else
#if POW_PD_EXT == 2
    if (!FS2711::probe()) {
      if (getSettingValue(SettingsOptions::PDMissingWarningCounter) < 2) {
        if (warnUser(translatedString(Tr->NoPowerDeliveryMessage), buttons)) {
          cxt->scratch_state.state1 = 4;
          nextSettingValue(SettingsOptions::PDMissingWarningCounter);
          saveSettings();
        }
      } else {
        cxt->scratch_state.state1 = 4;
      }
    } else {
      cxt->scratch_state.state1 = 4;
    }
#else
    cxt->scratch_state.state1 = 4;
#endif /*POW_PD_EXT==1*/
#endif /*POW_PD_EXT==2*/
#endif /*POW_PD*/

    break;
  default:
    // We are off the end, warnings done
    return OperatingMode::StartupLogo;
  }

  return OperatingMode::StartupWarnings; // Stay in warnings
}
