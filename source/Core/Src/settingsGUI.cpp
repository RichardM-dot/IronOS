/*
 * 【中文导读】settingsGUI.cpp —— 设置菜单实现（B级导读）
 * 每个设置项的 UI 生成与取值绑定（数据驱动的菜单生成器）。
 */

/*
 * settingsGUI.cpp
 *
 *  Created on: 3Sep.,2017
 *      Author: Ben V. Brown
 */

#include "settingsGUI.hpp"
#include "Buttons.hpp"
#include "Font.h"
#include "ScrollMessage.hpp"
#include "TipThermoModel.h"
#include "Translation.h"
#include "cmsis_os.h"
#include "configuration.h"
#include "main.hpp"
#include "ui_drawing.hpp"
/*
 * settingsGUI.cpp —— 设置菜单的"数据驱动"实现
 * ---------------------------------------------------------------------------
 * 核心思想：菜单不是一堆 if/else，而是一张 const menuitem 表。
 *   menuitem 五个字段 = 描述索引、增量回调、绘制回调、可见性回调、自动绑定设置项。
 *   - incrementHandler 非空：按"+"时调用它（用于需要自定义步进的项，如温度单位换算）
 *   - 为空且 autoSettingOption < SettingsOptionsLength：自动调 nextSettingValue()
 *   - draw 回调为 nullptr 是"菜单结束"标记（表尾必须放一个）
 *   - isVisible 为空 = 总是可见；非空则按条件隐藏（例如只有开了 Pulse 才显示其子项）
 * 目录结构：
 *   rootSettingsMenu → {Power, Soldering, PowerSaving, UI, Advanced, Language, End}
 *   subSettingsMenus[] 按 root 的顺序索引到各子表
 * 每个绘制函数只读设置并画到当前 OLED 光标处，不改状态；改状态全部集中在
 * 增量回调/确认回调里，这样"画"可以被反复调用（每帧）而不会产生副作用。
 * 单位约定：温度设置项统一以"用户单位"（C 或 F）存储，绘制时直接显示；
 * 需要参与计算的模块（PID/Sleep）再用 convertFtoC 转回来。
 */

#ifdef POW_DC
static void displayInputVRange(void);
static bool showInputVOptions(void);
static void displayInputMinVRange(void);
#endif /* POW_DC */

#ifdef POW_QC
static void displayQCInputV(void);
#endif /* POW_QC */

#if defined(POW_PD) || POW_PD_EXT == 2
static void displayPDNegTimeout(void);
static void displayUSBPDMode(void);
#endif /* POW_PD */

static void displaySensitivity(void);
static void displayShutdownTime(void);
static bool showSleepOptions(void);

#ifndef NO_SLEEP_MODE
static void setSleepTemp(void);
static void displaySleepTemp(void);
static void displaySleepTime(void);
#endif /* *not* NO_SLEEP_MODE */

static void setTempF(void);
static void displayTempF(void);
static void displayAdvancedSolderingScreens(void);
static void displayAdvancedIDLEScreens(void);
static void displayScrollSpeed(void);
static void displayReverseButtonTempChangeEnabled(void);
static void displayReverseButtonSettings(void);
static void displayPowerLimit(void);
#ifdef BUZZER_SETTING
static void displayBuzzerEnabled(void);
#endif /* BUZZER_SETTING */

#ifdef BLE_ENABLED
static void displayBluetoothLE(void);
#endif /* BLE_ENABLED */

#ifndef NO_DISPLAY_ROTATE
static void setDisplayRotation(void);
static void displayDisplayRotation(void);
#endif /* *not* NO_DISPLAY_ROTATE */

static void setBoostTemp(void);
static void displayBoostTemp(void);

#ifdef PROFILE_SUPPORT
static void setProfilePreheatTemp();
static void setProfilePhase1Temp();
static void setProfilePhase2Temp();
static void setProfilePhase3Temp();
static void setProfilePhase4Temp();
static void setProfilePhase5Temp();
static void displayProfilePhases(void);
static void displayProfilePreheatTemp(void);
static void displayProfilePreheatSpeed(void);
static void displayProfilePhase1Temp(void);
static void displayProfilePhase1Duration(void);
static void displayProfilePhase2Temp(void);
static void displayProfilePhase2Duration(void);
static void displayProfilePhase3Temp(void);
static void displayProfilePhase3Duration(void);
static void displayProfilePhase4Temp(void);
static void displayProfilePhase4Duration(void);
static void displayProfilePhase5Temp(void);
static void displayProfilePhase5Duration(void);
static void displayProfileCooldownSpeed(void);
static bool showProfileOptions(void);
static bool showProfilePhase2Options(void);
static bool showProfilePhase3Options(void);
static bool showProfilePhase4Options(void);
static bool showProfilePhase5Options(void);
#endif /* PROFILE_SUPPORT */

static void displayAutomaticStartMode(void);
static void displayLockingMode(void);
static void displayCoolingBlinkEnabled(void);
static void setResetSettings(void);
static void setCalibrate(void);
static void displayCalibrate(void);

static void setCalibrateVIN(void);
static void displayTempChangeShortStep(void);
static void displayTempChangeLongStep(void);
static void displayPowerPulse(void);
static bool displayAnimationOptions(void);
static void displayAnimationSpeed(void);
static void displayAnimationLoop(void);
static void displayPowerPulseWait(void);
static bool showPowerPulseOptions(void);
static void displayPowerPulseDuration(void);
static void displayBrightnessLevel(void);
static void displayInvertColor(void);
static void displayLogoTime(void);

#ifdef HALL_SENSOR
static void displayHallEffect(void);
static void displayHallEffectSleepTime(void);
static bool showHallEffect(void);
#endif /* HALL_SENSOR */

// Tip type selection
#ifdef TIP_TYPE_SUPPORT
static void displaySolderingTipType(void);
static bool showSolderingTipType(void);
#endif /* TIP_TYPE_SUPPORT */

// Menu values: on 128x32 panels the 8x16 small font keeps them next to the two-line names,
// on 96x16 panels the 12x16 large font is used as before.
#ifdef OLED_128x32
#define VALUE_FONT        FontStyle::SMALL
#define VALUE_SYMBOL(sym) SmallSymbol##sym
#define VALUE_Y           8
#else
#define VALUE_FONT        FontStyle::LARGE
#define VALUE_SYMBOL(sym) LargeSymbol##sym
#define VALUE_Y           0
#endif
// Translated value strings: ones encoded for the large font start with \x01 (CJK, 96x16)
/**
 * @brief 打印一个"值字符串"，自动处理大字体标记
 * @param str 翻译字符串；以 \x01 开头表示该串只能用 12x16 大字（CJK）
 * @note  96x16 面板用大字体显示值，128x32 面板用 8x16 小字体，二者宽度不同，
 *        所以字体选择被收敛到这一个函数里，避免每个 display 函数各写一份 #ifdef。
 */
static void printValue(const char *str) {
  if (str[0] == '\x01') {
    OLED::print(str + 1, FontStyle::LARGE);
  } else {
    OLED::print(str, VALUE_FONT);
  }
}

// Menu functions

#if defined(POW_DC) || defined(POW_QC) || defined(POW_PD) || POW_PD_EXT == 2
static void displayPowerMenu(void);
#endif /* POW_DC or POW_QC or POW_PD or POD_PD_EXT 2*/

static void displaySolderingMenu(void);
static void displayPowerSavingMenu(void);
static void displayUIMenu(void);
static void displayAdvancedMenu(void);

/*
 * Root Settings Menu
 *
 * Power Menu
 *  Power Source
 *  -Minimum Voltage
 *  QC Voltage
 *  PD Timeout
 *  USBPDMode
 *
 * Soldering
 *  Tip Type selection
 *  Boost Mode Temp
 *  Auto Start
 *  Temp Change Short Step
 *  Temp Change Long Step
 *  Locking Mode
 *  Profile Phases
 *  Profile Preheat Temperature
 *  Profile Preheat Max Temperature Change Per Second
 *  Profile Phase 1 Temperature
 *  Profile Phase 1 Duration (s)
 *  Profile Phase 2 Temperature
 *  Profile Phase 2 Duration (s)
 *  Profile Phase 3 Temperature
 *  Profile Phase 3 Duration (s)
 *  Profile Phase 4 Temperature
 *  Profile Phase 4 Duration (s)
 *  Profile Phase 5 Temperature
 *  Profile Phase 5 Duration (s)
 *  Profile Cooldown Max Temperature Change Per Second
 *
 * Power Saving
 *  Motion Sensitivity
 *  -Sleep Temp
 *  -Sleep Time
 *  -Shutdown Time
 *  Hall Sensor Sensitivity
 *  Hall Sensor Sleep Time
 *
 * UI
 *  Temperature Unit
 *  Display Orientation
 *  Cooldown Blink
 *  Scrolling Speed
 *  Swap Temp Change Buttons +/-
 *  Animation Speed
 *  -Animation Loop
 *  OLED Brightness
 *  Invert Screen
 *  Logo Timeout
 *  Detailed IDLE
 *  Detailed Soldering
 *
 * Advanced
 *  BluetoothLE
 *  Power Limit
 *  Calibrate CJC At Next Boot
 *  Calibrate Input V
 *  Power Pulse
 *  -Power Pulse Delay
 *  -Power Pulse Duration
 *  Factory Reset
 *
 */

// 空绘制函数：给"只需要动作、不需要显示值"的菜单项（如恢复出厂、电压校准）
// 占位，因为 menuitem.draw 不能为 nullptr（那是结束标记）。
void noOpDisplay() {}
/* vvv !!!DISABLE CLANG-FORMAT for menuitems initialization!!! vvv */

/* clang-format off */

/* A lot of suggestions by clang-format can be useful
 * but not when dealing with such menuitems declarations.
 */

/*
 * 菜单表的三条硬规则（改表前必读）：
 * 1) 每张表必须以"draw == nullptr"的哨兵项结尾，遍历靠它判断结束；
 * 2) 顺序必须与 subSettingsMenus[] 的索引一致（displayMenu(0..4) 就是下标）；
 * 3) 第 6 个字段是"值区域字符列宽"，决定值从第几列开始画，太小会和简称重叠。
 * 条件编译（POW_DC/POW_QC/POW_PD/PROFILE_SUPPORT/...）会改变表的内容，
 * 所以 getMenuLength() 必须在运行时数可见项，不能编译期算长度。
 */
const menuitem rootSettingsMenu[] {
  /*
   * Power Menu
   * Soldering Menu
   * Power Saving Menu
   * UI Menu
   * Advanced Menu
   * // Language
   * Exit
   */
#if defined(POW_DC) || defined(POW_QC) || defined(POW_PD)
  /* Power */
  {0, nullptr, displayPowerMenu, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0},
#endif
  /* Soldering */
  {0, nullptr, displaySolderingMenu, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0},
  /* Sleep Options Menu */
  {0, nullptr, displayPowerSavingMenu, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0},
  /* UI Menu */
  {0, nullptr, displayUIMenu, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0},
  /* Advanced Menu */
  {0, nullptr, displayAdvancedMenu, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0},
  /* Language Switch */
  {0, settings_setLanguageSwitch, settings_displayLanguageSwitch, settings_showLanguageSwitch, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0},
  /* vvvv end of menu marker. DO NOT REMOVE vvvv */
  {0, nullptr, nullptr, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0}
  /* ^^^^ end of menu marker. DO NOT REMOVE ^^^^ */
};

#if defined(POW_DC) || defined(POW_QC) || defined(POW_PD) || POW_PD_EXT == 2
const menuitem powerMenu[] = {
  /*
   * Power Source
   * -Minimum Voltage
   * QC Voltage
   * PD Timeout
   * USBPDMode
   */
#ifdef POW_DC
  /* Voltage input */
  {SETTINGS_DESC(SettingsItemIndex::DCInCutoff), nullptr, displayInputVRange, nullptr, SettingsOptions::MinDCVoltageCells, SettingsItemIndex::DCInCutoff, 6},
  /* Minimum voltage input */
  {SETTINGS_DESC(SettingsItemIndex::MinVolCell), nullptr, displayInputMinVRange, showInputVOptions, SettingsOptions::MinVoltageCells, SettingsItemIndex::MinVolCell, 5},
#endif
#ifdef POW_QC
  /* Voltage input */
  {SETTINGS_DESC(SettingsItemIndex::QCMaxVoltage), nullptr, displayQCInputV, nullptr, SettingsOptions::QCIdealVoltage, SettingsItemIndex::QCMaxVoltage, 4},
#endif
#if defined(POW_PD) || POW_PD_EXT == 2
  /* PD timeout setup */
  {SETTINGS_DESC(SettingsItemIndex::PDNegTimeout), nullptr, displayPDNegTimeout, nullptr, SettingsOptions::PDNegTimeout, SettingsItemIndex::PDNegTimeout, 6},
  /* Toggle PPS & EPR */
  {SETTINGS_DESC(SettingsItemIndex::USBPDMode), nullptr, displayUSBPDMode, nullptr, SettingsOptions::USBPDMode, SettingsItemIndex::USBPDMode, 4},
#endif
  /* vvvv end of menu marker. DO NOT REMOVE vvvv */
  {0, nullptr, nullptr, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0}
  /* ^^^^ end of menu marker. DO NOT REMOVE ^^^^ */
};
#endif /* POW_DC or POW_QC or POW_PD */

const menuitem solderingMenu[] = {
  /*
   *  Boost Mode Temp
   *  Auto Start
   *  Temp Change Short Step
   *  Temp Change Long Step
   *  Locking Mode
   *  Tip Type
   *  Profile Phases
   *  Profile Preheat Temperature
   *  Profile Preheat Max Temperature Change Per Second
   *  Profile Phase 1 Temperature
   *  Profile Phase 1 Duration (s)
   *  Profile Phase 2 Temperature
   *  Profile Phase 2 Duration (s)
   *  Profile Phase 3 Temperature
   *  Profile Phase 3 Duration (s)
   *  Profile Phase 4 Temperature
   *  Profile Phase 4 Duration (s)
   *  Profile Phase 5 Temperature
   *  Profile Phase 5 Duration (s)
   *  Profile Cooldown Max Temperature Change Per Second
   */
  /* Boost Temp */
  {SETTINGS_DESC(SettingsItemIndex::BoostTemperature), setBoostTemp, displayBoostTemp, nullptr, SettingsOptions::BoostTemp, SettingsItemIndex::BoostTemperature, 5},
  /* Auto start */
  {SETTINGS_DESC(SettingsItemIndex::AutoStart), nullptr, displayAutomaticStartMode, nullptr, SettingsOptions::AutoStartMode, SettingsItemIndex::AutoStart, 7},
  /* Temp change short step */
  {SETTINGS_DESC(SettingsItemIndex::TempChangeShortStep), nullptr, displayTempChangeShortStep, nullptr, SettingsOptions::TempChangeShortStep, SettingsItemIndex::TempChangeShortStep, 6},
  /* Temp change long step */
  {SETTINGS_DESC(SettingsItemIndex::TempChangeLongStep), nullptr, displayTempChangeLongStep, nullptr, SettingsOptions::TempChangeLongStep, SettingsItemIndex::TempChangeLongStep, 6},
  /* Locking Mode */
  {SETTINGS_DESC(SettingsItemIndex::LockingMode), nullptr, displayLockingMode, nullptr, SettingsOptions::LockingMode, SettingsItemIndex::LockingMode, 7},
#ifdef TIP_TYPE_SUPPORT
  /* Tip Type */
  {SETTINGS_DESC(SettingsItemIndex::SolderingTipType), nullptr, displaySolderingTipType, showSolderingTipType, SettingsOptions::SolderingTipType, SettingsItemIndex::SolderingTipType, 5},
#endif /* TIP_TYPE_SUPPORT */
#ifdef PROFILE_SUPPORT
  /* Profile Phases */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhases), nullptr, displayProfilePhases, nullptr, SettingsOptions::ProfilePhases, SettingsItemIndex::ProfilePhases, 7},
  /* Profile Preheat Temp */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePreheatTemp), setProfilePreheatTemp, displayProfilePreheatTemp, showProfileOptions, SettingsOptions::ProfilePreheatTemp, SettingsItemIndex::ProfilePreheatTemp, 5},
  /* Profile Preheat Speed */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePreheatSpeed), nullptr, displayProfilePreheatSpeed, showProfileOptions, SettingsOptions::ProfilePreheatSpeed, SettingsItemIndex::ProfilePreheatSpeed, 5},
  /* Phase 1 Temp */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhase1Temp), setProfilePhase1Temp, displayProfilePhase1Temp, showProfileOptions, SettingsOptions::ProfilePhase1Temp, SettingsItemIndex::ProfilePhase1Temp, 5},
  /* Phase 1 Duration */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhase1Duration), nullptr, displayProfilePhase1Duration, showProfileOptions, SettingsOptions::ProfilePhase1Duration, SettingsItemIndex::ProfilePhase1Duration, 5},
  /* Phase 2 Temp */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhase1Temp), setProfilePhase2Temp, displayProfilePhase2Temp, showProfilePhase2Options, SettingsOptions::ProfilePhase1Temp, SettingsItemIndex::ProfilePhase2Temp, 5},
  /* Phase 2 Duration */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhase1Duration), nullptr, displayProfilePhase2Duration, showProfilePhase2Options, SettingsOptions::ProfilePhase2Duration, SettingsItemIndex::ProfilePhase2Duration, 5},
  /* Phase 3 Temp */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhase1Temp), setProfilePhase3Temp, displayProfilePhase3Temp, showProfilePhase3Options, SettingsOptions::ProfilePhase1Temp, SettingsItemIndex::ProfilePhase3Temp, 5},
  /* Phase 3 Duration */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhase1Duration), nullptr, displayProfilePhase3Duration, showProfilePhase3Options, SettingsOptions::ProfilePhase3Duration, SettingsItemIndex::ProfilePhase3Duration, 5},
  /* Phase 4 Temp */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhase1Temp), setProfilePhase4Temp, displayProfilePhase4Temp, showProfilePhase4Options, SettingsOptions::ProfilePhase1Temp, SettingsItemIndex::ProfilePhase4Temp, 5},
  /* Phase 4 Duration */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhase1Duration), nullptr, displayProfilePhase4Duration, showProfilePhase4Options, SettingsOptions::ProfilePhase4Duration, SettingsItemIndex::ProfilePhase4Duration, 5},
  /* Phase 5 Temp */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhase1Temp), setProfilePhase5Temp, displayProfilePhase5Temp, showProfilePhase5Options, SettingsOptions::ProfilePhase1Temp, SettingsItemIndex::ProfilePhase5Temp, 5},
  /* Phase 5 Duration */
  {SETTINGS_DESC(SettingsItemIndex::ProfilePhase1Duration), nullptr, displayProfilePhase5Duration, showProfilePhase5Options, SettingsOptions::ProfilePhase5Duration, SettingsItemIndex::ProfilePhase5Duration, 5},
  /* Profile Cooldown Speed */
  {SETTINGS_DESC(SettingsItemIndex::ProfileCooldownSpeed), nullptr, displayProfileCooldownSpeed, showProfileOptions, SettingsOptions::ProfileCooldownSpeed, SettingsItemIndex::ProfileCooldownSpeed, 5},
#endif /* PROFILE_SUPPORT */
  /* vvvv end of menu marker. DO NOT REMOVE vvvv */
  {0, nullptr, nullptr, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0}
  /* ^^^^ end of menu marker. DO NOT REMOVE ^^^^ */
};

const menuitem PowerSavingMenu[] = {
  /*
   *  Motion Sensitivity
   *  -Sleep Temp
   *  -Sleep Time
   *  -Shutdown Time
   *  Hall Sensor Sensitivity
   */
  /* Motion Sensitivity */
  {SETTINGS_DESC(SettingsItemIndex::MotionSensitivity), nullptr, displaySensitivity, nullptr, SettingsOptions::Sensitivity, SettingsItemIndex::MotionSensitivity, 7},
#ifndef NO_SLEEP_MODE
  /* Sleep Temp */
  {SETTINGS_DESC(SettingsItemIndex::SleepTemperature), setSleepTemp, displaySleepTemp, showSleepOptions, SettingsOptions::SleepTemp, SettingsItemIndex::SleepTemperature, 5},
  /* Sleep Time */
  {SETTINGS_DESC(SettingsItemIndex::SleepTimeout), nullptr, displaySleepTime, showSleepOptions, SettingsOptions::SleepTime, SettingsItemIndex::SleepTimeout, 5},
#endif /* *not* NO_SLEEP_MODE */
  /* Shutdown Time */
  {SETTINGS_DESC(SettingsItemIndex::ShutdownTimeout), nullptr, displayShutdownTime, showSleepOptions, SettingsOptions::ShutdownTime, SettingsItemIndex::ShutdownTimeout, 5},
#ifdef HALL_SENSOR
  /* Hall Effect Sensitivity */
  {SETTINGS_DESC(SettingsItemIndex::HallEffSensitivity), nullptr, displayHallEffect, showHallEffect, SettingsOptions::HallEffectSensitivity, SettingsItemIndex::HallEffSensitivity, 7},
  /* Hall Effect Sleep Time */
  {SETTINGS_DESC(SettingsItemIndex::HallEffSleepTimeout), nullptr, displayHallEffectSleepTime, showHallEffect, SettingsOptions::HallEffectSleepTime, SettingsItemIndex::HallEffSleepTimeout, 5},
#endif /* HALL_SENSOR */
  /* vvvv end of menu marker. DO NOT REMOVE vvvv */
  {0, nullptr, nullptr, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0}
  /* ^^^^ end of menu marker. DO NOT REMOVE ^^^^ */
};

const menuitem UIMenu[] = {
  /*
   *  Temperature Unit
   *  Display Orientation
   *  Cooldown Blink
   *  Scrolling Speed
   *  Swap Temp Change Buttons +/-
   *  Animation Speed
   *  -Animation Loop
   *  OLED Brightness
   *  Invert Screen
   *  Logo Timeout
   *  Detailed IDLE
   *  Detailed Soldering
   */
  /* Temperature units, this has to be the first element in the array to work with the logic in enterUIMenu() */
  {SETTINGS_DESC(SettingsItemIndex::TemperatureUnit), setTempF, displayTempF, nullptr, SettingsOptions::TemperatureInF, SettingsItemIndex::TemperatureUnit, 7},
#ifndef NO_DISPLAY_ROTATE
  /* Display Rotation */
  {SETTINGS_DESC(SettingsItemIndex::DisplayRotation), setDisplayRotation, displayDisplayRotation, nullptr, SettingsOptions::OrientationMode, SettingsItemIndex::DisplayRotation, 7},
#endif /* *not* NO_DISPLAY_ROTATE */
  /* Cooling blink warning */
  {SETTINGS_DESC(SettingsItemIndex::CooldownBlink), nullptr, displayCoolingBlinkEnabled, nullptr, SettingsOptions::CoolingTempBlink, SettingsItemIndex::CooldownBlink, 7},
#ifdef BUZZER_SETTING
  /* Buzzer on/off */
  {SETTINGS_DESC(SettingsItemIndex::BuzzerEnabled), nullptr, displayBuzzerEnabled, nullptr, SettingsOptions::BuzzerEnabled, SettingsItemIndex::BuzzerEnabled, 7},
#endif /* BUZZER_SETTING */
  /* Scroll Speed for descriptions */
  {SETTINGS_DESC(SettingsItemIndex::ScrollingSpeed), nullptr, displayScrollSpeed, nullptr, SettingsOptions::DescriptionScrollSpeed, SettingsItemIndex::ScrollingSpeed, 7},
  /* Reverse Temp change buttons +/- */
  {SETTINGS_DESC(SettingsItemIndex::ReverseButtonTempChange), nullptr, displayReverseButtonTempChangeEnabled, nullptr, SettingsOptions::ReverseButtonTempChangeEnabled, SettingsItemIndex::ReverseButtonTempChange, 7},
  /* Reverse Settings menu buttons A/B */
  {SETTINGS_DESC(SettingsItemIndex::ReverseButtonSettings), nullptr, displayReverseButtonSettings, nullptr, SettingsOptions::ReverseButtonSettings, SettingsItemIndex::ReverseButtonSettings, 7},
  /* Animation Speed adjustment */
  {SETTINGS_DESC(SettingsItemIndex::AnimSpeed), nullptr, displayAnimationSpeed, nullptr, SettingsOptions::AnimationSpeed, SettingsItemIndex::AnimSpeed, 7},
  /* Animation Loop switch */
  {SETTINGS_DESC(SettingsItemIndex::AnimLoop), nullptr, displayAnimationLoop, displayAnimationOptions, SettingsOptions::AnimationLoop, SettingsItemIndex::AnimLoop, 7},
  /* Brightness Level */
  {SETTINGS_DESC(SettingsItemIndex::Brightness), nullptr, displayBrightnessLevel, nullptr, SettingsOptions::OLEDBrightness, SettingsItemIndex::Brightness, 7},
  /* Invert screen colour */
  {SETTINGS_DESC(SettingsItemIndex::ColourInversion), nullptr, displayInvertColor, nullptr, SettingsOptions::OLEDInversion, SettingsItemIndex::ColourInversion, 7},
  /* Set logo duration */
  {SETTINGS_DESC(SettingsItemIndex::LOGOTime), nullptr, displayLogoTime, nullptr, SettingsOptions::LOGOTime, SettingsItemIndex::LOGOTime, 6},
  /* Advanced idle screen */
  {SETTINGS_DESC(SettingsItemIndex::AdvancedIdle), nullptr, displayAdvancedIDLEScreens, nullptr, SettingsOptions::DetailedIDLE, SettingsItemIndex::AdvancedIdle, 7},
  /* Advanced soldering screen */
  {SETTINGS_DESC(SettingsItemIndex::AdvancedSoldering), nullptr, displayAdvancedSolderingScreens, nullptr, SettingsOptions::DetailedSoldering, SettingsItemIndex::AdvancedSoldering, 7},
  /* vvvv end of menu marker. DO NOT REMOVE vvvv */
  {0, nullptr, nullptr, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0}
  /* ^^^^ end of menu marker. DO NOT REMOVE ^^^^ */
};

const menuitem advancedMenu[] = {
  /*
   *  BluetoothLE
   *  Power Limit
   *  Calibrate CJC At Next Boot
   *  Calibrate Input V
   *  Power Pulse
   *  -Power Pulse Delay
   *  -Power Pulse Duration
   *  Factory Reset
   */
#ifdef BLE_ENABLED
  /* Toggle BLE */
  {SETTINGS_DESC(SettingsItemIndex::BluetoothLE), setBluetoothLE, displayBluetoothLE, nullptr, SettingsOptions::BluetoothLE, SettingsItemIndex::BluetoothLE, 7},
#endif /* BLE_ENABLED */
  /* Power limit */
  {SETTINGS_DESC(SettingsItemIndex::PowerLimit), nullptr, displayPowerLimit, nullptr, SettingsOptions::PowerLimit, SettingsItemIndex::PowerLimit, 4},
  /* Calibrate Cold Junktion Compensation at next boot */
  {SETTINGS_DESC(SettingsItemIndex::CalibrateCJC), setCalibrate, displayCalibrate, nullptr, SettingsOptions::CalibrateCJC, SettingsItemIndex::CalibrateCJC, 7},
  /* Voltage input cal */
  {SETTINGS_DESC(SettingsItemIndex::VoltageCalibration), setCalibrateVIN, noOpDisplay, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::VoltageCalibration, 5},
  /* Power Pulse adjustment */
  {SETTINGS_DESC(SettingsItemIndex::PowerPulsePower), nullptr, displayPowerPulse, nullptr, SettingsOptions::KeepAwakePulse, SettingsItemIndex::PowerPulsePower, 5},
  /* Power Pulse Wait adjustment */
  {SETTINGS_DESC(SettingsItemIndex::PowerPulseWait), nullptr, displayPowerPulseWait, showPowerPulseOptions, SettingsOptions::KeepAwakePulseWait, SettingsItemIndex::PowerPulseWait, 7},
  /* Power Pulse Duration adjustment */
  {SETTINGS_DESC(SettingsItemIndex::PowerPulseDuration), nullptr, displayPowerPulseDuration, showPowerPulseOptions, SettingsOptions::KeepAwakePulseDuration, SettingsItemIndex::PowerPulseDuration, 7},
  /* Resets settings */
  {SETTINGS_DESC(SettingsItemIndex::SettingsReset), setResetSettings, noOpDisplay, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::SettingsReset, 7},
  /* vvvv end of menu marker. DO NOT REMOVE vvvv */
  {0, nullptr, nullptr, nullptr, SettingsOptions::SettingsOptionsLength, SettingsItemIndex::NUM_ITEMS, 0}
  /* ^^^^ end of menu marker. DO NOT REMOVE ^^^^ */
};

/* clang-format on */

const menuitem *subSettingsMenus[]{
#if defined(POW_DC) || defined(POW_QC) || defined(POW_PD) || POW_PD_EXT == 2
    powerMenu,
#endif
    solderingMenu, PowerSavingMenu, UIMenu, advancedMenu,
};
/* ^^^ !!!ENABLE CLANG-FORMAT back!!! ^^^ */

/**
 * Prints two small lines (or one line for CJK) of short description for
 * setting items and prepares cursor after it.
 * @param settingsItemIndex Index of the setting item.
 * @param cursorCharPosition Custom cursor char position to set after printing
 * description.
 */
/**
 * @brief 在屏幕左侧打印设置项简称，并把光标移到"值"的起始列
 * @param settingsItemIndex 设置项索引（对应翻译表的简称数组）
 * @param cursorCharPosition 值区域的字符列号（不同项宽度不同，来自菜单表的第 6 字段）
 * @note  -2 像素是留给右侧滚动条指示器的；FONT_12_WIDTH=12 是 96x16 面板的字符宽。
 */
static void printShortDescription(SettingsItemIndex settingsItemIndex, uint16_t cursorCharPosition) {
  // print short description (default single line, explicit double line)
  uint8_t shortDescIndex = static_cast<uint8_t>(settingsItemIndex);
  OLED::printWholeScreen(translatedString(Tr->SettingsShortNames[shortDescIndex]));

  // prepare cursor for value
  // make room for scroll indicator
  OLED::setCursor(cursorCharPosition * FONT_12_WIDTH - 2, 0);
}

/**
 * @brief 阻塞式确认框：滚动显示 message，等待用户按键
 * @param message 需要用户确认的提示（会滚动显示，可能很长）
 * @return 1 = 用户按前键确认；0 = 其它任何按键
 * @note  本函数内部自己跑 40 ms 的渲染循环，因此调用期间 UI 其它部分完全停摆；
 *        它被"恢复出厂设置""CJC 校准"这类不可逆操作使用，属于有意为之的模态框。
 */
static int userConfirmation(const char *message) {
  TickType_t tickStart = xTaskGetTickCount();
  for (;;) {
    // 用"进入本函数后的相对时间"驱动滚动，而不是全局时间：
    // 这样每次弹出确认框都从消息开头开始滚，不会出现"一打开就在中间"的割裂感。
    drawScrollingText(message, xTaskGetTickCount() - tickStart);

    ButtonState buttons = getButtonState();
    switch (buttons) {
    case BUTTON_F_SHORT:
      // User confirmed
      return 1;

    case BUTTON_NONE:
      break;
    default:
    case BUTTON_BOTH:
    case BUTTON_B_SHORT:
    case BUTTON_F_LONG:
    case BUTTON_B_LONG:
      return 0;
    }

    OLED::refresh();
    osDelay(40);
  }
  return 0;
}

#ifdef POW_DC

// 电源输入类型：0=DC 固定输入，>0 表示按 N+2 串电池估算截止电压。
// 显示成 "3S" 这类电芯数，用户不需要理解背后的电压值。
static void displayInputVRange(void) {
  if (getSettingValue(SettingsOptions::MinDCVoltageCells)) {
    OLED::printNumber(2 + getSettingValue(SettingsOptions::MinDCVoltageCells), 1, VALUE_FONT);
    OLED::print(VALUE_SYMBOL(CellCount), VALUE_FONT);
  } else {
    OLED::print(VALUE_SYMBOL(DC), VALUE_FONT);
  }
}

static bool showInputVOptions(void) { return getSettingValue(SettingsOptions::MinDCVoltageCells) > 0; }

// 每节电池的最低截止电压，设置值以 0.1 V 存储（如 27 = 2.7 V），
// 所以整数部分 /10、小数部分 %10 分开打印。
static void displayInputMinVRange(void) {
  OLED::printNumber(getSettingValue(SettingsOptions::MinVoltageCells) / 10, 1, VALUE_FONT);
  OLED::print(VALUE_SYMBOL(Dot), VALUE_FONT);
  OLED::printNumber(getSettingValue(SettingsOptions::MinVoltageCells) % 10, 1, VALUE_FONT);
}

#endif /* POW_DC */

#ifdef POW_QC

// QC 目标电压：设置值以 0.1 V 存储（90 = 9.0 V），同样拆成两段打印。
static void displayQCInputV(void) {
  // These are only used in QC modes
  // Allows setting the voltage negotiated for QC
  auto voltage = getSettingValue(SettingsOptions::QCIdealVoltage);
  OLED::printNumber(voltage / 10, 2, VALUE_FONT);
  OLED::print(VALUE_SYMBOL(Dot), VALUE_FONT);
  OLED::printNumber(voltage % 10, 1, VALUE_FONT);
}

#endif /* POW_QC */

#if defined(POW_PD) || POW_PD_EXT == 2 /* POW_PD */

// PD 协商超时（单位 100 ms）；0 表示不限时，用"不可用"图标表示。
static void displayPDNegTimeout(void) {
  auto value = getSettingValue(SettingsOptions::PDNegTimeout);
  value ? OLED::printNumber(value, 2, VALUE_FONT) : OLED::drawUnavailableIcon();
}

/**
 * @brief 显示 PD 模式（DEFAULT/SAFE/NO_DYNAMIC）
 * @note  128x32 面板上这些译文可能折成两行（如 Default\nMode），所以先把光标
 *        移到右侧 64 px 区域，让换行落在正确的两行 16 px 行高内；
 *        最后一个参数 OLED::getCursorX() 是"软换行边界"，防止文字画到屏幕外。
 */
static void displayUSBPDMode(void) {
  /*
   * Supported PD modes:
   *  DEFAULT,    1 = PPS + EPR + more power request through increasing resistance by 0.5 Ohm to compensate power loss over cable/PCB/etc.
   *  SAFE,       2 = PPS + EPR, without requesting more power
   *  NO_DYNAMIC, 0 = PPS + EPR disabled, fixed PDO only
   */

#ifdef OLED_128x32
  // Values wrap to two lines (e.g. "Default\nMode"); give them the right part of
  // the panel and let the newline fill both 16px rows instead of overflowing.
  OLED::setCursor(OLED_WIDTH - 64, 0);
#endif
  switch (getSettingValue(SettingsOptions::USBPDMode)) {
  case usbpdMode_t::DEFAULT:
    OLED::print(translatedString(Tr->USBPDModeDefault), FontStyle::SMALL, 255, OLED::getCursorX());
    break;
  case usbpdMode_t::SAFE:
    OLED::print(translatedString(Tr->USBPDModeSafe), FontStyle::SMALL, 255, OLED::getCursorX());
    break;
  case usbpdMode_t::NO_DYNAMIC:
  default:
    OLED::print(translatedString(Tr->USBPDModeNoDynamic), FontStyle::SMALL, 255, OLED::getCursorX());
    break;
  }
}

#endif /* POW_PD */

/**
 * @brief Boost 温度的"+"步进：0(关闭) → MIN_BOOST → ... → MAX → 0 循环
 * @note  C 模式每次 +10 C、F 模式每次 +20 F；越过上限回到 0（关闭）而不是
 *        停在最大值，这样用户可以用"一直按"快速关掉 Boost。
 */
static void setBoostTemp(void) {
  uint16_t value = getSettingValue(SettingsOptions::BoostTemp);
  if (getSettingValue(SettingsOptions::TemperatureInF)) {
    if (value == 0) {
      value = MIN_BOOST_TEMP_F; // loop back at 480
    } else {
      value += 20; // Go up 20F at a time
    }

    if (value >= MAX_TEMP_F) {
      value = 0; // jump to off
    }
  } else {
    if (value == 0) {
      value = MIN_BOOST_TEMP_C; // loop back at 250
    } else {
      value += 10; // Go up 10C at a time
    }
    if (value > MAX_TEMP_C) {
      value = 0; // Go to off state
    }
  }
  setSettingValue(SettingsOptions::BoostTemp, value);
}

// Boost 温度；0 表示 Boost 功能关闭，画"不可用"图标而不是数字 0。
static void displayBoostTemp(void) {
  if (getSettingValue(SettingsOptions::BoostTemp)) {
    OLED::printNumber(getSettingValue(SettingsOptions::BoostTemp), 3, VALUE_FONT);
  } else {
    OLED::drawUnavailableIcon();
  }
}

// 上电自动进入的模式：关闭 / 直接焊接 / 睡眠温度 / 仅上电不加热。
// 用图标+短译文字符表示，避免在 96px 宽的屏上放长句。
static void displayAutomaticStartMode(void) {
  switch (getSettingValue(SettingsOptions::AutoStartMode)) {
  case autoStartMode_t::NO:
    OLED::drawUnavailableIcon();
    break;
  case autoStartMode_t::SOLDER:
    printValue(translatedString(Tr->SettingStartSolderingChar));
    break;
  case autoStartMode_t::SLEEP:
    printValue(translatedString(Tr->SettingStartSleepChar));
    break;
  case autoStartMode_t::ZERO:
    printValue(translatedString(Tr->SettingStartSleepOffChar));
    break;
  default:
    OLED::drawUnavailableIcon();
    break;
  }
}

// 短按调温步进（C），两位数字显示。
// 长按步进同理，但用户通常设得更大（默认 10 C）。
static void displayTempChangeShortStep(void) { OLED::printNumber(getSettingValue(SettingsOptions::TempChangeShortStep), 2, VALUE_FONT); }

static void displayTempChangeLongStep(void) { OLED::printNumber(getSettingValue(SettingsOptions::TempChangeLongStep), 2, VALUE_FONT); }

// 按键锁模式：关闭 / 只锁 Boost / 全锁。全锁时连调温都要先解锁。
static void displayLockingMode(void) {
  switch (getSettingValue(SettingsOptions::LockingMode)) {
  case lockingMode_t::DISABLED:
    OLED::drawUnavailableIcon();
    break;
  case lockingMode_t::BOOST:
    printValue(translatedString(Tr->SettingLockBoostChar));
    break;
  case lockingMode_t::FULL:
    printValue(translatedString(Tr->SettingLockFullChar));
    break;
  default:
    OLED::drawUnavailableIcon();
    break;
  }
}

#ifdef PROFILE_SUPPORT

static void displayProfilePhases(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhases), 1, VALUE_FONT); }

/**
 * @brief Profile 各阶段温度的通用步进器
 * @param option 要修改的设置项
 * @note  C 每次 +5、F 每次 +10；超过 MAX 回到 MIN（而不是关闭），
 *        因为回流曲线的温度不能为 0。
 */
static void setProfileTemp(const enum SettingsOptions option) {
  // If in C, 5 deg, if in F 10 deg
  uint16_t temp = getSettingValue(option);
  if (getSettingValue(SettingsOptions::TemperatureInF)) {
    temp += 10;
    if (temp > MAX_TEMP_F) {
      temp = MIN_TEMP_F;
    }
  } else {
    temp += 5;
    if (temp > MAX_TEMP_C) {
      temp = MIN_TEMP_C;
    }
  }
  setSettingValue(option, temp);
}

// 六个阶段温度共用上面的步进器；这里只是把"设置项"绑定成具体函数指针，
// 以便菜单表能直接存放无参回调。
static void setProfilePreheatTemp(void) { return setProfileTemp(SettingsOptions::ProfilePreheatTemp); }
static void setProfilePhase1Temp(void) { return setProfileTemp(SettingsOptions::ProfilePhase1Temp); }
static void setProfilePhase2Temp(void) { return setProfileTemp(SettingsOptions::ProfilePhase2Temp); }
static void setProfilePhase3Temp(void) { return setProfileTemp(SettingsOptions::ProfilePhase3Temp); }
static void setProfilePhase4Temp(void) { return setProfileTemp(SettingsOptions::ProfilePhase4Temp); }
static void setProfilePhase5Temp(void) { return setProfileTemp(SettingsOptions::ProfilePhase5Temp); }

// Profile 温度/时长/速率的绘制：统一 3 位或 2 位数字，单位由翻译表里的
// 描述文字承担（例如 "Preheat Temp"），屏幕上不再重复印单位以省像素。
static void displayProfilePreheatTemp(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePreheatTemp), 3, VALUE_FONT); }
static void displayProfilePhase1Temp(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhase1Temp), 3, VALUE_FONT); }
static void displayProfilePhase2Temp(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhase2Temp), 3, VALUE_FONT); }
static void displayProfilePhase3Temp(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhase3Temp), 3, VALUE_FONT); }
static void displayProfilePhase4Temp(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhase4Temp), 3, VALUE_FONT); }
static void displayProfilePhase5Temp(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhase5Temp), 3, VALUE_FONT); }
static void displayProfilePreheatSpeed(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePreheatSpeed), 2, VALUE_FONT); }
static void displayProfileCooldownSpeed(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfileCooldownSpeed), 2, VALUE_FONT); }
static void displayProfilePhase1Duration(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhase1Duration), 3, VALUE_FONT); }
static void displayProfilePhase2Duration(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhase2Duration), 3, VALUE_FONT); }
static void displayProfilePhase3Duration(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhase3Duration), 3, VALUE_FONT); }
static void displayProfilePhase4Duration(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhase4Duration), 3, VALUE_FONT); }
static void displayProfilePhase5Duration(void) { OLED::printNumber(getSettingValue(SettingsOptions::ProfilePhase5Duration), 3, VALUE_FONT); }

// 可见性回调：只有启用了回流曲线（Phases>0），才显示预热/各阶段参数；
// 第 N 阶段的参数要求 Phases>=N，避免用户看到"不存在的阶段"。
static bool showProfileOptions(void) { return getSettingValue(SettingsOptions::ProfilePhases); }
static bool showProfilePhase2Options(void) { return getSettingValue(SettingsOptions::ProfilePhases) >= 2; }
static bool showProfilePhase3Options(void) { return getSettingValue(SettingsOptions::ProfilePhases) >= 3; }
static bool showProfilePhase4Options(void) { return getSettingValue(SettingsOptions::ProfilePhases) >= 4; }
static bool showProfilePhase5Options(void) { return getSettingValue(SettingsOptions::ProfilePhases) >= 5; }

#endif /* PROFILE_SUPPORT */

// 运动灵敏度 0..9；0 = 关闭（同时会隐藏睡眠相关项，见 showSleepOptions）。
static void displaySensitivity(void) {
  if (getSettingValue(SettingsOptions::Sensitivity)) {
    OLED::printNumber(getSettingValue(SettingsOptions::Sensitivity), 1, VALUE_FONT, false);
  } else {
    OLED::drawUnavailableIcon();
  }
}
// 没有运动感应就没有"多久没动进入睡眠"的概念，因此睡眠/关机项依赖灵敏度。
static bool showSleepOptions(void) { return getSettingValue(SettingsOptions::Sensitivity) > 0; }

#ifndef NO_SLEEP_MODE

/**
 * @brief 睡眠温度步进：C +10（上限 300 回到 10），F +20（上限 580 回到 60）
 * @note  下限 10 C 是"仍有一点点热"的防冷凝设定；上限刻意低于焊接温度，
 *        避免用户把睡眠温度设得比工作温度还高导致"睡不睡都一样"。
 */
static void setSleepTemp(void) {
  // If in C, 10 deg, if in F 20 deg
  uint16_t temp = getSettingValue(SettingsOptions::SleepTemp);
  if (getSettingValue(SettingsOptions::TemperatureInF)) {
    temp += 20;
    if (temp > 580) {
      temp = 60;
    }
  } else {
    temp += 10;
    if (temp > 300) {
      temp = 10;
    }
  }
  setSettingValue(SettingsOptions::SleepTemp, temp);
}

static void displaySleepTemp(void) { OLED::printNumber(getSettingValue(SettingsOptions::SleepTemp), 3, VALUE_FONT); }

/**
 * @brief 睡眠倒计时显示，编码规则与设置值一致
 * @note  设置值 0=关闭；1..5 表示 10..50 秒（值×10）；6 以上表示分钟（值-5）。
 *        这个"分段编码"让一个 uint16 同时覆盖秒级调试与分钟级日常使用，
 *        换算逻辑必须与 getSleepTimeout() 保持一致（见 UI utils）。
 */
static void displaySleepTime(void) {
  if (getSettingValue(SettingsOptions::SleepTime) == 0) {
    OLED::drawUnavailableIcon();
  } else if (getSettingValue(SettingsOptions::SleepTime) < 6) {
    OLED::printNumber(getSettingValue(SettingsOptions::SleepTime) * 10, 2, VALUE_FONT);
    OLED::print(VALUE_SYMBOL(Seconds), VALUE_FONT);
  } else {
    OLED::printNumber(getSettingValue(SettingsOptions::SleepTime) - 5, 2, VALUE_FONT);
    OLED::print(VALUE_SYMBOL(Minutes), VALUE_FONT);
  }
}

#endif /* *not* NO_SLEEP_MODE */

// 关机倒计时，单位分钟；0 = 不自动关机。
static void displayShutdownTime(void) {
  if (getSettingValue(SettingsOptions::ShutdownTime) == 0) {
    OLED::drawUnavailableIcon();
  } else {
    OLED::printNumber(getSettingValue(SettingsOptions::ShutdownTime), 2, VALUE_FONT);
    OLED::print(VALUE_SYMBOL(Minutes), VALUE_FONT);
  }
}

#ifdef HALL_SENSOR
// 霍尔传感器灵敏度 0..9（阈值表见 Settings.cpp lookupHallEffectThreshold，近似指数映射）。
// 0 = 关闭；此时睡眠仍可由运动感应触发。
static void displayHallEffect(void) {
  if (getSettingValue(SettingsOptions::HallEffectSensitivity)) {
    OLED::printNumber(getSettingValue(SettingsOptions::HallEffectSensitivity), 1, VALUE_FONT, false);
  } else {
    OLED::drawUnavailableIcon();
  }
}
// 是否显示霍尔相关项取决于硬件是否焊了传感器（运行时探测，不是编译期宏）。
static bool showHallEffect(void) { return getHallSensorFitted(); }
/**
 * @brief 霍尔触发的睡眠延时，单位秒（设置值 ×5）
 * @note  设置值为 0 时并不显示 0，而是显示 1 秒：代码里"0"表示使用默认 1 秒
 *        的强制睡眠（见 getHallEffectSleepTimeout），显示 1 才符合实际行为。
 */
static void displayHallEffectSleepTime(void) {
  if (getSettingValue(SettingsOptions::HallEffectSleepTime)) {
    OLED::printNumber(getSettingValue(SettingsOptions::HallEffectSleepTime) * 5, 2, VALUE_FONT, false);
  } else {
    // When sleep time is set to zero, we sleep for 1 second anyways. This is the default.
    OLED::printNumber(1, 2, VALUE_FONT, false);
  }
  OLED::print(VALUE_SYMBOL(Seconds), VALUE_FONT);
}
#endif /* HALL_SENSOR */

#ifdef TIP_TYPE_SUPPORT
#ifdef OLED_128x32
// Width in pixels of the widest line of a (possibly two-line) tip name. Tip names
// embed a newline (0x01), e.g. "Auto\nSense"; the small font is 8px per glyph.
/**
 * @brief 计算烙铁头名称（可能含换行）在屏幕上的像素宽度
 * @param name 翻译字符串，0x01 表示换行标记
 * @return 最宽一行的像素宽度
 * @note  名称里嵌了 '\n'(0x01)，但 OLED::drawChar 只在 y==0 时处理换行，
 *        所以要自己算出宽度来做右对齐，否则两行文字会叠在一起。
 *        编码规则：<=0xF0 是单字节字符；>0xF0 是双字节（CJK），占 2 字节 1 字符。
 */
static uint16_t tipNameBlockWidth(const char *name) {
  const uint8_t *p   = reinterpret_cast<const uint8_t *>(name);
  uint16_t       cur = 0, widest = 0;
  while (p[0]) {
    if (p[0] == 0x01) { // newline marker
      if (cur > widest) {
        widest = cur;
      }
      cur = 0;
      p++;
    } else if (p[0] <= 0xF0) {
      cur++;
      p++;
    } else {
      if (!p[1]) {
        break;
      }
      cur++;
      p += 2;
    }
  }
  if (cur > widest) {
    widest = cur;
  }
  // Cell width of the small font: Terminus 8x16 on 128x32 panels, 6x8 elsewhere
#ifdef OLED_128x32
  return widest * 8;
#else
  return widest * 6;
#endif
}
#endif
// 烙铁头型号：右侧对齐显示（128x32 上名称为两行，需按最宽行算偏移）。
static void displaySolderingTipType(void) {
#ifdef OLED_128x32
  // Tip names embed a newline; drawChar only wraps at y==0, so anchor a
  // right-aligned, two-line block at the top of the panel.
  const char *name = lookupTipName();
  OLED::setCursor(OLED_WIDTH - tipNameBlockWidth(name) - 2, 0);
  OLED::print(name, FontStyle::SMALL, 255, OLED::getCursorX());
#else
  // TODO wrapping X value
  OLED::print(lookupTipName(), FontStyle::SMALL, 255, OLED::getCursorX());
#endif
}
// If there is no detection, and no options, max is 0
// 编译期是否支持多型号烙铁头：不支持时枚举只有一个值，TIP_TYPE_MAX==0。
static bool showSolderingTipType(void) { return tipType_t::TIP_TYPE_MAX != 0; }
#endif /* TIP_TYPE_SUPPORT */

/**
 * @brief 切换温度单位时，把某个设置项的数值在 C/F 之间换算
 * @param option 要换算的设置项
 * @note  调用前 TemperatureInF 已经被 nextSettingValue 翻转，所以这里用"新单位"
 *        判断方向：新单位是 F → 把旧值当 C 转 F；反之当 F 转 C。
 *        换算后向下取整到 10 的倍数，保证后续 +/- 步进仍然落在整齐刻度上。
 */
static void setTempF(const enum SettingsOptions option) {
  uint16_t Temp = getSettingValue(option);
  if (getSettingValue(SettingsOptions::TemperatureInF)) {
    // Change temp to the F equiv
    // C to F == F= ( (C*9) +160)/5
    Temp = ((Temp * 9) + 160) / 5;
  } else {
    // Change temp to the C equiv
    // F->C == C = ((F-32)*5)/9
    Temp = ((Temp - 32) * 5) / 9;
  }
  // Rescale to be multiples of 10
  Temp = Temp / 10;
  Temp *= 10;
  setSettingValue(option, Temp);
}

/**
 * @brief 温度单位切换的总入口（菜单里"Temperature Unit"项的 incrementHandler）
 * @note  必须把所有"以温度为单位"的设置项一起换算，否则切换后 Boost/Sleep/Profile
 *        会保留旧单位的数值（例如 320 F 变成 320 C，直接烧红）。
 *        这也是它不能走 autoSettingOption 自动步进的原因。
 */
static void setTempF(void) {
  nextSettingValue(SettingsOptions::TemperatureInF);
  setTempF(SettingsOptions::BoostTemp);
  setTempF(SettingsOptions::SolderingTemp);
#ifndef NO_SLEEP_MODE
  setTempF(SettingsOptions::SleepTemp);
#endif /* *not* NO_SLEEP_MODE */
#ifdef PROFILE_SUPPORT
  setTempF(SettingsOptions::ProfilePreheatTemp);
  setTempF(SettingsOptions::ProfilePhase1Temp);
  setTempF(SettingsOptions::ProfilePhase2Temp);
  setTempF(SettingsOptions::ProfilePhase3Temp);
  setTempF(SettingsOptions::ProfilePhase4Temp);
  setTempF(SettingsOptions::ProfilePhase5Temp);
#endif /* PROFILE_SUPPORT */
}

// 只画 "°C"/"°F" 符号，具体数字由各界面自己画。
static void displayTempF(void) {
#ifdef OLED_128x32
  // "°C"/"°F" is two small glyphs; right-align it so it stays on screen, centred
  OLED::setCursor(OLED_WIDTH - (2 * 8) - 2, VALUE_Y);
#endif
  OLED::printSymbolDeg(VALUE_FONT);
}

#ifndef NO_DISPLAY_ROTATE

// 显示方向：右/左/自动。AUTO 时不立即改屏幕，交给 MOV 任务按重力方向设置。
static void setDisplayRotation(void) {
  nextSettingValue(SettingsOptions::OrientationMode);
  switch (getSettingValue(SettingsOptions::OrientationMode)) {
  case orientationMode_t::RIGHT:
    OLED::setRotation(false);
    break;
  case orientationMode_t::LEFT:
    OLED::setRotation(true);
    break;
  case orientationMode_t::AUTO:
    // do nothing on auto
    break;
  default:
    break;
  }
}

// 方向显示为 Right/Left/Auto 三个短词。
static void displayDisplayRotation(void) {
  switch (getSettingValue(SettingsOptions::OrientationMode)) {
  case orientationMode_t::RIGHT:
    printValue(translatedString(Tr->SettingRightChar));
    break;
  case orientationMode_t::LEFT:
    printValue(translatedString(Tr->SettingLeftChar));
    break;
  case orientationMode_t::AUTO:
    printValue(translatedString(Tr->SettingAutoChar));
    break;
  default:
    printValue(translatedString(Tr->SettingRightChar));
    break;
  }
}

#endif /* NO_DISPLAY_ROTATE */

// 以下一组是"复选框"类设置：直接把 0/1 画成方框勾选状态。
// 蜂鸣器（部分机型有）、按键反转、动画循环、详细界面、屏幕反色都走同一套。
static void displayCoolingBlinkEnabled(void) { OLED::drawCheckbox(getSettingValue(SettingsOptions::CoolingTempBlink)); }

static void displayBuzzerEnabled(void) { OLED::drawCheckbox(getSettingValue(SettingsOptions::BuzzerEnabled)); }

// 描述文字滚动速度只有"慢/快"两档，映射到两个翻译短词。
static void displayScrollSpeed(void) { printValue(translatedString((getSettingValue(SettingsOptions::DescriptionScrollSpeed)) ? Tr->SettingFastChar : Tr->SettingSlowChar)); }

static void displayReverseButtonTempChangeEnabled(void) { OLED::drawCheckbox(getSettingValue(SettingsOptions::ReverseButtonTempChangeEnabled)); }

static void displayReverseButtonSettings(void) { OLED::drawCheckbox(getSettingValue(SettingsOptions::ReverseButtonSettings)); }

// 菜单图标动画速度：慢/中/快；OFF 时画"不可用"图标（同时隐藏动画循环项）。
static void displayAnimationSpeed(void) {
  switch (getSettingValue(SettingsOptions::AnimationSpeed)) {
  case settingOffSpeed_t::SLOW:
    printValue(translatedString(Tr->SettingSlowChar));
    break;
  case settingOffSpeed_t::MEDIUM:
    printValue(translatedString(Tr->SettingMediumChar));
    break;
  case settingOffSpeed_t::FAST:
    printValue(translatedString(Tr->SettingFastChar));
    break;
  default:
    OLED::drawUnavailableIcon();
    break;
  }
}

// 动画关闭时，"循环播放"没有意义，隐藏该项。
static bool displayAnimationOptions(void) { return getSettingValue(SettingsOptions::AnimationSpeed) > 0; }
static void displayAnimationLoop(void) { OLED::drawCheckbox(getSettingValue(SettingsOptions::AnimationLoop)); }

/**
 * @brief 显示亮度档位并把亮度立刻应用到屏幕
 * @note  显示值 = 设置值/25 + 1（1..5 档），但真正下发给 OLED 的是原始对比度值。
 *        在这里顺手调用 setBrightness 属于"绘制时产生副作用"，虽不优雅但能让
 *        用户按一下就看到效果；注释里也承认了这点。
 */
static void displayBrightnessLevel(void) {
  OLED::printNumber((getSettingValue(SettingsOptions::OLEDBrightness) / BRIGHTNESS_STEP + 1), 1, VALUE_FONT);
  // While not optimal to apply this here, it is _very_ convenient
  OLED::setBrightness(getSettingValue(SettingsOptions::OLEDBrightness));
}

// 反色显示：同样在绘制时立即生效，方便预览。
static void displayInvertColor(void) {
  OLED::drawCheckbox(getSettingValue(SettingsOptions::OLEDInversion));
  // While not optimal to apply this here, it is _very_ convenient
  OLED::setInverseDisplay(getSettingValue(SettingsOptions::OLEDInversion));
}

/**
 * @brief 开机 logo 时长/模式显示
 * @note  0=跳过（不可用图标）、5=显示一次、6=循环显示，其余值按秒数显示。
 *        三个特殊值刻意避开 1..4，避免与"秒数"语义冲突。
 */
static void displayLogoTime(void) {
  switch (getSettingValue(SettingsOptions::LOGOTime)) {
  case logoMode_t::SKIP:
    OLED::drawUnavailableIcon();
    break;
  case logoMode_t::ONETIME:
    OLED::drawArea(OLED_WIDTH - OLED_HEIGHT - 2, 0, OLED_HEIGHT, OLED_HEIGHT, RepeatOnce);
    break;
  case logoMode_t::INFINITY:
    OLED::drawArea(OLED_WIDTH - OLED_HEIGHT - 2, 0, OLED_HEIGHT, OLED_HEIGHT, RepeatInf);
    break;
  default:
    OLED::printNumber(getSettingValue(SettingsOptions::LOGOTime), 1, VALUE_FONT);
    OLED::print(VALUE_SYMBOL(Seconds), VALUE_FONT);
    break;
  }
}

static void displayAdvancedIDLEScreens(void) { OLED::drawCheckbox(getSettingValue(SettingsOptions::DetailedIDLE)); }

static void displayAdvancedSolderingScreens(void) { OLED::drawCheckbox(getSettingValue(SettingsOptions::DetailedSoldering)); }

#ifdef BLE_ENABLED
// BLE 三态：关闭 / 可读写 / 只读（只读用于产线或防误改）。
static void displayBluetoothLE(void) {
  switch (getSettingValue(SettingsOptions::BluetoothLE)) {
  case 0:
    OLED::drawUnavailableIcon();
    break;
  case 1:
    printValue(translatedString(Tr->SettingBLEOnChar));
    break;
  case 2:
    printValue(translatedString(Tr->SettingBLEReadOnlyChar));
    break;
  default:
    OLED::drawUnavailableIcon();
    break;
  }
}
#endif /* BLE_ENABLED */

// 用户功率上限（W）；0 = 不限制，画"不可用"图标。
static void displayPowerLimit(void) {
  if (getSettingValue(SettingsOptions::PowerLimit) == 0) {
    OLED::drawUnavailableIcon();
  } else {
    OLED::printNumber(getSettingValue(SettingsOptions::PowerLimit), 3, VALUE_FONT);
    OLED::print(VALUE_SYMBOL(Watts), VALUE_FONT);
  }
}

/**
 * @brief 请求"下次启动做 CJC 校准"
 * @note  这是不可逆操作（会覆盖温度补偿偏置），所以先弹确认框；
 *        已置位时再按一次则取消，不需要确认。
 */
static void setCalibrate(void) {
  if (getSettingValue(SettingsOptions::CalibrateCJC) < 1) {
    if (userConfirmation(translatedString(Tr->SettingsCalibrationWarning))) {
      // User confirmed
      // So we now set the tick
      setSettingValue(SettingsOptions::CalibrateCJC, 1);
    }
  } else {
    setSettingValue(SettingsOptions::CalibrateCJC, 0);
  }
}

static void displayCalibrate(void) { OLED::drawCheckbox(getSettingValue(SettingsOptions::CalibrateCJC)); }

/**
 * @brief 输入电压校准子界面：实时显示测量电压，用 +/- 调整分压系数
 * @note  这是一个"嵌套模态循环"：内部自己读按键、自己刷屏，直到用户按退出键。
 *        校准原理：设置项 VoltageDiv 是 ADC 分压比，调它使显示值等于万用表读数。
 *        退出时会把系数写 flash（saveSettings），因为用户可能直接断电。
 */
static void setCalibrateVIN(void) {
  // Jump to the voltage calibration subscreen
  OLED::clearScreen();

  for (;;) {
    OLED::setCursor(25, 0);
    // 每次都重新读 ADC（sample=0 表示用最新一次转换结果），
    // 这样调分压系数的效果能立刻反映到读数上。
    uint16_t voltage = getInputVoltageX10(getSettingValue(SettingsOptions::VoltageDiv), 0);
    OLED::printNumber(voltage / 10, 2, VALUE_FONT);
    OLED::print(VALUE_SYMBOL(Dot), VALUE_FONT);
    OLED::printNumber(voltage % 10, 1, VALUE_FONT, false);
    OLED::print(VALUE_SYMBOL(Volts), VALUE_FONT);
    OLED::setCursor(0, VALUE_Y + 8); // below the readout (which is 16px tall on 128x32)
    OLED::printNumber(getSettingValue(SettingsOptions::VoltageDiv), 3, FontStyle::SMALL);

    switch (getButtonState()) {
    case BUTTON_F_SHORT:
    // 前键减小分压系数、后键增大；两个方向都走 settings 的 wrap 逻辑，
    // 因此可以一直按着跨过边界继续调。
      prevSettingValue(SettingsOptions::VoltageDiv);
      break;
    case BUTTON_B_SHORT:
      nextSettingValue(SettingsOptions::VoltageDiv);
      break;
    case BUTTON_BOTH:
    case BUTTON_F_LONG:
    case BUTTON_B_LONG:
      saveSettings();
      OLED::clearScreen();
      OLED::setCursor(0, 0);
      warnUser(translatedString(Tr->CalibrationDone), getButtonState());
      OLED::refresh();
      waitForButtonPressOrTimeout(0.5 * TICKS_SECOND);
      return;
    case BUTTON_NONE:
    default:
      break;
    }

    OLED::refresh();
    osDelay(40);
  }
}

// KeepAwakePulse 功率（x10 W，显示成 x.y W）；0 = 关闭。
// 它的作用是给"会因轻载自动断电"的充电宝/DC-DC 一个周期性脉冲，保持供电。
static void displayPowerPulse(void) {
  if (getSettingValue(SettingsOptions::KeepAwakePulse)) {
    OLED::printNumber(getSettingValue(SettingsOptions::KeepAwakePulse) / 10, 1, VALUE_FONT);
    OLED::print(VALUE_SYMBOL(Dot), VALUE_FONT);
    OLED::printNumber(getSettingValue(SettingsOptions::KeepAwakePulse) % 10, 1, VALUE_FONT);
  } else {
    OLED::drawUnavailableIcon();
  }
}

// 没开脉冲就不显示"间隔/脉宽"两个子项。
static bool showPowerPulseOptions(void) { return getSettingValue(SettingsOptions::KeepAwakePulse) > 0; }

static void displayPowerPulseWait(void) { OLED::printNumber(getSettingValue(SettingsOptions::KeepAwakePulseWait), 1, VALUE_FONT); }

static void displayPowerPulseDuration(void) { OLED::printNumber(getSettingValue(SettingsOptions::KeepAwakePulseDuration), 1, VALUE_FONT); }

/**
 * @brief 恢复出厂设置：确认 → 擦除 → 提示 → 重启
 * @note  重启是必须的：很多模块在启动时读取设置并缓存（PID 限值、屏幕方向、
 *        翻译表），运行中重置无法保证全部生效，重启最干净。
 */
static void setResetSettings(void) {
  if (userConfirmation(translatedString(Tr->SettingsResetWarning))) {
    resetSettings();
    OLED::clearScreen();
    while (!warnUser(translatedString(Tr->ResetOKMessage), getButtonState())) {
      OLED::refresh();
      vTaskDelay(TICKS_100MS);
      OLED::clearScreen();
    }
    reboot();
  }
}

// Indicates whether a menu transition is in progress, so that the menu icon
// animation is paused during the transition.
// 菜单切换动画状态：进入/退出菜单时暂停图标动画，避免"转场 + 图标动画"
// 同时抢 I2C 带宽造成掉帧。
static bool animOpenState = false;

/**
 * @brief 画子菜单的标题和右侧动画图标
 * @param index 子菜单序号（0=Power,1=Soldering,2=PowerSaving,3=UI,4=Advanced）
 * @note  图标是 3 帧位图，帧号由"进入本菜单后经过的时间/步长"决定；
 *        AnimationLoop 决定是循环还是停在最后一帧。
 *        menuCurrentIndex 用 static 保存，切换菜单时重置计时，保证每次进入
 *        动画都从第 0 帧开始。
 */
static void displayMenu(size_t index) {
  // Call into the menu
  // Draw title
  OLED::printWholeScreen(translatedString(Tr->SettingsMenuEntries[index]));
  static TickType_t menuSwitchLoopTick = 0;
  static size_t     menuCurrentIndex   = sizeof(rootSettingsMenu) + 1;
  TickType_t        step               = TICKS_100MS * 5;
  switch (getSettingValue(SettingsOptions::AnimationSpeed)) {
  case settingOffSpeed_t::FAST:
    step = TICKS_100MS * 3;
    break;
  case settingOffSpeed_t::MEDIUM:
    step = TICKS_100MS * 4;
    break;
  default: // SLOW or off - defaulted above
    break;
  }
  size_t currentFrame;
  if (!animOpenState && (getSettingValue(SettingsOptions::AnimationSpeed) != settingOffSpeed_t::OFF)) {
    if (menuCurrentIndex != index) {
      menuCurrentIndex   = index;
      menuSwitchLoopTick = xTaskGetTickCount();
    }
    currentFrame = ((xTaskGetTickCount() - menuSwitchLoopTick) / step);
    if (getSettingValue(SettingsOptions::AnimationLoop)) {
      currentFrame %= 3;
    } else if (currentFrame > 2) {
      currentFrame = 2;
    }
  } else {
    // We want the animation to restart after completing the transition.
    menuCurrentIndex = sizeof(rootSettingsMenu) + 1;
    // Always draw the last frame if icon animation is disabled.
    currentFrame = getSettingValue(SettingsOptions::AnimationSpeed) == settingOffSpeed_t::OFF ? 2 : 0;
  }
  // Draw symbol
  // 16 pixel wide image
  // less 2 pixel wide scrolling indicator

  OLED::drawArea(OLED_WIDTH - SETTINGS_ICON_WIDTH - 2, 0, SETTINGS_ICON_WIDTH, SETTINGS_ICON_HEIGHT, (&SettingsMenuIcons[index][(SETTINGS_ICON_WIDTH * (SETTINGS_ICON_HEIGHT / 8)) * currentFrame]));
}

#if defined(POW_DC) || defined(POW_QC) || defined(POW_PD)
// 五个子菜单标题绘制：只是把序号传给 displayMenu，
// 序号必须与 subSettingsMenus[] 的顺序严格一致。
static void displayPowerMenu(void) { displayMenu(0); }

#endif /* POW_DC or POW_QC */

static void displaySolderingMenu(void) { displayMenu(1); }

static void displayPowerSavingMenu(void) { displayMenu(2); }

static void displayUIMenu(void) { displayMenu(3); }

static void displayAdvancedMenu(void) { displayMenu(4); }
