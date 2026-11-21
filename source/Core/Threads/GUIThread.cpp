/*
 * 【中文导读】GUIThread.cpp —— 界面任务与菜单状态机
 * OLED 绘制 + 按键扫描 + 多级菜单（设置/详情/锁屏）。无 LVGL 的轻量 GUI 实现，
 * UI/ 子目录是各菜单页面。GUIRendering.md 有渲染说明。
 */

/*
 * GUIThread.cpp
 *
 *  Created on: 19 Aug 2019
 *      Author: ralim
 */
extern "C" {
#include "FreeRTOSConfig.h"
}
#include "BootLogo.h"
#include "Buttons.hpp"
#include "I2CBB2.hpp"
#include "LIS2DH12.hpp"
#include "MMA8652FC.hpp"
#include "OLED.hpp"
#include "OperatingModeUtilities.h"
#include "OperatingModes.h"
#include "Settings.h"
#include "TipThermoModel.h"
#include "Translation.h"
#include "cmsis_os.h"
#include "configuration.h"
#include "history.hpp"
#include "main.hpp"
#include "power.hpp"
#include "settingsGUI.hpp"
#include "stdlib.h"
#include "string.h"
#include "ui_drawing.hpp"
#ifdef POW_PD
#include "USBPD.h"
#include "pd.h"
#endif

// File local variables
#define MOVEMENT_INACTIVITY_TIME (60 * configTICK_RATE_HZ)
#define BUTTON_INACTIVITY_TIME   (60 * configTICK_RATE_HZ)

ButtonState   buttonsAtDeviceBoot;                                      // We record button state at startup, incase of jumping to debug modes
OperatingMode currentOperatingMode = OperatingMode::InitialisationDone; // Current mode we are rendering
guiContext    context;                                                  // Context passed to functions to aid in state during render passes

/*
 * GUI 任务总览（20~25 fps）：
 *   guiRenderLoop() 每轮 = guiHandleDraw() 画一帧 + 处理状态迁移 + 可选转场动画 + OLED::refresh()
 *   所有"模式"都是纯函数：输入(按键, context) → 输出下一个 OperatingMode，
 *   自身不保存长期状态；长期状态放在 guiContext（scratch_state）里，
 *   模式切换时被 memset 清空（见 guiRenderLoop）。
 * 关键共享变量：
 *   currentOperatingMode  当前模式（本任务独占写）
 *   currentTempTargetDegC 本任务写、PID 任务读（volatile，单字写入安全）
 *   lastMovementTime/lastButtonTime 由 MOVTask/Buttons 更新，用于休眠判定
 */
OperatingMode handle_post_init_state();
/**
 * @brief 画一帧，并调用当前模式的处理函数得到"下一模式"
 * @return 下一帧应该处于的 OperatingMode
 * @note  顺序很重要：先看按键是否强制亮屏，再决定息屏/状态灯，最后才分发到
 *        具体模式。这样任何模式下按键都能立刻唤醒屏幕，而息屏判定只在
 *        "无按键"时执行，避免用户正在操作时突然黑屏。
 */
OperatingMode guiHandleDraw(void) {
  OLED::clearScreen(); // Clear ready for render pass
  // Read button state
  ButtonState buttons = getButtonState();
  // Enforce screen on if buttons pressed, movement, hot tip etc
  // 有任意按键（含长按/双键）就强制亮屏：这是"用户在场"的最强信号，
  // 也顺便重置了 OLED 的息屏计时（setDisplayState(ON) 内部不会改 lastButtonTime，
  // 但 getButtonState() 已经更新了它）。
  if (buttons != BUTTON_NONE) {
    OLED::setDisplayState(OLED::DisplayState::ON);
  } else {
    // Buttons are none; check if we can sleep display
    // 息屏条件：温度 < 50 C（不烫手）+ 运动感应开启 + 运动和按键都静默超过 60 s。
    // 温度门限的意义：烙铁还热着就把屏幕关掉，用户会误以为已断电。
    uint32_t tipTemp = TipThermoModel::getTipInC();
    if ((tipTemp < 50) && getSettingValue(SettingsOptions::Sensitivity) &&
        (((xTaskGetTickCount() - lastMovementTime) > MOVEMENT_INACTIVITY_TIME) && ((xTaskGetTickCount() - lastButtonTime) > BUTTON_INACTIVITY_TIME))) {
      OLED::setDisplayState(OLED::DisplayState::OFF);
      setStatusLED(LED_OFF);
    } else {
      OLED::setDisplayState(OLED::DisplayState::ON);
    }
    if (currentOperatingMode != OperatingMode::Soldering && currentOperatingMode != OperatingMode::SolderingProfile) {
      // Not in soldering mode, so set this based on temp
      if (tipTemp > 55) {
        setStatusLED(LED_COOLING_STILL_HOT);
      } else {
        setStatusLED(LED_STANDBY);
      }
    }
  }
  // Dispatch button state to gui mode
  OperatingMode newMode = currentOperatingMode;
  // 模式分发：每个 case 都是"画这一帧 + 返回下一模式"。
  // StartupLogo 分支除了画 logo 还会根据 AutoStartMode 决定首次进入哪个模式，
  // 这是整个状态机的"入口岔路"。
  switch (currentOperatingMode) {
  case OperatingMode::StartupWarnings:
    // 启动告警是阻塞式向导（设置被重置 / 器件校验 / 加速度计缺失 / PD 缺失），
    // 每个告警都要用户按键确认，所以它会留在 StartupWarnings 直到走完流程。
    newMode = showWarnings(buttons, &context);
    break;
  case OperatingMode::UsbPDDebug:
#ifdef HAS_POWER_DEBUG_MENU
    newMode = showPDDebug(buttons, &context);
    break;
#else
    newMode = OperatingMode::InitialisationDone;
#endif
  case OperatingMode::StartupLogo:
    showBootLogo();
#ifdef BUZZER_STARTUP_CHIRP
    // Short start-up chirp. Done here rather than in postRToSInit(): that runs right before the USB-PD
    // negotiation starts, and delaying it there makes strict sources hard-reset VBUS.
    setBuzzer(true);
    osDelay(TICKS_100MS);
    setBuzzer(false);
#endif

    if (getSettingValue(SettingsOptions::AutoStartMode) == autoStartMode_t::SLEEP) {
      lastMovementTime = lastButtonTime = 0; // We mask the values so that sleep goes until user moves again or presses a button
      newMode                           = OperatingMode::Sleeping;
    } else if (getSettingValue(SettingsOptions::AutoStartMode) == autoStartMode_t::SOLDER) {
      lastMovementTime = lastButtonTime = xTaskGetTickCount(); // Move forward so we dont go to sleep
      newMode                           = OperatingMode::Soldering;
    } else if (getSettingValue(SettingsOptions::AutoStartMode) == autoStartMode_t::ZERO) {
      lastMovementTime = lastButtonTime = 0; // We mask the values so that sleep goes until user moves again or presses a button
      newMode                           = OperatingMode::Hibernating;
    } else {
      newMode = OperatingMode::HomeScreen;
    }

    break;
  default:
    /* Fallthrough */
  case OperatingMode::HomeScreen:
    newMode = drawHomeScreen(buttons, &context);
    break;
  case OperatingMode::Soldering:
    context.scratch_state.state4 = 0;
    newMode                      = gui_solderingMode(buttons, &context);
    break;
  case OperatingMode::SolderingProfile:
    newMode = gui_solderingProfileMode(buttons, &context);
    break;
  case OperatingMode::Sleeping:
    newMode = gui_SolderingSleepingMode(buttons, &context);
    break;
  case OperatingMode::TemperatureAdjust:
    newMode = gui_solderingTempAdjust(buttons, &context);
    break;
  case OperatingMode::DebugMenuReadout:
    newMode = showDebugMenu(buttons, &context);
    break;
  case OperatingMode::CJCCalibration:
    newMode = performCJCC(buttons, &context);
    break;
  case OperatingMode::SettingsMenu:
    newMode = gui_SettingsMenu(buttons, &context);
    break;
  case OperatingMode::InitialisationDone:
    newMode = handle_post_init_state();
    break;
  case OperatingMode::Hibernating:
    context.scratch_state.state4 = 1;
    gui_SolderingSleepingMode(buttons, &context);
    if (lastButtonTime > 0 || lastMovementTime > 0) {
      newMode = OperatingMode::Soldering;
    }
    break;
  case OperatingMode::ThermalRunaway:
    /*TODO*/
    newMode = OperatingMode::HomeScreen;
    break;
  };
  return newMode;
}
/**
 * @brief 一帧的完整流程：绘制 → 状态迁移 → 转场动画 → 刷屏
 * @note  转场动画需要"下一帧"的内容，所以会临时切到第二 framebuffer 再画一遍
 *        （guiHandleDraw 被调用两次）。这也是本函数不能重入的原因：它直接操作
 *        OLED 的全局缓冲区和 context。
 */
void guiRenderLoop(void) {
  OperatingMode newMode = guiHandleDraw(); // This does the screen drawing

  // Post draw we handle any state transitions

  // 状态迁移的收尾工作：
  //   viewEnterTime 交给转场动画做时间轴；
  //   previousMode  让子模式（如温度调节）知道"退出后回哪"；
  //   scratch_state 必须清零，否则上一个模式残留的 state1/state2 会被新模
  //   式误读（各模式对 stateN 的含义完全不同）。
  if (newMode != currentOperatingMode) {
    context.viewEnterTime = xTaskGetTickCount();
    context.previousMode  = currentOperatingMode;
    // If the previous mode is the startup logo; we dont want to return to it, but instead dispatch out to either home or soldering
    // 特例：启动 logo 只应出现一次。若 AutoStartMode 非 0，退出 logo 后直接进
    // 焊接模式；否则强制去 HomeScreen，而不是回到 logo。
    if (currentOperatingMode == OperatingMode::StartupLogo) {
      if (getSettingValue(SettingsOptions::AutoStartMode)) {
        context.previousMode = OperatingMode::Soldering;
      } else {
        newMode = OperatingMode::HomeScreen;
      }
    }
    memset(&context.scratch_state, 0, sizeof(context.scratch_state));
    currentOperatingMode = newMode;
  }

  // If the transition marker is set, we need to make the next draw occur to the secondary buffer so we have something to transition to
  // 转场动画：先把"目标模式"画进第二缓冲区，再让 OLED 模块做像素级过渡。
  // 注意 Down 分支少了 break（贯穿到 None），行为上等价于"画完就结束"，
  // 属于历史遗留写法，阅读时不要误以为会继续执行 ScrollUp。
  if (context.transitionMode != TransitionAnimation::None) {
    OLED::useSecondaryFramebuffer(true);
    // Now we need to fill the secondary buffer with the _next_ frame to transistion to
    guiHandleDraw();
    OLED::useSecondaryFramebuffer(false);
    // Now dispatch the transition
    switch (context.transitionMode) {
    case TransitionAnimation::Down:
      OLED::transitionScrollDown(context.viewEnterTime);
      break;
    case TransitionAnimation::Left:
      OLED::transitionSecondaryFramebuffer(false, context.viewEnterTime);
      break;
    case TransitionAnimation::Right:
      OLED::transitionSecondaryFramebuffer(true, context.viewEnterTime);
      break;
    case TransitionAnimation::Up:
      OLED::transitionScrollUp(context.viewEnterTime);

    case TransitionAnimation::None:
    default:
      break; // Do nothing on unknown
    }

    context.transitionMode = TransitionAnimation::None; // Clear transition flag
  }
  // Render done, draw it out
  OLED::refresh();
}

/**
 * @brief 初始化完成后的路由：调试菜单 / CJC 校准 / 启动告警
 * @return 下一个模式
 * @note  上电时按住按键可以进 PD 调试菜单（HAS_POWER_DEBUG_MENU 才有）。
 *        buttonsAtDeviceBoot 是在 startGUITask 里、OLED 初始化后立即采样的，
 *        因此"按住不放再上电"能被可靠捕获。
 */
OperatingMode handle_post_init_state() {
#ifdef HAS_POWER_DEBUG_MENU
#ifdef DEBUG_POWER_MENU_BUTTON_B
  if (buttonsAtDeviceBoot == BUTTON_B_LONG || buttonsAtDeviceBoot == BUTTON_B_SHORT) {
#else
  if (buttonsAtDeviceBoot == BUTTON_F_LONG || buttonsAtDeviceBoot == BUTTON_F_SHORT) {
#endif
    buttonsAtDeviceBoot = BUTTON_NONE;
    return OperatingMode::UsbPDDebug;
  }
#endif

  if (getSettingValue(SettingsOptions::CalibrateCJC) > 0) {
    return OperatingMode::CJCCalibration;
  }

  return OperatingMode::StartupWarnings;
}

/* StartGUITask function */
/**
 * @brief GUI 任务入口：初始化 OLED/翻译/方向，然后以 ~20-25 fps 跑渲染循环
 * @param argument 未使用
 * @note  优先级 osPriorityBelowNormal（最低的业务任务），因为丢一帧无所谓；
 *        但它持有 I2C 总线（OLED 走 I2C），所以 vTaskDelayUntil 的节拍也起到
 *        "给 MOV 任务的加速度计读取让路"的作用（见 GUIDelay 的注释）。
 */
void startGUITask(void const *argument) {
  (void)argument;
  // 翻译表放在外部 flash/大数组里，首次访问前必须建立索引；
  // 放在 OLED 初始化之前，保证第一帧就能画出正确文字。
  prepareTranslations();

  OLED::initialize(); // start up the LCD
  OLED::setBrightness(getSettingValue(SettingsOptions::OLEDBrightness));
  OLED::setInverseDisplay(getSettingValue(SettingsOptions::OLEDInversion));

  bool buttonLockout = false;
  ui_pre_render_assets();
  // 丢弃上电后的第一次 ADC 读数并重置滤波历史：
  // 冷启动时 ADC 前端（运放/基准）尚未稳定，这次读数会污染滑动平均。
  getTipRawTemp(1); // reset filter
  memset(&context, 0, sizeof(context));

  OLED::setRotation(getSettingValue(SettingsOptions::OrientationMode) & 1);

  // Read boot button state
  // 记录上电瞬间的按键状态，用于 handle_post_init_state 的"调试模式入口"。
  // 这里只区分"按住"，不区分长短按，所以直接赋 LONG。
  if (getButtonA()) {
    buttonsAtDeviceBoot = BUTTON_F_LONG;
  }
  if (getButtonB()) {
    buttonsAtDeviceBoot = BUTTON_B_LONG;
  }

  TickType_t startRender = xTaskGetTickCount();
  for (;;) {
    guiRenderLoop();
    resetWatchdog();
    // 固定节拍 40 ms（25 fps）。用 DelayUntil 而不是 Delay，避免每帧的绘制耗时
    // 累积成漂移；绘制超时会自然降帧，但不会越拖越慢。
    vTaskDelayUntil(&startRender, TICKS_100MS * 4 / 10); // Try and maintain 20-25fps ish update rate, way to fast but if we can its nice
  }
}
