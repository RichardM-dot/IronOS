#include "USBPD.h"
#include "configuration.h"
#ifdef POW_PD
#include "BSP_PD.h"
#include "FreeRTOS.h"
#include "Settings.h"
#include "fusb302b.h"
#include "main.hpp"
#include "pd.h"
#include "policy_engine.h"

#ifndef USB_PD_VMAX
#error Max PD Voltage must be defined
#endif

/*
 * USB-PD 适配层总览
 * ---------------------------------------------------------------------------
 * Drivers/usb-pd/ 是一套独立的 PD 协议栈（PolicyEngine + FUSB302 驱动），
 * 它通过四个回调与固件交互：
 *   pdbs_dpm_get_sink_capability()  告诉充电器"我要什么"（Sink Capabilities）
 *   pdbs_dpm_evaluate_capability()  从 Source Capabilities 里挑一档并构造 Request
 *   EPREvaluateCapabilityFunc()     EPR（>20V）档位的同样逻辑
 *   ms_delay / get_ms_timestamp     时间基准与阻塞延时
 * 本文件就是这层"胶水"，外加一个纯 C 的整数平方根（EPR 用）。
 * 电压/电流单位非常容易混淆，统一约定：
 *   PDO 原始字段：电压 50 mV/bit、电流 10 mA/bit
 *   选择结果 bestVoltage 用 mV、bestCurrent 用 10 mA（centi-amp 的 1/10）
 * 协商结果写回 requested_voltage_mv 与 powerSupplyWattageLimit（单位 W）。
 */
void ms_delay(uint32_t delayms) {
  // Convert ms -> ticks
  TickType_t ticks = delayms / portTICK_PERIOD_MS;

  vTaskDelay(ticks ? ticks : 1); /* Minimum delay = 1 tick */
}
// 协议栈要的是毫秒时间戳，而 FreeRTOS 给的是 tick（本项目 1 tick=1 ms，
// 仍然显式乘 portTICK_PERIOD_MS，方便换到别的 tick 配置时不出错）。
uint32_t get_ms_timestamp() {
  // Convert ticks -> ms
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
bool         pdbs_dpm_evaluate_capability(const pd_msg *capabilities, pd_msg *request);
void         pdbs_dpm_get_sink_capability(pd_msg *cap, const bool isPD3);
bool         EPREvaluateCapabilityFunc(const epr_pd_msg *capabilities, pd_msg *request);
// 全局对象：FUSB302 驱动 + 策略引擎。构造参数是 I2C 地址（左移 1 位）与
// BSP 提供的读写/延时函数，因此本文件不直接碰 HAL。
// detectionState：0=未探测，1=找到芯片；缓存结果避免每次访问都读 I2C。
FUSB302      fusb((0x22 << 1), fusb_read_buf, fusb_write_buf, ms_delay); // Create FUSB driver
PolicyEngine pe(fusb, get_ms_timestamp, ms_delay, pdbs_dpm_get_sink_capability, pdbs_dpm_evaluate_capability, EPREvaluateCapabilityFunc, USB_PD_EPR_WATTAGE);
int          USBPowerDelivery::detectionState = 0;
bool         haveSeenCapabilityOffer          = false;
uint16_t     requested_voltage_mv             = 0;

/* The current draw when the output is disabled */
#define DPM_MIN_CURRENT PD_MA2PDI(100)

// Start processing
/**
 * @brief 初始化 FUSB302 并打开其 IRQ（由 POWTask 在调度器启动后调用一次）
 * @return true = 芯片存在且配置成功
 * @note  内部先做芯片 ID 探测，再做寄存器配置；失败时保持静默，后续
 *        negotiationComplete() 会因 fusbPresent()==false 而立即返回 true，
 *        整个系统退化为"普通 5V 供电"，不影响烙铁基本功能。
 */
bool USBPowerDelivery::start() {
  if (fusbPresent() && fusb.fusb_setup()) {
    setupFUSBIRQ();
    return true;
  }
  return false;
}
// 以下四个是极薄的转发：把"事件/节拍/状态查询"传给策略引擎。
// 之所以还要包一层，是为了让上层（POWTask/PIDThread/GUI）不直接依赖 usb-pd 头文件。
// negotiationInProgress：还在等第一份 Source Capabilities；PID 启动时会短暂等它。
// negotiationComplete：已收到能力集或超时（超时值来自设置项 PDNegTimeout）。

void    USBPowerDelivery::IRQOccured() { pe.IRQOccured(); }
bool    USBPowerDelivery::negotiationHasWorked() { return pe.pdHasNegotiated(); }
uint8_t USBPowerDelivery::getStateNumber() { return pe.currentStateCode(true); }
void    USBPowerDelivery::step() {
  while (pe.thread()) {
  }
}

void USBPowerDelivery::PPSTimerCallback() { pe.TimersCallback(); }
bool USBPowerDelivery::negotiationInProgress() {
  if (USBPowerDelivery::negotiationComplete()) {
    return false;
  }
  if (haveSeenCapabilityOffer) {
    return false;
  }
  return true;
}
bool USBPowerDelivery::negotiationComplete() {
  if (!fusbPresent()) {
    return true;
  }
  return pe.setupCompleteOrTimedOut(getSettingValue(SettingsOptions::PDNegTimeout));
}
bool USBPowerDelivery::fusbPresent() {
  if (detectionState == 0) {
    if (fusb.fusb_read_id()) {
      detectionState = 1;
    }
  }
  return detectionState == 1;
}

/**
 * @brief 探测 VBUS 是否真的带电（部分机型 PD 芯片由 VBUS 供电，拔线后仍可通信）
 * @return true = VBUS 存在
 * @note  结果用 static state 永久缓存（1=有，2=无）：VBUS 状态在运行中不会变，
 *        而 I2C 读一次要几百微秒，没必要反复测。协商未完成前一律返回 true，
 *        避免把"正在协商"误判成"没插电"。
 */
bool USBPowerDelivery::isVBUSConnected() {
#if NEEDS_VBUS_PROBE == 1
  static uint8_t state = 0;
  if (state) {
    return state == 1;
  }
  // Dont run if we havent negotiated
  if (!negotiationComplete()) {
    return true;
  }
  if (fusb.isVBUSConnected()) {
    state = 1;
    return true;
  } else {
    state = 2;
    return false;
  }
#else
  return false;
#endif
}
uint32_t  lastCapabilities[11];
uint32_t *USBPowerDelivery::getLastSeenCapabilities() { return lastCapabilities; }

#ifdef POW_EPR
/**
 * @brief 整数平方根（逐位逼近，无浮点、无除法）
 * @param sqrtArg 被开方数
 * @return floor(sqrt(sqrtArg))
 * @note  EPR/AVS 档位给的是"最大功率 + 电阻"，需要反解电压：V=sqrt(P*R)。
 *        在 Cortex-M0/M3 上没有 FPU，浮点 sqrt 会拖入大量库代码，所以手写。
 *        算法：从 bit15 开始试置位，平方后超了就撤销，16 次迭代得到 16 位结果。
 */
static unsigned int sqrtI(unsigned long sqrtArg) {
  unsigned int  answer, x;
  unsigned long temp;
  if (sqrtArg == 0) {
    return 0; // undefined result
  }
  if (sqrtArg == 1) {
    return 1; // identity
  }
  answer = 0;                           // integer square root
  for (x = 0x8000; x > 0; x = x >> 1) { // 16 bit shift
    answer |= x;                        // possible bit in root
    temp = answer * answer;             //
    if (temp == sqrtArg) {
      break; // exact, found it
    }
    if (temp > sqrtArg) {
      answer ^= x; // too large, reverse bit
    }
  }
  return answer; // approximate root
}
#endif

// parseCapabilitiesArray returns true if a valid capability was found
// caps is the array of capabilities objects
// best* are output references
/**
 * @brief 从充电器广播的 PDO 列表里挑"最适合当前烙铁"的一档
 * @param numCaps     PDO 个数
 * @param bestIndex   输出：选中的 PDO 下标（0xFF 表示没选中）
 * @param bestVoltage 输出：目标电压（mV）
 * @param bestCurrent 输出：目标电流（10 mA 单位）
 * @param bestIsPPS   输出：是否为 PPS 档（可调压）
 * @param bestIsAVS   输出：是否为 AVS 档（EPR 可调压）
 * @return true = 找到可用档位
 * @note  选择策略不是"电压越高越好"，而是"在电流能力足够的前提下取最高电压"：
 *        烙铁是纯电阻负载，功率 ≈ V²/R，电压越高功率越大；但如果某档的
 *        最大电流撑不住 V/R，插上去会被拉垮（充电器限流/重启），必须排除。
 *        判据 min_resistance = V/I <= R_tip 即为此意。
 *        DEFAULT 模式额外把 R_tip 加 0.5 Ω，用来补偿线缆/PCB 压降，
 *        让充电器多给一点电流余量（SAFE 模式不加，NO_DYNAMIC 只选固定档）。
 */
bool parseCapabilitiesArray(const uint8_t numCaps, uint8_t *bestIndex, uint16_t *bestVoltage, uint16_t *bestCurrent, bool *bestIsPPS, bool *bestIsAVS) {
  // Walk the given capabilities array; and select the best option
  // Given assumption of fixed tip resistance; this can be simplified to highest voltage selection
  *bestIndex   = 0xFF; // Mark unselected
  *bestVoltage = 5000; // Default 5V

  // Fudge of 0.5 ohms to round up a little to account for us always having off periods in PWM
  uint8_t     tipResistance = getTipResistanceX10();
  usbpdMode_t pd_mode       = (usbpdMode_t)getSettingValue(SettingsOptions::USBPDMode);
  if (pd_mode == usbpdMode_t::DEFAULT) {
    tipResistance += 5;
  }
#ifdef MODEL_HAS_DCDC
  // If this device has step down DC/DC inductor to smooth out current spikes
  // We can instead ignore resistance and go for max voltage we can accept; and rely on the DC/DC regulation to keep under current limit
  tipResistance = 255; // (Push to 25.5 ohms to effectively disable this check)
#endif

  for (uint8_t i = 0; i < numCaps; i++) {
    if ((lastCapabilities[i] & PD_PDO_TYPE) == PD_PDO_TYPE_FIXED) {
      // This is a fixed PDO entry
      // Evaluate if it can produve sufficient current based on the TIP_RESISTANCE (ohms*10)
      // V=I*R -> V/I => minimum resistance, if our tip resistance is >= this then we can use this supply

      int voltage_mv             = PD_PDV2MV(PD_PDO_SRC_FIXED_VOLTAGE_GET(lastCapabilities[i])); // voltage in mV units
      int current_a_x100         = PD_PDO_SRC_FIXED_CURRENT_GET(lastCapabilities[i]);            // current in 10mA units
      // 单位推导：mV / (10 mA) = 0.1 Ω，正好与 tipResistance（0.1 Ω 单位）同量纲，
      // 所以可以直接比较，不需要再乘除 100。
      int min_resistance_ohmsx10 = voltage_mv / current_a_x100;
      if (voltage_mv > 0) {
        if (voltage_mv <= (USB_PD_VMAX * 1000)) {
          if (voltage_mv <= 20000 || (pd_mode != usbpdMode_t::NO_DYNAMIC)) {
            if (min_resistance_ohmsx10 <= tipResistance) {
              // This is a valid power source we can select as
              // 只在"更高电压"时替换：同等电压下保留先出现的那档（通常是更低电流），
              // 避免为了同一电压去要一个更激进的电流档。
              if (voltage_mv > *bestVoltage) {

                // Higher voltage and valid, select this instead
                *bestIndex   = i;
                *bestVoltage = voltage_mv;
                *bestCurrent = current_a_x100;
                *bestIsPPS   = false;
                *bestIsAVS   = false;
              }
            }
          }
        }
      }
    } else if (((lastCapabilities[i] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED) && (pd_mode != usbpdMode_t::NO_DYNAMIC)) {
      bool sourceIsEPRCapable = lastCapabilities[0] & PD_PDO_SRC_FIXED_EPR_CAPABLE;
      bool isPPS              = false;
      bool isAVS              = false;
      if (sourceIsEPRCapable) {
        isPPS = (lastCapabilities[i] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS;
        isAVS = (lastCapabilities[i] & PD_APDO_TYPE) == PD_APDO_TYPE_AVS;
      } else {
        isPPS = true; // Assume PPS if no EPR support
      }
      if (isPPS) {
        // If this is a PPS slot, calculate the max voltage in the PPS range that can we be used and maintain
        uint16_t max_voltage = PD_PAV2MV(PD_APDO_PPS_MAX_VOLTAGE_GET(lastCapabilities[i]));
        // uint16_t min_voltage = PD_PAV2MV(PD_APDO_PPS_MIN_VOLTAGE_GET(lastCapabilities[i]));
        uint16_t max_current = PD_PAI2CA(PD_APDO_PPS_CURRENT_GET(lastCapabilities[i])); // max current in 10mA units
        // Using the current and tip resistance, calculate the ideal max voltage
        // if this is range, then we will work with this voltage
        // if this is not in range; then max_voltage can be safely selected
        // PPS：电压可调，于是直接解出"刚好用满这档电流"的电压 V=I*R。
        // 若超过档位上限则夹到上限；再统一夹到 20 V（有些充电器标 21 V 但不稳）
        // 与机型允许的 USB_PD_VMAX。
        int ideal_voltage_mv = (tipResistance * max_current);
        if (ideal_voltage_mv > max_voltage) {
          ideal_voltage_mv = max_voltage; // constrain to what this PDO offers
        }
        if (ideal_voltage_mv > 20000) {
          ideal_voltage_mv = 20000; // Limit to 20V as some advertise 21 but are not stable at 21
        }
        if (ideal_voltage_mv > (USB_PD_VMAX * 1000)) {
          ideal_voltage_mv = (USB_PD_VMAX * 1000); // constrain to model max voltage safe to select
        }
        if (ideal_voltage_mv > *bestVoltage) {
          *bestIndex   = i;
          *bestVoltage = ideal_voltage_mv;
          *bestCurrent = max_current;
          *bestIsPPS   = true;
          *bestIsAVS   = false;
        }
      }
#ifdef POW_EPR
      else if (isAVS) {
        uint16_t max_voltage = PD_PAV2MV(PD_APDO_AVS_MAX_VOLTAGE_GET(lastCapabilities[i]));
        uint8_t  max_wattage = PD_APDO_AVS_MAX_POWER_GET(lastCapabilities[i]);
        tipResistance        = getTipResistanceX10(); // Dont use fudge factor for EPR

        // W = v^2/tip_resistance => Wattage*tip_resistance == Max_voltage^2
        // AVS：档位给的是功率(W)与电阻(0.1Ω)。W = V²/R → V = sqrt(W*R)。
        // max_wattage 单位 W，tipResistance 单位 0.1Ω，先 /10 化到 Ω 再开方，
        // 结果乘 1000 转成 mV。
        auto ideal_max_voltage = sqrtI((max_wattage * tipResistance) / 10) * 1000;
        if (ideal_max_voltage > (USB_PD_VMAX * 1000)) {
          ideal_max_voltage = (USB_PD_VMAX * 1000); // constrain to model max voltage safe to select
        }
        if (ideal_max_voltage > (max_voltage)) {
          ideal_max_voltage = (max_voltage); // constrain to model max voltage safe to select
        }
        auto operating_current = (ideal_max_voltage / tipResistance); // Current in centiamps

        if (ideal_max_voltage > *bestVoltage) {
          *bestIndex   = i;
          *bestVoltage = ideal_max_voltage;
          *bestCurrent = operating_current;
          *bestIsAVS   = true;
          *bestIsPPS   = false;
        }
      }
#endif
    }
  }
  // Now that the best index is known, set the current values
  return *bestIndex != 0xFF; // have we selected one
}

/**
 * @brief EPR（Extended Power Range，>20 V）能力评估回调
 * @param capabilities 11 个 PDO（1..7 标准档，8..11 为 EPR 档）
 * @param request      输出：要发给充电器的 Request 消息
 * @return 恒为 true（即使没匹配上也回 5 V，保证握手继续）
 * @note  与普通档最大的区别：Request 有两个数据对象，obj[1] 必须回抄选中的 PDO，
 *        obj[0] 才是 RDO；并且要置 EPR_CAPABLE 位，否则充电器不会进 EPR 模式。
 *        协商成功后会更新 powerSupplyWattageLimit，PID 的功率上限立刻跟着变。
 */
bool EPREvaluateCapabilityFunc(const epr_pd_msg *capabilities, pd_msg *request) {
#ifdef POW_EPR
  // Select any EPR slots up to USB_PD_VMAX
  memset(lastCapabilities, 0, sizeof(lastCapabilities));
  memcpy(lastCapabilities, capabilities->obj, sizeof(lastCapabilities));
  // PDO slots 1-7 shall be the standard PDO's
  // PDO slots 8-11 shall be the >20V slots
  uint8_t  numobj           = 11;
  uint8_t  bestIndex        = 0xFF;
  uint16_t bestIndexVoltage = 0;
  uint16_t bestIndexCurrent = 0;
  bool     bestIsPPS        = false;
  bool     bestIsAVS        = false;

  if (parseCapabilitiesArray(numobj, &bestIndex, &bestIndexVoltage, &bestIndexCurrent, &bestIsPPS, &bestIsAVS)) {
    /* We got what we wanted, so build a request for that */
    request->hdr    = PD_MSGTYPE_EPR_REQUEST | PD_NUMOBJ(2);
    request->obj[1] = lastCapabilities[bestIndex]; // Copy PDO into slot 2

    if (bestIsAVS) {
      request->obj[0] = PD_RDO_PROG_CURRENT_SET(PD_CA2PAI(bestIndexCurrent)) | PD_RDO_PROG_VOLTAGE_SET(PD_MV2APS(bestIndexVoltage));
    } else if (bestIsPPS) {
      request->obj[0] = PD_RDO_PROG_CURRENT_SET(PD_CA2PAI(bestIndexCurrent)) | PD_RDO_PROG_VOLTAGE_SET(PD_MV2PRV(bestIndexVoltage));
    } else {
      request->obj[0] = PD_RDO_FV_MAX_CURRENT_SET(bestIndexCurrent) | PD_RDO_FV_CURRENT_SET(bestIndexCurrent);
    }
    request->obj[0] |= PD_RDO_EPR_CAPABLE;
    request->obj[0] |= PD_RDO_NO_USB_SUSPEND;
    request->obj[0] |= PD_RDO_OBJPOS_SET(bestIndex + 1);

    // We dont do usb
    // request->obj[0] |= PD_RDO_USB_COMMS;

    /* Update requested voltage */
    requested_voltage_mv    = bestIndexVoltage;
    powerSupplyWattageLimit = bestIndexVoltage * bestIndexCurrent / 100 / 1000; // Set watts for limit from PSU limit

  } else {
    /* Nothing matched (or no configuration), so get 5 V at low current */
    request->hdr    = PD_MSGTYPE_EPR_REQUEST | PD_NUMOBJ(2);
    request->obj[1] = lastCapabilities[0];
    request->obj[0] = PD_RDO_FV_MAX_CURRENT_SET(100) | PD_RDO_FV_CURRENT_SET(100) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(1);
    // We dont do usb
    // request->obj[0] |= PD_RDO_USB_COMMS;

    /* Update requested voltage */
    requested_voltage_mv = 5000;
  }
  return true;
#endif
  return false;
}

/**
 * @brief 普通（SPR，<=20 V）能力评估回调：挑档 + 构造 Request
 * @note  把 Source Capabilities 缓存到 lastCapabilities（GUI 调试页要显示），
 *        置 haveSeenCapabilityOffer，让 negotiationInProgress() 立刻变 false。
 *        即使一档都选不上也返回 true 并请求 5 V/100 mA——PD 规范要求 Sink
 *        必须给出合法 Request，否则充电器会认为对方不支持 PD。
 */
bool pdbs_dpm_evaluate_capability(const pd_msg *capabilities, pd_msg *request) {
  memset(lastCapabilities, 0, sizeof(lastCapabilities));
  memcpy(lastCapabilities, capabilities->obj, sizeof(uint32_t) * 7);
  haveSeenCapabilityOffer = true;
  /* Get the number of PDOs */
  uint8_t numobj = PD_NUMOBJ_GET(capabilities);

  /* Make sure we have configuration */
  /* Look at the PDOs to see if one matches our desires */
  // Look against USB_PD_Desired_Levels to select in order of preference
  uint8_t  bestIndex        = 0xFF;
  uint16_t bestIndexVoltage = 0;
  uint16_t bestIndexCurrent = 0;
  bool     bestIsPPS        = false;
  bool     bestIsAVS        = false;

  if (parseCapabilitiesArray(numobj, &bestIndex, &bestIndexVoltage, &bestIndexCurrent, &bestIsPPS, &bestIsAVS)) {
    /* We got what we wanted, so build a request for that */
    request->hdr = PD_MSGTYPE_REQUEST | PD_NUMOBJ(1);
    if (bestIsPPS) {
      request->obj[0] = PD_RDO_PROG_CURRENT_SET(PD_CA2PAI(bestIndexCurrent)) | PD_RDO_PROG_VOLTAGE_SET(PD_MV2PRV(bestIndexVoltage)) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(bestIndex + 1);
    } else {
      request->obj[0] = PD_RDO_FV_MAX_CURRENT_SET(bestIndexCurrent) | PD_RDO_FV_CURRENT_SET(bestIndexCurrent) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(bestIndex + 1);
    }
    // We dont do usb
    // request->obj[0] |= PD_RDO_USB_COMMS;
#ifdef POW_EPR
    request->obj[0] |= PD_RDO_EPR_CAPABLE;
#endif

    /* Update requested voltage */
    requested_voltage_mv    = bestIndexVoltage;
    powerSupplyWattageLimit = bestIndexVoltage * bestIndexCurrent / 100 / 1000; // Set watts for limit from PSU limit

  } else {
    /* Nothing matched (or no configuration), so get 5 V at low current */
    request->hdr    = PD_MSGTYPE_REQUEST | PD_NUMOBJ(1);
    request->obj[0] = PD_RDO_FV_MAX_CURRENT_SET(100) | PD_RDO_FV_CURRENT_SET(100) | PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(1);
    // We dont do usb
    // request->obj[0] |= PD_RDO_USB_COMMS;

    /* Update requested voltage */
    requested_voltage_mv = 5000;
  }
  // Even if we didnt match, we return true as we would still like to handshake on 5V at the minimum
  return true;
}

/**
 * @brief 往 Sink Capabilities 里追加一条固定电压 PDO
 * @param voltage_mv 电压（mV）
 * @param numobj     写入下标
 * @note  电流按 I=V/R 计算：告诉充电器"这个电压下我需要多少电流"。
 *        用当前烙铁电阻直接算，等于向充电器声明"我是一根电阻丝"。
 */
void add_v_record(pd_msg *cap, uint16_t voltage_mv, int numobj) {

  uint16_t current = (voltage_mv) / getTipResistanceX10(); // In centi-amps

  /* Add a PDO for the desired power. */
  cap->obj[numobj] = PD_PDO_TYPE_FIXED | PD_PDO_SNK_FIXED_VOLTAGE_SET(PD_MV2PDV(voltage_mv)) | PD_PDO_SNK_FIXED_CURRENT_SET(current);
}
/**
 * @brief 构造 Sink Capabilities：声明本机想要的电压/电流档位
 * @param cap   输出消息
 * @param isPD3 是否 PD3.0（本实现未区分，保留参数）
 * @note  第一档必须是 5 V（PD 规范强制），随后按 USB_PD_VMAX 依次追加
 *        9/12/15/20 V。电压必须"从低到高"排列，否则部分充电器会拒绝。
 *        这些档位只是"我能接受的范围"，真正要哪一档由 evaluate 回调决定。
 */
void pdbs_dpm_get_sink_capability(pd_msg *cap, const bool isPD3) {
  /* Keep track of how many PDOs we've added */
  int numobj = 0;

  /* If we have no configuration or want something other than 5 V, add a PDO
   * for vSafe5V */
  /* Minimum current, 5 V, and higher capability. */
  cap->obj[numobj++] = PD_PDO_TYPE_FIXED | PD_PDO_SNK_FIXED_VOLTAGE_SET(PD_MV2PDV(5000)) | PD_PDO_SNK_FIXED_CURRENT_SET(DPM_MIN_CURRENT);
  // Voltages must be in order of lowest -> highest
#if USB_PD_VMAX >= 20
  add_v_record(cap, 9000, numobj);
  numobj++;
  add_v_record(cap, 15000, numobj);
  numobj++;
  add_v_record(cap, 20000, numobj);
  numobj++;
#elif USB_PD_VMAX >= 15
  add_v_record(cap, 9000, numobj);
  numobj++;
  add_v_record(cap, 12000, numobj);
  numobj++;
  add_v_record(cap, 15000, numobj);
  numobj++;
#elif USB_PD_VMAX >= 12
  add_v_record(cap, 9000, numobj);
  numobj++;
  add_v_record(cap, 12000, numobj);
  numobj++;
#elif USB_PD_VMAX >= 9
  add_v_record(cap, 9000, numobj);
  numobj++;
#endif

  /* Set the USB communications capable flag. */
  // 声明支持 USB 通信（本机不做 USB-PD 数据通信，但这个位影响部分充电器的
  // 供电策略：置位后它们更愿意在 5 V 档给出较大电流）。
  cap->obj[0] |= PD_PDO_SNK_FIXED_USB_COMMS;

  /* Set the Sink_Capabilities message header */
  cap->hdr = PD_DATAROLE_UFP | PD_SPECREV_3_0 | PD_POWERROLE_SINK | PD_MSGTYPE_SINK_CAPABILITIES | PD_NUMOBJ(numobj);
}

#endif
