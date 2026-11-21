/*
 * Power.hpp
 *
 *  Created on: 28 Oct, 2018
 *     Authors: Ben V. Brown, David Hilton (David's Idea)
 */

#include "BSP.h"
#include "configuration.h"
#include "expMovingAverage.h"
#include "stdint.h"
#include <history.hpp>
/*
 * ============================================================================
 * 【文件导读】power.hpp —— 功率换算模块的对外接口
 * ----------------------------------------------------------------------------
 *   availableW10(sample)      当前电源在满占空下的可用功率（x10 W）
 *   getX10WattageLimits()     综合限值（电源/用户/PD/硬件/手柄降额）
 *   setTipX10Watts(mw)        设定功率：换算 PWM + 下发 + 回读统计
 *   X10WattsToPWM(mw, sample) 纯换算，不改硬件
 * wattHistoryFilter=24：x10WattHistory 是 24 点滑动平均，GUI 功率条与
 * "实际输出"显示都读它；窗口约 24 个控制周期（约 3 s @8 Hz）。
 * 单位说明：函数名里的 W10/x10W 都是 0.1 W；历史注释里的 mW 是早期命名残留。
 * ============================================================================
 */
#ifndef POWER_HPP_
#define POWER_HPP_

// thermal mass = 1690 milliJ/*C for my tip.
//  ->  Wattsx10*Seconds to raise Temp from room temp to +100*C, divided by 100*C.
// we divide mass by 20 to let the I term dominate near the set point.
//  This is necessary because of the temp noise and thermal lag in the system.
// Once we have feed-forward temp estimation we should be able to better tune this.

const uint8_t                                        wattHistoryFilter = 24; // I term look back weighting
extern expMovingAverage<uint32_t, wattHistoryFilter> x10WattHistory;

uint32_t availableW10(uint8_t sample);
int32_t  getX10WattageLimits(); // Effective x10 W limit after supply / user / hardware / derate limits (PIDThread)
void     setTipX10Watts(int32_t mw);
uint8_t  X10WattsToPWM(int32_t milliWatts, uint8_t sample = 0);
#endif /* POWER_HPP_ */
