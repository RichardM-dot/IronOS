/*
 * 【中文导读】Translation.cpp —— 多语言字库（B级导读）
 * 编译期选择语言字库（由 Translations/ 的 Python 脚本生成），内存友好。
 */

#include "Translation.h"

const char *translatedString(uint16_t offset) { return TranslationStrings + offset; }
