/*
 * 【中文导读】Translation.cpp —— 多语言字库（B级导读）
 * 编译期选择语言字库（由 Translations/ 的 Python 脚本生成），内存友好。
 */

#include "Translation.h"
/*
 * ============================================================================
 * 【文件导读】Translation.cpp —— 多语言字符串表的访问器
 * ----------------------------------------------------------------------------
 * 翻译表由 Translations/ 下的 Python 脚本在编译期生成（C 级生成物，勿手改），
 * 所有语言被打包成一个大 char 数组 TranslationStrings，每条字符串用
 * uint16 偏移量索引（Translation.h 里的 Tr-> 结构体就是一堆偏移）。
 * translatedString(offset) 只做一次指针加法：offset 为 0 时返回空串。
 * 这样做的代价是"字符串不能超过 64 KB 总长"，收益是零运行时解析、可直接
 * 放在 flash 里按需读取（NOR flash 支持随机读）。
 * ============================================================================
 */

const char *translatedString(uint16_t offset) { return TranslationStrings + offset; }
