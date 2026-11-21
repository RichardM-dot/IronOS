/*
 * 【中文导读】syscalls.c —— newlib 系统调用桩（B级导读）
 * printf 重定向到调试串口的底层对接。
 */

/* Includes */
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <time.h>
/*
 * ============================================================================
 * 【文件导读】syscalls.c —— newlib 系统调用桩
 * ----------------------------------------------------------------------------
 * 裸机环境没有文件系统/进程，newlib 的 _read/_write/_lseek 等必须提供桩，
 * 否则链接会失败。Pinecil 系列已在别处提供真实实现（调试串口），因此这里
 * 用条件编译跳过；其它机型统一返回 -1（表示"不支持"）。
 * 注意：_write 返回 -1 会让 printf 静默失败，调试输出请用 BSP 的
 * log_system_state()/DEBUG_UART_OUTPUT 通道。
 * ============================================================================
 */

/* Functions */
void initialise_monitor_handles() {}

/* Syscalls (stub implementations to avoid compile warnings and possibe future problems) */
int _getpid(void) { return 1; }

#if defined(MODEL_Pinecil) || defined(MODEL_Pinecilv2)
// do nothing here because some stubs and real implementations added for Pinecils already
#else
off_t   _lseek(int fd, off_t ptr, int dir) { return -1; }
ssize_t _read(int fd, void *ptr, size_t len) { return -1; }
ssize_t _write(int fd, const void *ptr, size_t len) { return -1; }
int     _close(int fd) { return -1; }
#endif
