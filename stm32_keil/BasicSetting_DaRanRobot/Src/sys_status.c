/**
 ******************************************************************************
 * @file    sys_status.c
 * @brief   系统运行状态管理实现
 ******************************************************************************
 */

#include "app_threads.h"
#include "sys_status.h"
#include <rtthread.h>

static const char *state_name(rt_uint8_t st)
{
    switch (st) {
    case SYS_STATE_INIT:        return "INIT";
    case SYS_STATE_READY:       return "READY";
    case SYS_STATE_RUNNING:     return "RUNNING";
    case SYS_STATE_ESTOP:       return "ESTOP";
    case SYS_STATE_ERROR:       return "ERROR";
    case SYS_STATE_TEACH:       return "TEACH";
    case SYS_STATE_ZERO_CHECK:  return "ZERO_CHK";
    case SYS_STATE_CALIBRATION: return "CALIB";
    default:                    return "?";
    }
}

void sys_mark_ready(uint16_t module_bit)
{
    g_sys.modules_ready |= module_bit;
}

void sys_cmd_inc(void)
{
    g_sys.cmd_count++;
}

void sys_dump_status(void)
{
    rt_kprintf("\n[SYS] Uptime=%lums Cmds=%lu State=%s Err=%d",
               g_sys.uptime_ms, g_sys.cmd_count,
               state_name(g_sys.state),
               g_sys.error_code);
    if (g_sys.estop_triggered)
        rt_kprintf(" ESTOP!");

    rt_kprintf("\n[SYS] Modules: USART2=%c USART1=%c CAN=%c SERVO=%c PROTO=%c"
               " KIN=%c TRAJ=%c JOINTS=%c\n",
               (g_sys.modules_ready & SYS_MOD_USART2)     ? 'Y' : '-',
               (g_sys.modules_ready & SYS_MOD_USART1)     ? 'Y' : '-',
               (g_sys.modules_ready & SYS_MOD_CAN)        ? 'Y' : '-',
               (g_sys.modules_ready & SYS_MOD_SERVO)      ? 'Y' : '-',
               (g_sys.modules_ready & SYS_MOD_PROTOCOL)   ? 'Y' : '-',
               (g_sys.modules_ready & SYS_MOD_KINEMATICS) ? 'Y' : '-',
               (g_sys.modules_ready & SYS_MOD_TRAJECTORY) ? 'Y' : '-',
               (g_sys.modules_ready & SYS_MOD_JOINTS)     ? 'Y' : '-');
}
