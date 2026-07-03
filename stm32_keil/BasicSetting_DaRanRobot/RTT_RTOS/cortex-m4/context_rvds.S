;******************************************************************************
; Copyright (c) 2006-2022, RT-Thread Development Team
; SPDX-License-Identifier: Apache-2.0
;
; Cortex-M4 上下文切换与异常处理汇编（Keil MDK-ARM / ARMCC）
; 适配 YaowenLi STM32F407 平台
;******************************************************************************

    AREA |.text|, CODE, READONLY, ALIGN=2
    THUMB
    PRESERVE8

    IMPORT rt_thread_switch_interrupt_flag
    IMPORT rt_interrupt_from_thread
    IMPORT rt_interrupt_to_thread
    IMPORT rt_hw_hard_fault_exception

;------------------------------------------------------------------------------
; void rt_hw_context_switch_to(rt_uint32_t to)
;   Sets PendSV pending, then triggers SVC. The SVC handler returns, causing
;   an exception return. Since PendSV is pending and has the lowest priority,
;   it fires immediately (tail-chaining) and PendSV_Handler performs the
;   context switch.
;------------------------------------------------------------------------------
    EXPORT rt_hw_context_switch_to
rt_hw_context_switch_to
    ; r0 = &to_thread->sp
    LDR     r1, =rt_interrupt_to_thread
    STR     r0, [r1]
    LDR     r1, =rt_interrupt_from_thread
    MOV     r2, #0
    STR     r2, [r1]
    LDR     r1, =rt_thread_switch_interrupt_flag
    MOV     r2, #1
    STR     r2, [r1]

    ; Trigger PendSV
    LDR     r0, =0xE000ED04
    LDR     r1, =0x10000000
    STR     r1, [r0]

    ; Trigger SVC — SVC handler returns → PendSV fires
    SVC     0

    ; ★ 如果走到这里 = PendSV 没触发!! LED2 交替亮灭报错 ★
    LDR     r0, =0x40020000           ; GPIOA base = 0x40020000
    LDR     r1, =0x000000C0           ; PA6+PA7 mask
    LDR     r2, =0x00C00000           ; PA6+PA7 reset mask
fault_loop
    STR     r2, [r0, #0x18]           ; BSRR reset → PA6+PA7 LOW → 双灯亮
    LDR     r3, =1000000
fault_dly1
    SUBS    r3, r3, #1
    BNE     fault_dly1
    STR     r1, [r0, #0x18]           ; BSRR set → PA6+PA7 HIGH → 双灯灭
    LDR     r3, =1000000
fault_dly2
    SUBS    r3, r3, #1
    BNE     fault_dly2
    B       fault_loop

;------------------------------------------------------------------------------
; void rt_hw_context_switch(rt_uint32_t from, rt_uint32_t to)
;   Called from thread context.  Trigger PendSV so the switch happens on the
;   next exception return (SysTick exit or similar).
;------------------------------------------------------------------------------
    EXPORT rt_hw_context_switch
rt_hw_context_switch
    LDR     r2, =rt_thread_switch_interrupt_flag
    LDR     r3, [r2]
    CMP     r3, #1
    BEQ     _reswitch
    MOV     r3, #1
    STR     r3, [r2]
    LDR     r2, =rt_interrupt_from_thread
    STR     r0, [r2]
_reswitch
    LDR     r2, =rt_interrupt_to_thread
    STR     r1, [r2]

    ; ★ Trigger PendSV — was missing! ★
    LDR     r0, =0xE000ED04
    LDR     r1, =0x10000000
    STR     r1, [r0]
    BX      lr

;------------------------------------------------------------------------------
; rt_base_t rt_hw_interrupt_disable(void)
;------------------------------------------------------------------------------
    EXPORT rt_hw_interrupt_disable
rt_hw_interrupt_disable
    MRS     r0, PRIMASK
    CPSID   I
    BX      lr

;------------------------------------------------------------------------------
; void rt_hw_interrupt_enable(rt_base_t level)
;------------------------------------------------------------------------------
    EXPORT rt_hw_interrupt_enable
rt_hw_interrupt_enable
    MSR     PRIMASK, r0
    BX      lr

;------------------------------------------------------------------------------
; void rt_hw_context_switch_interrupt(rt_ubase_t from, rt_ubase_t to)
;   Called from ISR context.  Trigger PendSV so it fires on ISR exit.
;------------------------------------------------------------------------------
    EXPORT rt_hw_context_switch_interrupt
rt_hw_context_switch_interrupt
    LDR     r2, =rt_thread_switch_interrupt_flag
    LDR     r3, [r2]
    CMP     r3, #1
    BEQ     _reswitch_int
    MOV     r3, #1
    STR     r3, [r2]
    LDR     r2, =rt_interrupt_from_thread
    STR     r0, [r2]
_reswitch_int
    LDR     r2, =rt_interrupt_to_thread
    STR     r1, [r2]

    ; ★ Trigger PendSV — was missing! ★
    LDR     r0, =0xE000ED04
    LDR     r1, =0x10000000
    STR     r1, [r0]
    BX      lr

;------------------------------------------------------------------------------
; void PendSV_Handler(void)
;------------------------------------------------------------------------------
    EXPORT PendSV_Handler
PendSV_Handler
    MRS     r2, PRIMASK
    CPSID   I

    LDR     r0, =rt_thread_switch_interrupt_flag
    LDR     r1, [r0]
    CBZ     r1, pendsv_exit

    MOV     r1, #0
    STR     r1, [r0]

    LDR     r0, =rt_interrupt_from_thread
    LDR     r1, [r0]
    CBZ     r1, switch_to_thread

    MRS     r1, PSP

    TST     lr, #0x10
    IT      EQ
    VSTMDBEQ r1!, {s16 - s31}

    STMDB   r1!, {r4 - r11}

    TST     lr, #0x10
    IT      EQ
    MOVEQ   r4, #0x00
    ITT     NE
    MOVNE   r4, #0x01
    VSTMNE  r1!, {s0 - s15}

    SUB     r1, r1, #4
    STR     r4, [r1]

    LDR     r0, =rt_interrupt_from_thread
    LDR     r0, [r0]
    STR     r1, [r0]

switch_to_thread
    LDR     r1, =rt_interrupt_to_thread
    LDR     r1, [r1]
    LDR     r1, [r1]

    LDR     r3, [r1]              ; r3 = flag (保存在 r3, 不会被 LDMIA 覆盖!)
    ADD     r1, r1, #4

    TST     r3, #0x01             ; 测试 FLAG (非 r4!)
    IT      NE
    VLDMNE  r1!, {s0 - s15}       ; flag=1: 恢复 s0-s15

    LDMIA   r1!, {r4 - r11}       ; 恢复 r4-r11 (r4 被覆盖, 但 flag 在 r3)

    TST     r3, #0x01             ; 测试 FLAG (仍在 r3 中)
    IT      EQ
    VLDMIAEQ r1!, {s16 - s31}     ; flag=0: 恢复 s16-s31

    MSR     PSP, r1

    TST     r3, #0x01             ; 测试 FLAG 决定 EXC_RETURN
    ITE     EQ
    MOVEQ   lr, #0xFFFFFFED       ; flag=0 → FPU 活跃
    MOVNE   lr, #0xFFFFFFFD       ; flag=1 → FPU 未用
    ORR     lr, lr, #0x04

pendsv_exit
    MSR     PRIMASK, r2
    BX      lr

;------------------------------------------------------------------------------
; void HardFault_Handler(void)
;------------------------------------------------------------------------------
    EXPORT HardFault_Handler
HardFault_Handler
    TST     lr, #0x04
    ITE     EQ
    MRSEQ   r0, MSP
    MRSNE   r0, PSP
    B       rt_hw_hard_fault_exception

    NOP
    END
