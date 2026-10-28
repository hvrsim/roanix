.section .text

.extern rtrap
.extern rmain
.extern rsyscall_return
.extern rsignal_return
.extern syscall_not_implemented
.extern syscall_table
.extern syscall_table_count

.globl rv64_start
.globl rthread_resume
rv64_start:
.option push
.option norelax
    la gp, __global_pointer$
.option pop

    li t0, 0x6000ull
    csrrc x0, sstatus, t0
    csrw sscratch, zero

    call rmain

    /* halt and catch fire (just kidding, no fire :^) */
    csrci sstatus, 0x2
.hcf:
    wfi
    j .hcf

.globl rtrap_entry
.align 4
rtrap_entry:
    csrrw tp, sscratch, tp
    beqz tp, .in_kmode

.in_umode:
    sd sp, 8(tp)
    ld sp, 0(tp)
    j .save_context

.in_kmode:
    csrr tp, sscratch
    sd sp, 8(tp)

.save_context:
    addi sp, sp, -560 /* size_of::<TrapFrame>() aligned to 16-bytes */

    /* store the general purpose registers */
    sd a0, 0(sp)
    sd a1, 8(sp)
    sd a2, 16(sp)
    sd a3, 24(sp)
    sd a4, 32(sp)
    sd a5, 40(sp)
    sd a6, 48(sp)
    sd a7, 56(sp)
    sd t0, 64(sp)
    sd t1, 72(sp)
    sd t2, 80(sp)
    sd t3, 88(sp)
    sd t4, 96(sp)
    sd t5, 104(sp)
    sd t6, 112(sp)
    sd s0, 120(sp)
    sd s1, 128(sp)
    sd s2, 136(sp)
    sd s3, 144(sp)
    sd s4, 152(sp)
    sd s5, 160(sp)
    sd s6, 168(sp)
    sd s7, 176(sp)
    sd s8, 184(sp)
    sd s9, 192(sp)
    sd s10, 200(sp)
    sd s11, 208(sp)

    /* store the aux registers */
    sd ra, 216(sp)
    sd gp, 224(sp)
    ld a0, 8(tp)
    sd a0, 232(sp)
    csrr a0, sscratch
    sd a0, 240(sp)

    /* Eagerly retain user FP state before disabling FS. Kernel traps have
     * FS=Off; initialize their unused slots so TrapFrame is fully defined. */
    sd zero, 280(sp)
    sd zero, 552(sp)
    csrr t0, sstatus
    li t1, 0x6000
    and t0, t0, t1
    beqz t0, .zero_fpu
.option push
.option arch, +f, +d
    fsd f0, 288(sp)
    fsd f1, 296(sp)
    fsd f2, 304(sp)
    fsd f3, 312(sp)
    fsd f4, 320(sp)
    fsd f5, 328(sp)
    fsd f6, 336(sp)
    fsd f7, 344(sp)
    fsd f8, 352(sp)
    fsd f9, 360(sp)
    fsd f10, 368(sp)
    fsd f11, 376(sp)
    fsd f12, 384(sp)
    fsd f13, 392(sp)
    fsd f14, 400(sp)
    fsd f15, 408(sp)
    fsd f16, 416(sp)
    fsd f17, 424(sp)
    fsd f18, 432(sp)
    fsd f19, 440(sp)
    fsd f20, 448(sp)
    fsd f21, 456(sp)
    fsd f22, 464(sp)
    fsd f23, 472(sp)
    fsd f24, 480(sp)
    fsd f25, 488(sp)
    fsd f26, 496(sp)
    fsd f27, 504(sp)
    fsd f28, 512(sp)
    fsd f29, 520(sp)
    fsd f30, 528(sp)
    fsd f31, 536(sp)
    frcsr t0
    sd t0, 544(sp)
.option pop
    j .fpu_saved
.zero_fpu:
    addi t0, sp, 288
    addi t1, sp, 552
1:
    sd zero, 0(t0)
    addi t0, t0, 8
    bne t0, t1, 1b
.fpu_saved:

    /* disable FPU and clear sscratch for nested interrupts */
    li t0, 0x6000ull
    csrw sscratch, zero

    /* store the CSRs with information on the exception */
    csrr a0, scause
    sd a0, 248(sp)
    csrr a0, stval
    csrr a1, sepc
    csrrc a2, sstatus, t0
    sd a0, 256(sp)
    sd a1, 264(sp)
    sd a2, 272(sp)

    /* set the kernel's global pointer */
.option push
.option norelax
    la gp, __global_pointer$
.option pop

    /* Dispatch user ecalls directly through the syscall table. */
    ld t0, 248(sp)
    li t1, 8
    bne t0, t1, .handle_trap
    ld t0, 272(sp)
    andi t0, t0, 0x100
    bnez t0, .handle_trap

    csrsi sstatus, 0x2
    ld t0, 264(sp)
    addi t0, t0, 4
    sd t0, 264(sp)

    ld t0, 56(sp)
    la t1, syscall_table_count
    ld t1, 0(t1)
    bgeu t0, t1, .unknown_syscall
    la t1, syscall_table
    slli t0, t0, 3
    add t1, t1, t0
    ld t1, 0(t1)
    j .call_syscall

.unknown_syscall:
    la t1, syscall_not_implemented

.call_syscall:
    mv a0, sp
    jalr t1
    sd a0, 0(sp)
    csrci sstatus, 0x2
    mv a0, sp
    call rsyscall_return
    /* Signal delivery may block; use the selected thread's stack. */
    mv sp, a0
    call rsignal_return
    mv sp, a0
    j .prepare_return

    /* Set the context and jump to the generic trap handler. */
.handle_trap:
    mv a0, sp
    call rtrap
    /* The outgoing thread may already be running on another hart. */
    mv sp, a0
    call rsignal_return
    mv sp, a0

    /*
     *  disable interrupts and set the kernel thread-local
     *  info in sscratch if returning to umode
     */
.prepare_return:
    csrci sstatus, 0x2
    ld a1, 272(sp)
    andi  a1, a1, 0x100
    bnez  a1, .return_to_kernel
    csrw sscratch, tp
    j .restore_context

.return_to_kernel:
    csrw sscratch, zero

.restore_context:
    /* FS is Off throughout kernel code. Enable it only while restoring the
     * selected user frame, then restore the frame's sanitized status below. */
    ld t0, 272(sp)
    li t1, 0x6000
    and t0, t0, t1
    beqz t0, .fpu_restored
    csrs sstatus, t1
.option push
.option arch, +f, +d
    fld f0, 288(sp)
    fld f1, 296(sp)
    fld f2, 304(sp)
    fld f3, 312(sp)
    fld f4, 320(sp)
    fld f5, 328(sp)
    fld f6, 336(sp)
    fld f7, 344(sp)
    fld f8, 352(sp)
    fld f9, 360(sp)
    fld f10, 368(sp)
    fld f11, 376(sp)
    fld f12, 384(sp)
    fld f13, 392(sp)
    fld f14, 400(sp)
    fld f15, 408(sp)
    fld f16, 416(sp)
    fld f17, 424(sp)
    fld f18, 432(sp)
    fld f19, 440(sp)
    fld f20, 448(sp)
    fld f21, 456(sp)
    fld f22, 464(sp)
    fld f23, 472(sp)
    fld f24, 480(sp)
    fld f25, 488(sp)
    fld f26, 496(sp)
    fld f27, 504(sp)
    fld f28, 512(sp)
    fld f29, 520(sp)
    fld f30, 528(sp)
    fld f31, 536(sp)
    ld t0, 544(sp)
    fscsr t0
.option pop
.fpu_restored:
    /* restore the general purpose registers */
    ld a1, 8(sp)
    ld a2, 16(sp)
    ld a3, 24(sp)
    ld a4, 32(sp)
    ld a5, 40(sp)
    ld a6, 48(sp)
    ld a7, 56(sp)
    ld t0, 64(sp)
    ld t1, 72(sp)
    ld t2, 80(sp)
    ld t3, 88(sp)
    ld t4, 96(sp)
    ld t5, 104(sp)
    ld t6, 112(sp)
    ld s0, 120(sp)
    ld s1, 128(sp)
    ld s2, 136(sp)
    ld s3, 144(sp)
    ld s4, 152(sp)
    ld s5, 160(sp)
    ld s6, 168(sp)
    ld s7, 176(sp)
    ld s8, 184(sp)
    ld s9, 192(sp)
    ld s10, 200(sp)
    ld s11, 208(sp)

    /* restore some of the aux registers */
    ld ra, 216(sp)
    ld gp, 224(sp)

    /* restore CSRs */
    ld a0, 248(sp)
    csrw scause, a0
    ld a0, 256(sp)
    csrw stval, a0
    ld a0, 264(sp)
    csrw sepc, a0
    ld a0, 272(sp)
    csrw sstatus, a0

    /* restore a0, tp & sp then off we go! */
    ld a0, 0(sp)
    ld tp, 240(sp)
    ld sp, 232(sp)

    sret

rthread_resume:
    mv sp, a0
    j .prepare_return
