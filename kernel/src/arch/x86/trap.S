.section .text

.extern rsyscall_return
.extern syscall_not_implemented
.extern syscall_table
.extern syscall_table_count
.extern rtrap
.globl rsyscall_entry
.globl rtrap_entry
.globl rthread_resume
.globl vstub0

rsyscall_entry:
    /* Perform a swapgs and switch to the kernel stack. */
    swapgs
    movq %rsp, %gs:8
    movq %gs:0, %rsp

    /* Create an iret-compatible user trap frame header. */
    pushq $0x43
    pushq %gs:8
    pushq %r11
    pushq $0x3b
    pushq %rcx
    pushq $0
    pushq $0

    /* Push the remaining registers. */
    push %r15
    push %r14
    push %r13
    push %r12
    push %r11
    push %r10
    push %r9
    push %r8
    push %rbp
    push %rdi
    push %rsi
    push %rdx
    push %rcx
    push %rbx
    push %rax
    cld
    sti

    /* Index directly into the syscall handler table. */
    mov 0(%rsp), %rax
    mov %rsp, %rdi
    xor %rbp, %rbp
    cmp syscall_table_count(%rip), %rax
    jae .Lunknown_syscall
    lea syscall_table(%rip), %rcx
    call *(%rcx,%rax,8)
    jmp .Lfinish_syscall

.Lunknown_syscall:
    call syscall_not_implemented

.Lfinish_syscall:
    mov %rax, 0(%rsp)
    mov %rsp, %rdi
    call rsyscall_return
    mov %rax, %rsp

.Lrestore_syscall:
    /* Pop the trapframe back into place. */
    pop %rax
    pop %rbx
    pop %rcx
    pop %rdx
    pop %rsi
    pop %rdi
    pop %rbp
    pop %r8
    pop %r9
    pop %r10
    pop %r11
    pop %r12
    pop %r13
    pop %r14
    pop %r15
    add $16, %rsp

    /* Restore the remaining context and return through the full user frame. */
    cli
    cmpq $0x3b, 8(%rsp)
    jne 1f
    swapgs
1:
    iretq

rtrap_entry:
    /* Perform a swapgs (if we came from usermode). */
    cmpq $0x3b, 24(%rsp)
    jne 1f
    swapgs

    /* Push the trapframe to the stack. */
1:
    push %r15
    push %r14
    push %r13
    push %r12
    push %r11
    push %r10
    push %r9
    push %r8
    push %rbp
    push %rdi
    push %rsi
    push %rdx
    push %rcx
    push %rbx
    push %rax
    cld

    /* Setup C enviroment and index into the handler. */
    mov %rsp, %rdi
    xor %rbp, %rbp
    call rtrap
    mov %rax, %rsp

.Lrestore_trap:
    /* Pop the trapframe back into place. */
    pop %rax
    pop %rbx
    pop %rcx
    pop %rdx
    pop %rsi
    pop %rdi
    pop %rbp
    pop %r8
    pop %r9
    pop %r10
    pop %r11
    pop %r12
    pop %r13
    pop %r14
    pop %r15
    add $16, %rsp

    /* Swap back to user gs (if needed). */
    cmpq $0x3b, 8(%rsp)
    jne 1f
    swapgs

    /* And away we go :-) */
1:
    iretq

rthread_resume:
    mov %rdi, %rsp
    jmp .Lrestore_trap

.altmacro
.macro make_stubs start_vec, end_vec, has_ec=1
    .align 0x10
    vstub\start_vec:
    .if \has_ec
        push $0x0
    .endif
    push $\start_vec
    jmp rtrap_entry
    .if \end_vec-\start_vec
        make_stubs %\start_vec+1, \end_vec, \has_ec
    .endif
.endm

/* Limit to 20 recursions, which is the maximum for both GCC and Clang */
make_stubs 0, 7
make_stubs 8, 8, 0
make_stubs 9, 9
make_stubs 10, 14, 0
make_stubs 15, 16
make_stubs 17, 17, 0
make_stubs 18, 20
make_stubs 21, 21, 0
make_stubs 22, 41
make_stubs 42, 61
make_stubs 62, 81
make_stubs 82, 101
make_stubs 102, 121
make_stubs 122, 141
make_stubs 142, 161
make_stubs 162, 181
make_stubs 182, 201
make_stubs 202, 221
make_stubs 222, 241
make_stubs 242, 255
