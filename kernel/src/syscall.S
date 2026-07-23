.section .rodata.syscall_table, "a"
.p2align 3
.globl syscall_table
.type syscall_table, @object
syscall_table:
    .quad syscall_process_exit
    .quad syscall_thread_set_pointer
    .quad syscall_futex_wait
    .quad syscall_futex_wake
    .quad syscall_file_open
    .quad syscall_file_close
    .quad syscall_file_read
    .quad syscall_file_write
    .quad syscall_file_seek
    .quad syscall_clock_get
    .quad syscall_memory_map
    .quad syscall_memory_unmap
    .quad syscall_file_ioctl
    .quad syscall_file_dup
    .quad syscall_file_dup2
    .quad syscall_process_fork
    .quad syscall_process_exec
    .quad syscall_process_wait
    .quad syscall_process_getpid
    .quad syscall_process_getppid
    .quad syscall_process_getpgid
    .quad syscall_process_setpgid
    .quad syscall_process_getsid
    .quad syscall_process_setsid
    .quad syscall_process_kill
    .quad syscall_file_chdir
    .quad syscall_file_getcwd
    .quad syscall_file_access
    .quad syscall_file_stat
    .quad syscall_file_fstat
    .quad syscall_file_fcntl
    .quad syscall_file_pipe
    .quad syscall_file_readdir
    .quad syscall_file_readlink
    .quad syscall_file_unlink
    .quad syscall_clock_sleep
    .quad syscall_file_mkdir
    .quad syscall_process_gettid
    .quad syscall_file_fchmod
    .quad syscall_file_rename
    .quad syscall_file_poll
    .quad syscall_system_uname
    .quad syscall_signal_action
    .quad syscall_signal_mask
    .quad syscall_signal_return
    .quad syscall_signal_fd
    .quad syscall_file_openat
    .quad syscall_file_accessat
    .quad syscall_file_statat
    .quad syscall_file_readlinkat
    .quad syscall_file_unlinkat
    .quad syscall_file_mkdirat
    .quad syscall_file_fchmodat
    .quad syscall_file_renameat
    .quad syscall_file_linkat
    .quad syscall_file_symlinkat
.size syscall_table, . - syscall_table

.globl syscall_table_count
.type syscall_table_count, @object
syscall_table_count:
    .quad (. - syscall_table) / 8
.size syscall_table_count, . - syscall_table_count
