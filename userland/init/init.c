#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/wait.h>
#include <unistd.h>

#include <roanix/syscall.h>

enum {
    INIT_SYS_FILE_DUP2 = 14
};

#ifdef ROANIX_SYS_FILE_DUP2
_Static_assert(
    ROANIX_SYS_FILE_DUP2 == INIT_SYS_FILE_DUP2,
    "init and kernel disagree on the dup2 syscall number"
);
#endif

static void clear_terminal(void) {
    printf("\033[2J\033[H");
    fflush(stdout);
}

static int open_console(void) {
    int serial = open("/dev/ttyS0", O_RDWR);
    if (serial < 0)
        serial = open("/dev/ttys0", O_RDWR);
    if (serial < 0)
        return -1;

    for (int fd = STDIN_FILENO; fd <= STDERR_FILENO; fd++) {
        if (serial == fd)
            continue;
        long result = roanix_syscall3(INIT_SYS_FILE_DUP2, serial, fd, 0);
        if (result < 0) {
            errno = (int)-result;
            if (serial > STDERR_FILENO)
                close(serial);
            return -1;
        }
    }

    if (serial > STDERR_FILENO)
        close(serial);
    return 0;
}

static void run_shell(void) {
    pid_t child = fork();
    if (child < 0) {
        perror("init: fork");
        sleep(1);
        return;
    }

    if (child == 0) {
        execl("/usr/bin/bash", "bash", (char *)NULL);
        perror("init: exec /usr/bin/bash");
        _exit(127);
    }

    for (;;) {
        if (waitpid(child, NULL, 0) >= 0)
            return;
        if (errno != EINTR) {
            perror("init: waitpid");
            return;
        }
    }
}

int main(void) {
    if (open_console() < 0)
        return 1;

    clear_terminal();
    printf("Welcome to roanix!\n");
    fflush(stdout);

    for (;;) {
        run_shell();
        clear_terminal();
    }
}
