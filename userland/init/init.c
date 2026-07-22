#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

static void clear_terminal(void) {
    printf("\033[2J\033[H");
    fflush(stdout);
}

static int create_tmp(void) {
    mode_t mode = S_IRWXU | S_IRWXG | S_IRWXO | S_ISVTX;
    if (mkdir("/tmp", mode) == 0)
        return 0;
    if (errno != EEXIST)
        return -1;

    struct stat status;
    if (stat("/tmp", &status) < 0)
        return -1;
    if (!S_ISDIR(status.st_mode)) {
        errno = ENOTDIR;
        return -1;
    }
    return 0;
}

static int open_console(void) {
    int serial = open("/dev/ttyS0", O_RDWR);
    if (serial < 0)
        return -1;

    for (int fd = STDIN_FILENO; fd <= STDERR_FILENO; fd++) {
        if (serial == fd)
            continue;
        if (dup2(serial, fd) < 0) {
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
    if (create_tmp() < 0)
        return 1;
    if (open_console() < 0)
        return 1;
    if (setenv("TERM", "vt220", 0) < 0)
        return 1;

    clear_terminal();
    printf("Welcome to roanix!\n");
    fflush(stdout);

    for (;;) {
        run_shell();
        clear_terminal();
    }
}
