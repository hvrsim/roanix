#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <termios.h>
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

static int parse_cursor_position(
        const char *response,
        size_t length,
        unsigned short *rows,
        unsigned short *columns) {
    for (size_t start = 0; start + 4 < length; start++) {
        if (response[start] != '\033' || response[start + 1] != '[')
            continue;

        size_t position = start + 2;
        unsigned row = 0;
        unsigned column = 0;
        while (position < length &&
                response[position] >= '0' &&
                response[position] <= '9') {
            row = row * 10 + (unsigned)(response[position++] - '0');
        }
        if (position >= length || response[position++] != ';')
            continue;
        while (position < length &&
                response[position] >= '0' &&
                response[position] <= '9') {
            column = column * 10 + (unsigned)(response[position++] - '0');
        }
        if (position < length &&
                response[position] == 'R' &&
                row > 0 && row <= UINT16_MAX &&
                column > 0 && column <= UINT16_MAX) {
            *rows = (unsigned short)row;
            *columns = (unsigned short)column;
            return 0;
        }
    }
    return -1;
}

static void publish_terminal_size(unsigned short rows, unsigned short columns) {
    struct winsize size = {
        .ws_row = rows,
        .ws_col = columns,
    };
    (void)tcsetwinsize(STDIN_FILENO, &size);

    char value[16];
    snprintf(value, sizeof(value), "%hu", columns);
    (void)setenv("COLUMNS", value, 1);
    snprintf(value, sizeof(value), "%hu", rows);
    (void)setenv("LINES", value, 1);
}

static void detect_terminal_size(void) {
    unsigned short rows = 24;
    unsigned short columns = 80;
    publish_terminal_size(rows, columns);

    struct termios saved;
    if (tcgetattr(STDIN_FILENO, &saved) < 0)
        return;

    struct termios query = saved;
    query.c_iflag &= ~(ICRNL | IXON);
    query.c_lflag &= ~(ICANON | ECHO);
    query.c_cc[VMIN] = 0;
    query.c_cc[VTIME] = 2;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &query) < 0)
        return;

    static const char request[] = "\0337\033[999;999H\033[6n";
    static const char restore[] = "\0338";
    (void)write(STDOUT_FILENO, request, sizeof(request) - 1);

    char response[32];
    size_t length = 0;
    while (length < sizeof(response)) {
        ssize_t count = read(
            STDIN_FILENO,
            response + length,
            sizeof(response) - length);
        if (count <= 0)
            break;
        length += (size_t)count;
        if (memchr(response, 'R', length) != NULL)
            break;
    }

    (void)write(STDOUT_FILENO, restore, sizeof(restore) - 1);
    (void)tcsetattr(STDIN_FILENO, TCSANOW, &saved);
    if (parse_cursor_position(response, length, &rows, &columns) == 0)
        publish_terminal_size(rows, columns);
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

    detect_terminal_size();
    clear_terminal();
    printf("Welcome to roanix!\n");
    fflush(stdout);

    for (;;) {
        run_shell();
        clear_terminal();
    }
}
