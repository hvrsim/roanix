#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#define ROOT_DEVICE "/dev/nvme0n1p1"
#define ROOT_FILESYSTEM "ext4"
#define ROOT_READY "ROANIX_ROOT_READY"

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
    /* Leave ownership to the console service once the real manager starts. */
    int serial = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
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

static int mount_system_root(void) {
    if (getenv(ROOT_READY) != NULL)
        return 0;

    for (unsigned attempt = 0; attempt < 10; attempt++) {
        if (mount(ROOT_DEVICE, "/", ROOT_FILESYSTEM, MS_RDONLY, NULL) == 0) {
            if (setenv(ROOT_READY, "1", 1) < 0)
                return -1;
            if (chdir("/") < 0)
                return -1;
            execl("/sbin/init", "/sbin/init", (char *)NULL);
            return -1;
        }

        if (errno != ENOENT && errno != ENODEV && errno != EIO)
            break;
        sleep(1);
    }

    return -1;
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

static pid_t spawn_console(int graphical) {
    pid_t child = fork();
    if (child < 0) {
        perror("init: fork");
        return -1;
    }

    if (child == 0) {
        if (graphical) {
            if (setsid() < 0) {
                perror("init: graphical session");
                _exit(1);
            }
            execl("/usr/bin/kmscon", "kmscon", "--no-reset-env", "--session-control",
                  "--login", "--", "/usr/bin/bash", (char *)NULL);
            perror("init: exec /usr/bin/kmscon");
            _exit(127);
        }
        execl("/usr/bin/bash", "bash", (char *)NULL);
        perror("init: exec /usr/bin/bash");
        _exit(127);
    }

    return child;
}

int main(void) {
    int bootstrapping = getenv(ROOT_READY) == NULL;
    if (bootstrapping && create_tmp() < 0) {
        perror("init: create /tmp");
        return 1;
    }
    if (bootstrapping && open_console() < 0) {
        perror("init: open console");
        return 1;
    }
    if (setenv("TERM", "vt220", 0) < 0)
        return 1;

    if (bootstrapping && mount_system_root() < 0) {
        fprintf(stderr, "init: cannot mount %s as root: %s\n",
                ROOT_DEVICE, strerror(errno));
        fprintf(stderr, "init: continuing with the emergency initramfs\n");
    }

    if (getenv(ROOT_READY) != NULL) {
        execl("/usr/lib/systemd/systemd", "systemd", (char *)NULL);
        fprintf(stderr, "init: cannot start systemd: %s\n", strerror(errno));
        fprintf(stderr, "init: continuing with the emergency console\n");
    }

    /* Only the emergency supervisor owns the terminal directly. */
    (void)ioctl(STDIN_FILENO, TIOCSCTTY, 0);
    detect_terminal_size();
    clear_terminal();
    printf("Welcome to roanix!\n");
    fflush(stdout);

    int graphical = access("/usr/bin/kmscon", X_OK) == 0
        && access("/dev/dri/card0", R_OK | W_OK) == 0;
    pid_t serial = -1, display = -1;
    for (;;) {
        if (serial < 0)
            serial = spawn_console(0);
        if (graphical && display < 0)
            display = spawn_console(1);
        if (serial < 0 || (graphical && display < 0)) {
            sleep(1);
            continue;
        }
        int status;
        pid_t child = waitpid(-1, &status, 0);
        if (child == serial) {
            serial = -1;
            clear_terminal();
        } else if (graphical && child == display) {
            display = -1;
            fprintf(stderr, "init: kmscon exited (status %d), restarting\n", status);
            sleep(1);
        } else if (child < 0 && errno != EINTR) {
            perror("init: waitpid");
            sleep(1);
        }
    }
}
