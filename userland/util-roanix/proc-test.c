// Process lifecycle smoke test.
//
// Narrow companion to pthread-test, aimed at fork/exit/wait rather than
// threads. Everything reports through write(2) so a stuck stdio lock cannot
// hide the progress markers.

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

static void say(const char *text) {
    write(STDOUT_FILENO, text, strlen(text));
}

static void say_number(long value) {
    char buffer[24];
    int index = sizeof(buffer);
    int negative = value < 0;
    unsigned long magnitude = negative ? (unsigned long)-value : (unsigned long)value;

    buffer[--index] = '\n';
    if (!magnitude)
        buffer[--index] = '0';
    while (magnitude) {
        buffer[--index] = '0' + (char)(magnitude % 10);
        magnitude /= 10;
    }
    if (negative)
        buffer[--index] = '-';
    write(STDOUT_FILENO, &buffer[index], sizeof(buffer) - index);
}

// Each case forks and lets the child leave in a different way, so a hang
// identifies which exit path is at fault.
static int run_case(const char *name, int mode) {
    say("-- ");
    say(name);
    say("\n");

    pid_t child = fork();
    if (child < 0) {
        say("   fork failed\n");
        return 1;
    }
    if (child == 0) {
        switch (mode) {
            case 0:
                _exit(7);
            case 1:
                exit(7);
            case 2:
                execl("/usr/bin/true", "true", (char *)NULL);
                _exit(127);
            default:
                execl("/nonexistent-on-purpose", "x", (char *)NULL);
                _exit(127);
        }
    }

    say("   forked child ");
    say_number(child);

    int status = 0;
    pid_t reaped = waitpid(child, &status, 0);
    if (reaped != child) {
        say("   waitpid failed, errno ");
        say_number(errno);
        return 1;
    }
    say("   reaped, exit status ");
    say_number(WIFEXITED(status) ? WEXITSTATUS(status) : -1);
    return 0;
}

int main(void) {
    say("proc smoke test\n");

    int failures = 0;
    failures += run_case("fork + _exit (no libc cleanup)", 0);
    failures += run_case("fork + exit (libc cleanup)", 1);
    failures += run_case("fork + successful exec", 2);
    failures += run_case("fork + failed exec", 3);

    say(failures ? "proc smoke test FAILED\n" : "proc smoke test passed\n");
    return failures ? 1 : 0;
}
