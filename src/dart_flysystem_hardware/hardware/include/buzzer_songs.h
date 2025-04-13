#ifndef BUZZER_SONGS_H
#define BUZZER_SONGS_H
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/time.h>
#include <linux/input.h>
#include "buzzer_tones.h"

// Disable Autopilot Beep 音符序列
static note_t buzzer_autopilot_disconnect[] = {
    {NOTE_GS6, 8 * 7},   // 1661Hz, 56ms
    {NOTE_GS5, 7 * 7},   //  831Hz, 49ms
    {0,        4 * 7},   // 静音 28ms
    {NOTE_GS6, 15 * 7},  // 1661Hz, 105ms
    {0,        15 * 7},  // 静音 105ms
    {NOTE_GS6, 8 * 7},   // 1661Hz, 56ms
    {NOTE_GS5, 7 * 7},   //  831Hz, 49ms
    {0,        4 * 7},   // 静音 28ms
    {NOTE_GS6, 15 * 7},  // 1661Hz, 105ms
    {0,        15 * 7},  // 静音 105ms
    {NOTE_GS6, 8 * 7},   // 1661Hz, 56ms
    {NOTE_GS5, 7 * 7},   //  831Hz, 49ms
    {0,        4 * 7},   // 静音 28ms
    {NOTE_GS6, 15 * 7},  // 1661Hz, 105ms
    {0,        45 * 7}   // 静音 315ms
};

#define play_songs(song) do { \
    const char *device = "/dev/input/by-path/platform-pwm-beeper-event"; \
    int fd = ::open(device, O_WRONLY); \
    if (fd < 0) { \
        perror("打开设备失败"); \
        exit(EXIT_FAILURE); \
    } \
    int noteCount = sizeof(song) / sizeof(note_t); \
    struct input_event ev; \
    for (int i = 0; i < noteCount; ++i) { \
        note_t note = (song)[i]; \
        gettimeofday(&ev.time, NULL); \
        ev.type = EV_SND; \
        ev.code = SND_TONE; \
        if (note.pitch != 0) { \
            ev.value = note.pitch; \
            if (::write(fd, &ev, sizeof(ev)) != sizeof(ev)) { \
                perror("写入音调失败"); \
                break; \
            } \
        } else { \
            ev.value = 0; \
            if (::write(fd, &ev, sizeof(ev)) != sizeof(ev)) { \
                perror("写入静音失败"); \
                break; \
            } \
        } \
        usleep(note.duration * 1000); \
        if (note.pitch != 0) { \
            gettimeofday(&ev.time, NULL); \
            ev.value = 0; \
            if (::write(fd, &ev, sizeof(ev)) != sizeof(ev)) { \
                perror("发送停止信号失败"); \
                break; \
            } \
        } \
    } \
    close(fd); \
} while(0)

#endif // BUZZER_SONGS_H