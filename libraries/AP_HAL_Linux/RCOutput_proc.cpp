
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCOutput_proc.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>
#include <signal.h>
using namespace Linux;

// Interface for modified OMAP proc PWM driver
// This assumes it has complete control over the PWMs, caching information
// instead of the overhead of reading from proc

#define PWM_CHAN_COUNT (4)
#define PWM_PROC_FILENAME_SIZE (64)
#define PWM_PROC_COMMAND_SIZE (64)

static const uint8_t chan_pwm_map[] = { 0, 1, 2, 3 };                //chan_pwm_map[CHANNEL_NUM] = /proc/pwmX;
static const uint8_t pwm_chan_map[] = { 0, 1, 2, 3 };                //pwm_chan_map[pwmX] = CHANNEL_NUM;
static uint32_t pwm_fd[PWM_CHAN_COUNT];
static char proc_cmd[PWM_PROC_COMMAND_SIZE];

// Cached period (ns) and frequency (Hz), keep both for easy reference
// 32-bit unsigned can hold up to a 4.29 second period in ns, and as we
// cannot adjust to < 1 Hz, that will work fine
static uint32_t pwm_period[PWM_CHAN_COUNT];
static uint32_t pwm_frequency[PWM_CHAN_COUNT];
// Cached duty (ns)
static uint32_t pwm_duty[PWM_CHAN_COUNT];

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
void LinuxRCOutput_proc::init(void *machtnicht)
{
    uint8_t i;

    // Open file descriptors to proc files and keep open
    for (i = 0; i < PWM_CHAN_COUNT; i++) {
        char pwm_proc_filename[PWM_PROC_FILENAME_SIZE];
        snprintf(pwm_proc_filename, PWM_PROC_FILENAME_SIZE, "/proc/pwm/pwm%d", chan_pwm_map[i]);
        // Open the file as synchronized, ensuring that output is complete before write returns
        pwm_fd[i] = open(pwm_proc_filename, O_RDWR | O_SYNC);
        pwm_period[i] = 0;
        pwm_frequency[i] = 0;
        pwm_duty[i] = 0;
    }

    // all outputs default to 50Hz, the top level vehicle code
    // overrides this when necessary
    set_freq(0xFFFFFFFF, 50);
}

void LinuxRCOutput_proc::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    uint8_t i;
    uint32_t period = NS_PER_S / (uint32_t)freq_hz;

    for (i = 0; i < PWM_CHAN_COUNT; i++) {
        if (chmask & (1U << i)) {
            pwm_frequency[i] = freq_hz;
            pwm_period[i] = period;
            snprintf(proc_cmd, PWM_PROC_COMMAND_SIZE, "%zu\n", period);
            // RCB we don't currently support changing the frequency
            //::write(pwm_fd[i], proc_cmd, strlen(proc_cmd));
        }
    }
}

uint16_t LinuxRCOutput_proc::get_freq(uint8_t ch)
{
    return pwm_frequency[ch];
}

void LinuxRCOutput_proc::enable_ch(uint8_t ch)
{
    // Set duty to cached value
    write(ch, pwm_duty[ch]);
}

void LinuxRCOutput_proc::disable_ch(uint8_t ch)
{
    // Set duty to zero, but don't zero cache
    write(ch, 0);
}

void LinuxRCOutput_proc::write(uint8_t ch, uint16_t period_us)
{
    pwm_duty[ch] = NS_PER_US * (uint32_t)period_us;
    snprintf(proc_cmd, PWM_PROC_COMMAND_SIZE, "%zu\n", pwm_duty[ch]);
    ::write(pwm_fd[ch], proc_cmd, strlen(proc_cmd));
}

void LinuxRCOutput_proc::write(uint8_t ch, uint16_t *period_us, uint8_t len)
{
    uint8_t i;
    if (len > PWM_CHAN_COUNT) {
        len = PWM_CHAN_COUNT;
    }
    for (i = 0; i < len; i++) {
        write(ch + i, period_us[i]);
    }
}

uint16_t LinuxRCOutput_proc::read(uint8_t ch)
{
    return (pwm_duty[ch] / NS_PER_US);
}

void LinuxRCOutput_proc::read(uint16_t *period_us, uint8_t len)
{
    uint8_t i;
    if (len > PWM_CHAN_COUNT) {
        len = PWM_CHAN_COUNT;
    }
    for (i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

#endif
