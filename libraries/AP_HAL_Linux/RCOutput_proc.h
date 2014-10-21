
#ifndef __AP_HAL_LINUX_RCOUTPUT_PROC_H__
#define __AP_HAL_LINUX_RCOUTPUT_PROC_H__

#include <AP_HAL_Linux.h>

// PWM driver takes period and duty cycle in ns
#define NS_PER_S (1000000000)
#define NS_PER_US (1000)

class Linux::LinuxRCOutput_proc : public AP_HAL::RCOutput {
    void     init(void *machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t *period_us, uint8_t len);
    uint16_t read(uint8_t ch); 
    void     read(uint16_t *period_us, uint8_t len);

private:

};

#endif // __AP_HAL_LINUX_RCOUTPUT_PROC_H__
