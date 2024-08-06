#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

int interp(int x, int in_min, int in_max, int out_min, int out_max);

class Servo {

public:
    Servo(uint8_t pin);

    void set(int angle);
    void update();
    void Calibrate(uint pwm_max, uint pwm_min);

    uint get_slice();

private:
    uint8_t _pin;
    uint _pwm_slice;

    uint _pwm_max = 1000;
    uint _pwm_min = 500;
    uint _pwm_value = 0;

    uint _pwm_freq = 50;
    uint _pwm_wrap = 10000;

    int _angle_min = 50;
    int _angle_max = -30;
};