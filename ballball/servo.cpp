
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

#include "servo.h"

int interp(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Servo::Servo(uint8_t pin) {
    _pin = pin;

    gpio_set_function(_pin, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    _pwm_slice = pwm_gpio_to_slice_num(_pin);
    
    pwm_clear_irq(_pwm_slice);
    pwm_set_irq_enabled(_pwm_slice, true);

    pwm_config config = pwm_get_default_config();
    float clk_frac = 8 * 125 * 1e6 / _pwm_freq;
    pwm_config_set_clkdiv(&config, clk_frac);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(_pwm_slice, &config, true);
    pwm_set_wrap(_pwm_slice, _pwm_wrap);

    // reset
    pwm_set_gpio_level(_pin, 0);
}

void Servo::set(int angle) {
    _pwm_value = interp(angle, _angle_min, _angle_max, _pwm_min, _pwm_max);
}

void Servo::update() {
    // set pwm value on pwm wrap finish
    pwm_set_gpio_level(_pin, _pwm_value);
}

void Servo::Calibrate(uint pwm_max, uint pwm_min) {
    _pwm_max = pwm_max;
    _pwm_min = pwm_min;
    
}

uint Servo::get_slice() {
    return _pwm_slice;
}