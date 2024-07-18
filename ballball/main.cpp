#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

#include "servo.h"

Servo servos[3] = {Servo(8), Servo(10), Servo(12)};

void pwm_interrupt_handler() {
    // set pwm value on pwm wrap finish

    int irq;
    int slice;

    irq = pwm_get_irq_status_mask();

    for (int i=0; i<4; i++)
    {
        slice = servos[i].get_slice();

        if (irq & (1<<slice))
        {
            pwm_clear_irq(slice);
            servos[i].update();
        }
    }
}

int main(int argc, char *argv[])
{

    stdio_init_all();

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);
    
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    sleep_ms(1000);

    for (int j=90; j>180; j--) {
        for (int i=0; i<3; i++) {
        servos[i].set(j);
        }
        sleep_ms(1000);
    }
    

    printf("Hello, World!\n");

    return 0;
}
