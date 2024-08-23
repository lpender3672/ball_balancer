#include <stdio.h>
#include "pico/stdlib.h"

#include <string.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

#include "servo.h"

#define MAX_COMMAND_LENGTH 64

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

float x[3] = {0 , 80 * sqrt(3)/2, - 80 * sqrt(3)/2};
float y[3] = {-80, 40, 40};

void set_servo_angle_phi_theta(float phi, float theta) {

    if (theta < -30) {
        theta = -30;
    } else if (theta > 30) {
        theta = 30;
    }

    if (abs(phi - 90) < 0.01) {
        phi = 90 + signbit(phi - 90) * 0.01;
    }

    for (int i=0; i<3; i++) {

            float z = - tan(theta * 3.1415926 / 180) * ( x[i] * cos(phi * 3.1415926 / 180) + y[i] * sin(phi * 3.1415926 / 180) );
            float angle = asin(z / 70) * 180 / 3.1415926;

            printf("angle: %f\n", angle);

            if (angle < -25) {
                angle = -25;
            } else if (angle > 45) {
                angle = 45;
            }

            servos[i].set(angle);
    }
}

void set_servo_angle_alpha_beta(float alpha, float beta) {

    if (alpha < -30) {
        alpha = -30;
    } else if (alpha > 30) {
        alpha = 30;
    }

    if (beta < -30) {
        beta = -30;
    } else if (beta > 30) {
        beta = 30;
    }

    for (int i=0; i<3; i++) {

        float a, b, c;
        a = sin(alpha * 3.1415926 / 180) * cos(beta * 3.1415926 / 180);
        b = cos(alpha * 3.1415926 / 180) * sin(beta * 3.1415926 / 180);
        c = cos(alpha * 3.1415926 / 180) * cos(beta * 3.1415926 / 180);

        float z = - (1 / c) * ( x[i] * a + y[i] * b );
        float angle = asin(z / 70) * 180 / 3.1415926;

        printf("angle: %f\n", angle);

        if (angle < -25) {
            angle = -25;
        } else if (angle > 45) {
            angle = 45;
        }

        servos[i].set(angle);
    }
}

void parse_command(const char* command) {
    float a_value = 0, b_value = 0;
    char *a_str = strstr(command, "A");
    char *b_str = strstr(command, "B");
    
    if (a_str && b_str) {
        a_value = strtof(a_str + 1, NULL);
        b_value = strtof(b_str + 1, NULL);
        printf("Received: A=%f B=%f\n", a_value, b_value);
        set_servo_angle_alpha_beta(a_value, b_value);
    } else {
        printf("Invalid command format\n");
    }
}

int main() {
    stdio_init_all();

    servos[0].Calibrate(1000, 560);
    servos[1].Calibrate(1180, 650);
    servos[2].Calibrate(1020, 720);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    printf("USB Serial Command Parser Ready\n");

    char command_buffer[MAX_COMMAND_LENGTH];
    int buffer_index = 0;

    while (1) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\n' || c == '\r') {
                command_buffer[buffer_index] = '\0';  // Null-terminate the string
                parse_command(command_buffer);
                buffer_index = 0;  // Reset buffer
            } else if (buffer_index < MAX_COMMAND_LENGTH - 1) {
                command_buffer[buffer_index++] = c;
            }
        }
        // You can add other non-blocking tasks here
    }

    return 0;
}
