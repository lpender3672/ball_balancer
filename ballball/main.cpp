#include <stdio.h>
#include "pico/stdlib.h"

#include <string.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

#include "servo.h"

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "dhcpserver.h"
#include "dnsserver.h"

#define TCP_PORT 80
#define DEBUG_printf printf
#define POLL_TIME_S 5
#define HTTP_GET "GET"
#define HTTP_RESPONSE_HEADERS "HTTP/1.1 %d OK\nContent-Length: %d\nContent-Type: text/html; charset=utf-8\nConnection: close\n\n"
#define LED_TEST_BODY "<html><body><h1>Hello from Pico W.</h1><p>Led is %s</p><p><a href=\"?led=%d\">Turn led %s</a></body></html>"
#define LED_PARAM "led=%d"
#define LED_TEST "/ledtest"
#define LED_GPIO 0
#define HTTP_RESPONSE_REDIRECT "HTTP/1.1 302 Redirect\nLocation: http://%s" LED_TEST "\n\n"

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    bool complete;
    ip_addr_t gw;
    async_context_t *context;
} TCP_SERVER_T;

typedef struct TCP_CONNECT_STATE_T_ {
    struct tcp_pcb *pcb;
    int sent_len;
    char headers[128];
    char result[256];
    int header_len;
    int result_len;
    ip_addr_t *gw;
} TCP_CONNECT_STATE_T;

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

float x[3] = {0 , 60 * sqrt(3)/2, - 60 * sqrt(3)/2};
float y[3] = {60, -30, -30};

void set_servo_angle(float phi, float theta) {

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

int main(int argc, char *argv[])
{

    stdio_init_all();

    servos[0].Calibrate(980, 590);
    servos[1].Calibrate(1090, 580);
    servos[2].Calibrate(920, 580);


    /*
    if (cyw43_arch_init()) {
        return -1; 
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(1000);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(1000);
    */

    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    float phi, theta;
    phi = 0;
    theta = 10;

    while (true) {

        set_servo_angle(phi, theta);
        sleep_ms(10);

        phi += 1;
        if (phi > 360) {
            phi = 0;
        }
    }

    printf("Hello, World!\n");

    return 0;
}
