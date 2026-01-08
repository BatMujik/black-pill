#include "esc.h"
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
void esc_set(char **args, uint8_t argc) {
    if (argc < 1) return;
    int speed = atoi(args[0]);
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    // TODO: Set ESC PWM duty cycle here
    // Example: TIM5->CCR1 = map_speed_to_pwm(speed);
}
