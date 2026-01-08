#include "servo.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"

extern TIM_HandleTypeDef htim5;

void servo_set(char **args, uint8_t argc) {
    if (argc < 1) return;
    
    // Parse PWM value directly (1000-2000 range)
    int pwm_value = atoi(args[0]);
    
    // Clamp to safe servo range
    if (pwm_value < 1000) pwm_value = 1000;
    if (pwm_value > 2000) pwm_value = 2000;
    
    // Set TIM5 channel 1 PWM
    TIM5->CCR1 = pwm_value;
    
    // Send confirmation back
    char response[32];
    snprintf(response, sizeof(response), "ANG,%d\r", pwm_value);
    spi_rpc_send_response(response);
}