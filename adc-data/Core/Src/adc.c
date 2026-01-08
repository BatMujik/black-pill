#include "adc.h"
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
uint16_t adc_read(char **args, uint8_t argc) {
    if (argc < 1) return 0;
    int ch = atoi(args[0]);
    if (ch < 0) ch = 0;
    if (ch > 3) ch = 3;
    // TODO: Read ADC channel using HAL
    // Example: return ADC_VAL[ch];
    return 1234; // Dummy value for now
}
