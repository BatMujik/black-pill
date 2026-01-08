#include "main.h"
#include "spi_rpc.h"
#include "esc.h"
#include "servo.h"
#include "adc.h"
#include <string.h>
#include <stdio.h>

// Use 64-byte buffer matching your SPI transfer size
#define MSG_BUFFER_SIZE 64

static char rx_buf[MSG_BUFFER_SIZE];
static uint16_t rx_idx = 0;
static bool debug_enabled = false;

// Function ID string table
static const char *fn_id_strs[] = {"ESC", "SER", "ADC"};

// Command handler table
static const spi_cmd_handler_t cmd_handlers[] = {
    esc_set,
    servo_set,
    NULL // ADC is query only
};

// Query handler table
static const spi_qry_handler_t qry_handlers[] = {
    NULL, // ESC not query
    NULL, // SER not query
    adc_read
};

void spi_rpc_enable_debug(bool enable) {
    debug_enabled = enable;
}

void spi_rpc_init(void) {
    rx_idx = 0;
    memset(rx_buf, 0, sizeof(rx_buf));
}

static spi_rpc_fn_id_t fn_id_from_str(const char *str) {
    for (int i = 0; i < FN_INVALID; ++i) {
        if (strcmp(str, fn_id_strs[i]) == 0) return (spi_rpc_fn_id_t)i;
    }
    return FN_INVALID;
}

static void spi_rpc_process_msg(char *msg) {
    // Find message end (look for \r or \n)
    char *end = strchr(msg, '\r');
    if (!end) end = strchr(msg, '\n');
    if (end) *end = '\0'; // Null-terminate at line ending
    
    // Skip if empty
    if (strlen(msg) == 0) return;
    
    // Tokenize
    char *argv[SPI_RPC_MAX_ARGS+2];
    uint8_t argc = 0;
    char *tok = strtok(msg, ",");
    while (tok && argc < SPI_RPC_MAX_ARGS+2) {
        argv[argc++] = tok;
        tok = strtok(NULL, ",");
    }
    
    if (argc < 2) return;
    
    // CMD/QRY
    bool is_query = (strcmp(argv[0], "QRY") == 0);
    spi_rpc_fn_id_t fn_id = fn_id_from_str(argv[1]);
    if (fn_id == FN_INVALID) return;
    
    if (!is_query && cmd_handlers[fn_id]) {
        cmd_handlers[fn_id](&argv[2], argc-2);
    } else if (is_query && qry_handlers[fn_id]) {
        uint16_t value = qry_handlers[fn_id](&argv[2], argc-2);
        char resp_buf[64] = {0};
        snprintf(resp_buf, sizeof(resp_buf), "RESP,ADC,%u\r\n", value);
        spi_rpc_send_response(resp_buf);
    }
}

void spi_rpc_on_rx_byte(uint8_t byte) {
    // Collect bytes until we find a complete message (ending with \r or \n)
    if (rx_idx < MSG_BUFFER_SIZE - 1) {
        rx_buf[rx_idx++] = byte;
        rx_buf[rx_idx] = '\0'; // Keep null-terminated
        
        // Check if message is complete
        if (byte == '\r' || byte == '\n') {
            spi_rpc_process_msg(rx_buf);
            rx_idx = 0;
            memset(rx_buf, 0, sizeof(rx_buf));
        }
    } else {
        // Buffer overflow - reset
        rx_idx = 0;
        memset(rx_buf, 0, sizeof(rx_buf));
    }
}