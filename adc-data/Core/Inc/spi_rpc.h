#ifndef SPI_RPC_H
#define SPI_RPC_H

#include <stdint.h>
#include <stdbool.h>

#define SPI_RPC_MAX_MSG_LEN 8
#define SPI_RPC_MAX_ARGS    4

// Enum for supported function IDs
typedef enum {
    FN_ESC_SET = 0,
    FN_SERVO_SET,
    FN_ADC_READ,
    FN_INVALID
} spi_rpc_fn_id_t;

// Handler function pointer type
typedef void (*spi_cmd_handler_t)(char **args, uint8_t argc);
typedef uint16_t (*spi_qry_handler_t)(char **args, uint8_t argc);

void spi_rpc_init(void);
void spi_rpc_on_rx_byte(uint8_t byte);
void spi_rpc_enable_debug(bool enable);

#endif // SPI_RPC_H
