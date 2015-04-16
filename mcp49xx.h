#ifndef __MCP49XX_H
#define __MCP49XX_H

#include <stdint.h>

typedef void (*spi_transfer_func)(void *ctx, char data[], int len);

typedef struct {
	spi_transfer_func spi_transfer;
	void *ctx;
	uint16_t registers[2];
} mcp49xx_t;

typedef enum {
	MCP49XX_GAIN_1X,
	MCP49XX_GAIN_2X
} mcp49xx_gain_t;

void mcp49xx_init(mcp49xx_t *dac, spi_transfer_func func, void *ctx);
void mcp49xx_set_is_buffered(mcp49xx_t *dac, uint8_t idx, uint8_t is_buffered);
void mcp49xx_set_gain(mcp49xx_t *dac, uint8_t idx, mcp49xx_gain_t gain);
void mcp49xx_set_is_shutdown(mcp49xx_t *dac, uint8_t idx, uint8_t is_shutdown);
void mcp49xx_write(mcp49xx_t *dac, uint8_t idx, uint16_t value);

#endif
