#include "mcp49xx.h"

#define MCP49XX_DAC_ID 15
#define MCP49XX_DAC_BUF 14
#define MCP49XX_DAC_GAIN 13
#define MCP49XX_DAC_SHDN 12
#define MCP49XX_DAC_VALUE_MASK 0x0FFF

void mcp49xx_init(mcp49xx_t *dac, spi_transfer_func func, void *ctx) {
	dac->spi_transfer = func;
	dac->ctx = ctx;
	dac->registers[0] = 0;
	dac->registers[1] = 1 << MCP49XX_DAC_ID;
	dac->spi_transfer(dac->registers[0], sizeof(dac->registers[0]), dac->ctx);
	dac->spi_transfer(dac->registers[1], sizeof(dac->registers[1]), dac->ctx);
}

void mcp49xx_set_is_buffered(mcp49xx_t *dac, uint8_t idx, uint8_t is_buffered) {
	if(is_buffered) {
		dac->registers[idx] |= 1 << MCP49XX_DAC_BUF;
	} else {
		dac->registers[idx] &= ~(1 << MCP49XX_DAC_BUF);
	}
	dac->spi_transfer(dac->registers[idx], sizeof(dac->registers[idx]), dac->ctx);
}

void mcp49xx_set_gain(mcp49xx_t *dac, uint8_t idx, mcp49xx_gain_t gain) {
	if(gain == MCP49XX_GAIN_2X) {
		dac->registers[idx] &= ~(1 << MCP49XX_DAC_GAIN);
	} else {
		dac->registers[idx] |= 1 << MCP49XX_DAC_GAIN;
	}
	dac->spi_transfer(dac->registers[idx], sizeof(dac->registers[idx]), dac->ctx);
}

void mcp49xx_set_is_shutdown(mcp49xx_t *dac, uint8_t idx, uint8_t is_shutdown) {
	if(is_shutdown) {
		dac->registers[idx] &= ~(1 << MCP49XX_DAC_SHDN);
	} else {
		dac->registers[idx] |= 1 << MCP49XX_DAC_SHDN;
	}
	dac->spi_transfer(dac->registers[idx], sizeof(dac->registers[idx]), dac->ctx);
}

void mcp49xx_write(mcp49xx_t *dac, uint8_t idx, uint16_t value) {
	dac->registers[idx] = (dac->registers[idx] & ~MCP49XX_DAC_VALUE_MASK)
	  | (value & MCP49XX_DAC_VALUE_MASK);
	dac->spi_transfer(dac->registers[idx], sizeof(dac->registers[idx]), dac->ctx);
}
