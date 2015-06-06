#include "mcp49xx.h"

#define MCP49XX_DAC_ID 15
#define MCP49XX_DAC_BUF 14
#define MCP49XX_DAC_GAIN 13
#define MCP49XX_DAC_SHDN 12
#define MCP49XX_DAC_VALUE_MASK 0x0FFF

static void _transfer(mcp49xx_t *dac, uint8_t idx) {
	uint8_t data[2];
	data[0] = dac->registers[idx] >> 8;
	data[1] = dac->registers[idx] & 0xFF;
	dac->spi_transfer(dac->ctx, data, sizeof(data));
}

void mcp49xx_init(mcp49xx_t *dac, spi_transfer_func func, void *ctx) {
	dac->spi_transfer = func;
	dac->ctx = ctx;
	dac->registers[0] = 0;
	dac->registers[1] = 1 << MCP49XX_DAC_ID;
	_transfer(dac, 0);
	_transfer(dac, 1);
}

void mcp49xx_set_is_buffered(mcp49xx_t *dac, uint8_t idx, uint8_t is_buffered) {
	if(is_buffered) {
		dac->registers[idx] |= 1 << MCP49XX_DAC_BUF;
	} else {
		dac->registers[idx] &= ~(1 << MCP49XX_DAC_BUF);
	}
	_transfer(dac, idx);
}

void mcp49xx_set_gain(mcp49xx_t *dac, uint8_t idx, mcp49xx_gain_t gain) {
	if(gain == MCP49XX_GAIN_2X) {
		dac->registers[idx] &= ~(1 << MCP49XX_DAC_GAIN);
	} else {
		dac->registers[idx] |= 1 << MCP49XX_DAC_GAIN;
	}
	_transfer(dac, idx);
}

void mcp49xx_set_is_shutdown(mcp49xx_t *dac, uint8_t idx, uint8_t is_shutdown) {
	if(is_shutdown) {
		dac->registers[idx] &= ~(1 << MCP49XX_DAC_SHDN);
	} else {
		dac->registers[idx] |= 1 << MCP49XX_DAC_SHDN;
	}
	_transfer(dac, idx);
}

void mcp49xx_write(mcp49xx_t *dac, uint8_t idx, uint16_t value) {
	dac->registers[idx] = (dac->registers[idx] & ~MCP49XX_DAC_VALUE_MASK)
	  | (value & MCP49XX_DAC_VALUE_MASK);
	_transfer(dac, idx);
}
