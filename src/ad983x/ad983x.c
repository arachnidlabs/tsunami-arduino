#include "ad983x.h"

#define REG_FREQ1   0x8000
#define REG_FREQ0   0x4000
#define REG_PHASE0  0xC000
#define REG_PHASE1  0xE000

#define REG_B28     0x2000
#define REG_HLB     0x1000
#define REG_FSEL    0x0800
#define REG_PSEL    0x0400
#define REG_PINSW   0x0200
#define REG_RESET   0x0100
#define REG_SLEEP1  0x0080
#define REG_SLEEP12 0x0040
#define REG_OPBITEN 0x0020
#define REG_SIGNPIB 0x0010
#define REG_DIV2    0x0008
#define REG_MODE    0x0002

#define SIGN_OUTPUT_MASK (REG_OPBITEN | REG_SIGNPIB | REG_DIV2 | REG_MODE)

static void write_reg(ad983x_t *dds, uint16_t value) {
  char data[2];
  data[0] = value >> 8;
  data[1] = value & 0xFF;
  dds->spi_transfer(dds->ctx, &data, sizeof(data));
}

void ad983x_init(ad983x_t *dds, spi_transfer_func func, void *ctx) {
	dds->spi_transfer = func;
	dds->ctx = ctx;
	// Set to two writes per frequency register, and pin control of functions.
	dds->reg = REG_B28 | REG_PINSW;
}

void ad983x_start(ad983x_t *dds) {
	write_reg(dds, dds->reg);
	ad983x_set_frequency(dds, 0, 0);
	ad983x_set_frequency(dds, 1, 0);
	ad983x_set_phase(dds, 0, 0);
	ad983x_set_phase(dds, 1, 0);
}

void ad983x_set_frequency(ad983x_t *dds, uint8_t reg, uint32_t frequency) {
  write_reg(dds, (reg?REG_FREQ1:REG_FREQ0) | (frequency & 0x3FFF));
  write_reg(dds, (reg?REG_FREQ1:REG_FREQ0) | ((frequency >> 14) & 0x3FFF));
}

void ad983x_set_phase(ad983x_t *dds, uint8_t reg, uint32_t phase) {
  write_reg(dds, (reg?REG_PHASE1:REG_PHASE0) | (phase & 0x0FFF));
}

void ad983x_set_sign_output(ad983x_t *dds, ad983x_sign_output_t output) {
  dds->reg = (dds->reg & ~SIGN_OUTPUT_MASK) | output;
  write_reg(dds, dds->reg);
}

void ad983x_set_output_mode(ad983x_t *dds, ad983x_output_mode_t mode) {
  if(mode == AD983X_OUTPUT_MODE_TRIANGLE) {
    dds->reg = (dds->reg & ~SIGN_OUTPUT_MASK) | mode;
  } else {
    dds->reg &= ~REG_MODE;
  }
  write_reg(dds, dds->reg);
}
