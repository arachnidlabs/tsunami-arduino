#ifndef __AD983X_H
#define __AD983X_H

#include <stdint.h>

typedef void (*spi_transfer_func)(void *ctx, char data[], int len);

typedef struct {
	spi_transfer_func spi_transfer;
	void *ctx;
	uint16_t reg;
} ad983x_t;

typedef enum {
	AD983X_SIGN_OUTPUT_NONE			= 0x0000,
	AD983X_SIGN_OUTPUT_MSB 			= 0x0028,
	AD983X_SIGN_OUTPUT_MSB_2		= 0x0020,
	AD983X_SIGN_OUTPUT_COMPARATOR	= 0x0038,
} ad983x_sign_output_t;

typedef enum {
	AD983X_OUTPUT_MODE_SINE 	= 0x0000,
	AD983X_OUTPUT_MODE_TRIANGLE = 0x0002,
} ad983x_output_mode_t;

void ad983x_init(ad983x_t *dds, spi_transfer_func func, void *ctx);
void ad983x_start(ad983x_t *dds);
void ad983x_set_frequency(ad983x_t *dds, uint8_t reg, uint32_t frequency);
void ad983x_set_phase(ad983x_t *dds, uint8_t reg, uint32_t phase);
void ad983x_set_sign_output(ad983x_t *dds, ad983x_sign_output_t output);
void ad983x_set_output_mode(ad983x_t *dds, ad983x_output_mode_t mode);

#endif
