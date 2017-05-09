#ifndef _TLV5636_H_
#define _TLV5636_H_

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/ssi.h"
#include "inc/hw_memmap.h"

typedef enum {
	TLV5636_FAST_MODE = 0x4000,
	TLV5636_SLOW_MODE = 0x0000,
} tlv5636_mode_t;

typedef enum {
	TLV5636_PWR_DOWN = 0x2000,
	TLV5636_PWR_NORMAL = 0x0000,
} tlv5636_pwr_t;

typedef enum {
	TLV5636_REF_EXTERNAL = 0,
	TLV5636_REF_1024 = 1,
	TLV5636_REF_2048 = 2,
} tlv5636_ref_t;

void tlv5636_init();
void tlv5636_config(tlv5636_ref_t ref);
/* Write the raw DAC output (12-bit) */
void tlv5636_set_output(uint16_t out);

#endif
