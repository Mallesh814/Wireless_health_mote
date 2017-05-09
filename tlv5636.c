#include "tlv5636.h"


#define TLV5636_CMD_WRITE_CTRL	(0x9000)
#define TLV5636_CMD_WRITE_DATA	(0x0000)

#define TLV5636_DEFAULT_MODE		((uint16_t)(TLV5636_FAST_MODE | TLV5636_PWR_NORMAL))

void tlv5636_init() {
	// Init chip select

	//Hardwired To GND

	// default power up configuration
	tlv5636_config(TLV5636_REF_2048);
}

void tlv5636_config(tlv5636_ref_t ref) {
	uint16_t data;

	data = TLV5636_CMD_WRITE_CTRL | TLV5636_DEFAULT_MODE| (uint16_t)ref;

	SSIDataPut(SSI0_BASE, data);
}

void tlv5636_set_output(uint16_t out) {
	uint16_t data;

	data = (out & 0x0FFF) | TLV5636_CMD_WRITE_DATA | TLV5636_DEFAULT_MODE;

	SSIDataPut(SSI0_BASE, data);
}
