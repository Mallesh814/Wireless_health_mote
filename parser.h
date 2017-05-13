/*
 * parser.h
 *
 *  Created on: Nov 10, 2015
 *      Author: Mallesh
 */

#ifndef PARSER_H_
#define PARSER_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"

	void str_cpy(char* ,char* );
	char* str_ncpy(char* ,char* ,uint32_t );
    uint32_t str_len(char*);

    uint32_t *mem_cpy(void* dest_memory_address, const void* src_memory_address, uint32_t number_of_bytes);
    void transfer(uint8_t*, uint32_t);
	void dec_ascii(uint8_t*, uint32_t);
	uint32_t ascii_dec(char* ,uint32_t*);
	uint32_t ascii_hex_dec(char*,uint32_t*);
	uint32_t int_hex_ascii(uint8_t* , uint8_t);
	uint32_t int_hex_ascii_big(char* , uint32_t );

#endif /* PARSER_H_ */
