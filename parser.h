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

int32_t detect_peak(
        int32_t*    data, /* the data */
        int32_t     data_count, /* row count of data */
        int32_t*    emi_peaks, /* emission peaks will be put here */
        int32_t*    num_emi_peaks, /* number of emission peaks found */
        int32_t     max_emi_peaks, /* maximum number of emission peaks */
        int32_t*    absop_peaks, /* absorption peaks will be put here */
        int32_t*    num_absop_peaks, /* number of absorption peaks found */
        int32_t     max_absop_peaks, /* maximum number of absorption peaks*/
        int32_t     delta, /* delta used for distinguishing peaks */
        int32_t     emi_first /* should we search emission peak first of absorption peak first? */
        );
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
