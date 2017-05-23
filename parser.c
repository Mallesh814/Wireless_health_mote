#include "parser.h"
#define MAX_ARGS 4

uint32_t deci = 0;
char ascii[6]="\0";
uint32_t hex_addr = 0;

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
        )
{
    int32_t i;
    int32_t mx, mn;
    int32_t mx_pos = 0;
    int32_t mn_pos = 0;

    int32_t is_detecting_emi = emi_first;

    mx = data[0];
    mn = data[0];

    *num_emi_peaks = 0;
    *num_absop_peaks = 0;

    for(i = 1; i < data_count; ++i)
    {
        if(data[i] > mx)
        {
            mx_pos = i;
            mx = data[i];
        }
        if(data[i] < mn)
        {
            mn_pos = i;
            mn = data[i];
        }

        if(is_detecting_emi &&
                data[i] < mx - delta)
        {
            if(*num_emi_peaks >= max_emi_peaks) /* not enough spaces */
                return 1;

            emi_peaks[*num_emi_peaks] = mx_pos;
            ++ (*num_emi_peaks);

            is_detecting_emi = 0;

            i = mx_pos - 1;

            mn = data[mx_pos];
            mn_pos = mx_pos;
        }
        else if((!is_detecting_emi) &&
                data[i] > mn + delta)
        {
            if(*num_absop_peaks >= max_absop_peaks)
                return 2;

            absop_peaks[*num_absop_peaks] = mn_pos;
            ++ (*num_absop_peaks);

            is_detecting_emi = 1;
            i = mn_pos - 1;

            mx = data[mn_pos];
            mx_pos = mn_pos;
        }
    }

    return 0;
}


uint32_t *mem_cpy(void* dest_memory_address, const void* src_memory_address, uint32_t number_of_bytes)
{
    uint32_t i;

	if( ((uint32_t)dest_memory_address % 4 == 0) &&
        ((uint32_t)src_memory_address % 4 == 0) &&
            number_of_bytes % 4 == 0)
    {
        uint32_t *dest = dest_memory_address;
        const uint32_t *src = src_memory_address;

        for(i=0; i < number_of_bytes/4 ; i++){
            dest[i] = src[i];
        }
    }
    else {
        uint8_t *dest = dest_memory_address;
        const uint8_t *src = src_memory_address;

        for(i=0; i < number_of_bytes ; i++){
            dest[i] = src[i];
        }
    }

    return dest_memory_address;
}

void str_cpy(char* dest,char* src)
		{
			uint32_t i,len1;
			len1 = str_len(dest);
			for(i=0;src[i] != '\0';i++) dest[i] = src[i];
			if(i > len1) dest[i] = '\0';
		}


char* str_ncpy(char* dest,char* src,uint32_t n)
		{
			uint32_t len1=0,len2=0,i;
			len1 = str_len(dest);
			len2 = str_len(src);
			if(n > len2) n=len2;

			for(i=0;i < n;i++) dest[i] = src[i];
			if(i > len1) dest[i] = '\0';
			return dest;
		}

uint32_t str_len(char* str)
{
	uint32_t len;
	for(len = 0;str[len] != '\0';len++);
	return(len);
}


//****** Function that sends the received STRING to UART interface

void transfer(uint8_t* data, uint32_t uart_base){
		while (*data){
			UARTCharPut(uart_base, *data);
			data++;
		}

}

void dec_ascii(uint8_t*pasc, uint32_t num){
	uint32_t j;
	j=num;
	do{
		++pasc;
		j=j/10;
	}while(j);
	*pasc = '\0';
	do{
		*(--pasc)=(num%10) + 0x30;
			num = num/10;
	}while(num);
}

uint32_t ascii_dec(char* asc,uint32_t* num)										// blink determines the mode of operation
{
	uint32_t dec=0;
	while(*asc)
		{
			if (((*asc) ^ 0x30) <= 9)	dec = (dec * 10) + ((*asc) ^ 0x30);	// (asc[k] ^ 0x30) will be > 9 for non numerical characters
			else
			{
				*num = dec;
				return 0;														// second element holds the half converted value
			}
			asc++;
		}
	*num = dec;
	return 1;
}

uint32_t ascii_hex_dec(char* asc,uint32_t* num)										// blink determines the mode of operation
{
	uint32_t hex=0;
	while(*asc)
		{
			if((*asc) >= '0' &&  (*asc) <= '9') hex = 	(hex * 16) + ((*asc) - '0');
			else if((*asc) >= 'A' &&  (*asc) <= 'F') hex = 	(hex * 16) + (0xA + (*asc) - 'A');
			else if((*asc) >= 'a' &&  (*asc) <= 'f') hex = 	(hex * 16) + (0xA + (*asc) - 'a');
			else return 0;
			asc++;
		}
	*num = hex;
	return 1;
}

uint32_t int_hex_ascii(uint8_t* hex_str, uint8_t num)
{
	uint32_t i=0;
	char str[]="00";
	char *pstr = &str[1];
	*hex_str = '\0';//	*(hex_str+1) = 'x';	*(hex_str+2) = '\0';

	while(num){
		i = num%16;

		if(i <= 9)	*pstr = (i + '0');
		else if(i <= 15) *pstr =(i  - 0xA + 'A');
		else	return 0;

		num = num/16;
		pstr--;
	}
	str_ncpy(hex_str,str,3);
	return 1;
}

uint32_t int_hex_ascii_big(char* hex_str, uint32_t num)
{
	uint32_t i=0;
	char str[11]="0x00000000";
	char *pstr = &str[9];
	*hex_str = '\0';//	*(hex_str+1) = 'x';	*(hex_str+2) = '\0';

	while(num){
		i = num%16;

		if(i <= 9)	*pstr = (i + '0');
		else if(i <= 15) *pstr =(i  - 0xA + 'A');	
		else	return 0;

		num = num/16;
		pstr--;
	}
	str_cpy(hex_str,str);
	return 1;
}
