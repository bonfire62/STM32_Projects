#include <stdint.h>

char const my_data[] = "embedded flash out";
#define BASE_ADDRESS_OF_SRAM 0x200003F0 //offset by base code

int main(void){
	for(int i = 0; i < sizeof(my_data); i++){
		
		*((uint8_t*) BASE_ADDRESS_OF_SRAM +i ) = my_data[i];
	}
	
	return 0;
}
