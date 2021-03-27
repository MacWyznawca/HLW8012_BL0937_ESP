#include "gpios_id.h"

#define NUM_VALID_INPUT_PINS 11
static const uint32_t pin_mux[NUM_VALID_INPUT_PINS] = {PERIPHS_IO_MUX_GPIO0_U,PERIPHS_IO_MUX_U0TXD_U,PERIPHS_IO_MUX_GPIO2_U,PERIPHS_IO_MUX_U0RXD_U,PERIPHS_IO_MUX_GPIO4_U,PERIPHS_IO_MUX_GPIO5_U,PERIPHS_IO_MUX_MTDI_U,PERIPHS_IO_MUX_MTCK_U,PERIPHS_IO_MUX_MTMS_U,PERIPHS_IO_MUX_MTDO_U,0};
static const uint8_t pin_num[NUM_VALID_INPUT_PINS] = {0,1,2,3,4,5,12,13,14,15,16};
static const uint8_t pin_func[NUM_VALID_INPUT_PINS] = {FUNC_GPIO0,FUNC_GPIO1,FUNC_GPIO2,FUNC_GPIO3,FUNC_GPIO4,FUNC_GPIO5,FUNC_GPIO12,FUNC_GPIO13,FUNC_GPIO14,FUNC_GPIO15,0};

int8_t ICACHE_FLASH_ATTR get_pin_index(uint8_t pin){
	int i = 0;
	for(i = 0; i < NUM_VALID_INPUT_PINS; i++){
		if(pin==pin_num[i]){
			return i;
		}
	}
	return -1;
}

uint32_t ICACHE_FLASH_ATTR get_pin_mux(uint8_t pin){
  uint8_t id = get_pin_index(pin);
  return id == -1 ? -1 : pin_mux[get_pin_index(pin)];
}

uint8_t ICACHE_FLASH_ATTR get_pin_func(uint8_t pin){
  uint8_t id = get_pin_index(pin);
  return id == -1 ? -1 : pin_func[get_pin_index(pin)];
}

int8_t ICACHE_FLASH_ATTR hex2int(char ch){
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'z')
        return ch - 'a' + 10;
    return -1;
}

