#ifndef GPIOS_ID_H
#define GPIOS_ID_H

#include <stdio.h>
#include <stdint.h>

#include <esp_system.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"


#include <ets_sys.h>

#include "user_config.h"

int8_t  get_pin_index(uint8_t pin);

uint32_t get_pin_mux(uint8_t pin);

uint8_t  get_pin_func(uint8_t pin);

int8_t hex2int(char ch);

#endif
