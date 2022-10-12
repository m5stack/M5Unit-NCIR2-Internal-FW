#ifndef _MLX90614_H_
#define _MLX90614_H_

#include "main.h"
 
bool mlx90614_get_data(uint8_t *data);
bool mlx90614_set_emissivity(uint16_t emissivity);
bool mlx90614_get_soc_data(uint8_t *data);
bool mlx90614_get_emissivity(uint16_t* emissivity);

#endif
