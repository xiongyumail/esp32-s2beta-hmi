/*  Copyright (C) 2017  Florian Menne
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAIN_WS2812B_H_
#define MAIN_WS2812B_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/rmt.h"

typedef struct
{
    uint8_t g;
    uint8_t b;
    uint8_t r;
}wsRGB_t;

/**
 * Init RMT module and allocates space
 * @param channel RMT channel
 * @param gpio GPIO Pin
 * @param size Number of LED's
 */
void WS2812B_init(rmt_channel_t channel, gpio_num_t gpio, unsigned int size);

/**
 * Writes to the LED
 * @param data
 * @param size Number of LED's - Must not exceed initialization size
 */
void WS2812B_setLeds(wsRGB_t* data, unsigned int size);

/**
 * Deinit driver and free memory space
 */
void WS2812B_deInit(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_WS2812B_H_ */

