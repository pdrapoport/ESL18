/* FAST INTEGER SQUARE ROOT FUNCTION
 * From http://www.codecodex.com/wiki/Calculate_an_integer_square_root
 * With some modification
 * 
 * Example use:
 * uint16_t result = isqrt16(15);
 * 
*/

#ifndef FASTINTSQRT_H__
#define FASTINTSQRT_H__

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

uint16_t isqrt16(uint16_t n);

#endif