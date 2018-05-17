/* FAST INTEGER SQUARE ROOT FUNCTION
 * From http://www.codecodex.com/wiki/Calculate_an_integer_square_root
 * With some modification
 * 
 * Example use:
 * uint16_t result = isqrt16(15);
 * 
*/

#include "fastintsqrt.h"

uint16_t isqrt16 (uint16_t n) // OR isqrt16 ( uint16 n ) OR  isqrt8 ( uint8 n ) - respectively [ OR overloaded as isqrt (uint?? n) in C++ ]  
{  
    register uint16_t root, remainder, place;
    
    root = 0;  
    remainder = n;  
    place = 0x4000; // OR place = 0x4000; OR place = 0x40; - respectively  
      
    while (place > remainder)  
        place = place >> 2;  
    while (place)  
    {  
        if (remainder >= root + place)  
        {  
            remainder = remainder - root - place;  
            root = root + (place << 1);  
        }  
        root = root >> 1;  
        place = place >> 2;  
    }  
    if(remainder>root) root++;
    return root;  
}  