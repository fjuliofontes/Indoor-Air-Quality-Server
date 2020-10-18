#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>
#include <stdint.h>

#ifdef DEBUG
    #define DEBUG_PRINT_VERBOSE(fmt,...)\
        _printf((char*)" [%u] ",fmt);\
            _printf(__VA_ARGS__)
    #define DEBUG_PRINT(...)\
        _printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...) 
    #define DEBUG_PRINT_VERBOSE(fmt,...)
#endif

char *myITOA(uint32_t num, uint8_t base, uint8_t decimal_points);
char *myFTOA(float float_part, uint8_t decimal_points , uint8_t base);
void _printf(char* format,...);


#endif