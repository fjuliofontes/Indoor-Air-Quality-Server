#include "util.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

char *myITOA(uint32_t num, uint8_t base, uint8_t decimal_points){
    static char Representation[]= "0123456789ABCDEF";
    static char buffer[50];
    char *ptr;
    uint8_t i = 0;

    ptr = &buffer[49];
    *ptr = '\0';

    do
    {
        i++;
        *--ptr = Representation[num%base];
        num /= base;
    }while(num != 0);

    // padding with left zeros if needed
    while (i < decimal_points){
        i++;
        *--ptr = '0';
    }

    return(ptr);
}

char *myFTOA(float float_part, uint8_t decimal_points , uint8_t base){
    static char buffer[50];
    char *tmp, point = '.';
    uint32_t integer_part = (uint32_t)(float_part);
    uint32_t decimal_part = (uint32_t)((float_part-integer_part)*pow(10,decimal_points));

    memset(buffer,'\0',sizeof(buffer)); // clean string
    tmp = myITOA(integer_part,base, 0);     //  convert messages counter to decimal
    memcpy(buffer, tmp, strlen(tmp));
    strncat(buffer, &point, 1);
    tmp = myITOA(decimal_part,base, decimal_points);     //  convert messages counter to decimal
    strncat(buffer, tmp, strlen(tmp));

    return &buffer[0];
}

void _printf(char* format,...){
    int16_t int_numb;
    uint32_t i, format_len;
    float float_numb;
    char *str;

    // Initializing _printf arguments
    va_list arg;
    va_start(arg, format);
    format_len = strlen(format);

    for(i=0; i < format_len; i++){
        // if current char is not an input spec print it
        // stop at the end of the format string
        while((format[i] != '%') && (i < format_len)){
            Serial.write(format[i]);
            i++;
        }

        // if is end of string break
        if(i >= format_len){
            break;
        }

        //Fetching and executing arguments
        switch(format[++i]){
            case 'c' : int_numb = va_arg(arg,int);         //Fetch char argument
                        Serial.write(int_numb);
                        break;
            case 'i' :
            case 'd' : int_numb = va_arg(arg,int);         //Fetch Decimal/Integer argument
                        if(int_numb<0){ int_numb *= -1; Serial.write('-'); }
                        Serial.print(myITOA(int_numb,10,0));
                        break;

            case 'u' : int_numb = va_arg(arg,int);         //Fetch unsigned Decimal/Integer argument
                        Serial.print(myITOA(int_numb,10,0));
                        break;

            case 'o': int_numb = va_arg(arg,int); //Fetch Octal representation
                        Serial.print(myITOA(int_numb,8,0));
                        break;

            case 's': str = va_arg(arg,char *);       //Fetch string
                        Serial.print(str);
                        break;

            case 'x': int_numb = va_arg(arg,int); //Fetch Hexadecimal representation
                        Serial.print(myITOA(int_numb,16,0));
                        break;

            case 'f': float_numb = va_arg(arg,double);         //Fetch float representation
                        if(float_numb<0){    float_numb *= -1;    Serial.write('-'); }
                        Serial.print(myFTOA(float_numb,3,10));
                        break;
        }
    }

    //Module 3: Closing argument list to necessary clean-up
    va_end(arg);
}
