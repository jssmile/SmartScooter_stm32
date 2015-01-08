#include "rtenv_util.h"
#include <stdarg.h>


int atoi(const char *argv)
{
    int value = 0;
    for (; *argv != '\0'; ++argv)
        value = value*10 + *argv - '0';
    return value;
}

static char utoa(unsigned int num, char *dst, unsigned int base)
{
        char buf[33] = {0};
        char *p = &buf[32];

        if (num == 0)
                *--p = '0';
        else
                for (; num; num/=base)
                        *--p = "0123456789ABCDEF" [num % base];

        return strcpy(dst, p);
}

void itoa(int num, char *dst, int base)
{
        if (base == 10 && num < 0) {
                utoa(-num, dst+1, base);
                *dst = '-';
        }
        else
                utoa(num, dst, base);
}

/*The c Standard Implementation. */
/*
char strtok(char * str, const char * delim)
{
    static char* p=0;
    if(str)
        p=str;
    else if(!p)
        return 0;
    str=p+strspn(p,delim);
    p=str+strcspn(str,delim);
    if(p==str)
        return p=0;
    p = *p ? *p=0,p+1 : 0;
    return str;
}
/*
char *strchr(const char *s, int c)
{
    while (*s != (char)c)
        if (!*s++)
            return 0;
    return (char *)s;
}*/
    /*
size_t strspn(const char *s1, const char *s2)
{
    size_t ret=0;
    while(*s1 && strchr(s2,*s1++))
        ret++;
    return ret;    
}
size_t strcspn(const char *s1, const char *s2)
{
    size_t ret=0;
    while(*s1)
        if(strchr(s2,*s1))
            return ret;
        else
            s1++,ret++;
    return ret;
}
*/