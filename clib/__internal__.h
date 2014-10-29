#ifndef __INTERNAL__H
#define __INTERNAL__H

#include <stdarg.h>

typedef int (*putc_t)(void*, char);
typedef int (*puts_t)(void*, const char*);

int vprintf_core(const char*, va_list, putc_t, puts_t, void*);

#endif
