#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "clib/__internal__.h"

#define _CTYPE_DATA_0_127 \
	_C,     _C,     _C,     _C,     _C,     _C,     _C,     _C, \
	_C,     _C|_S, _C|_S, _C|_S,    _C|_S,  _C|_S,  _C,     _C, \
	_C,     _C,     _C,     _C,     _C,     _C,     _C,     _C, \
	_C,     _C,     _C,     _C,     _C,     _C,     _C,     _C, \
	_S|_B,  _P,     _P,     _P,     _P,     _P,     _P,     _P, \
	_P,     _P,     _P,     _P,     _P,     _P,     _P,     _P, \
	_N,     _N,     _N,     _N,     _N,     _N,     _N,     _N, \
	_N,     _N,     _P,     _P,     _P,     _P,     _P,     _P, \
	_P,     _U|_X,  _U|_X,  _U|_X,  _U|_X,  _U|_X,  _U|_X,  _U, \
	_U,     _U,     _U,     _U,     _U,     _U,     _U,     _U, \
	_U,     _U,     _U,     _U,     _U,     _U,     _U,     _U, \
	_U,     _U,     _U,     _P,     _P,     _P,     _P,     _P, \
	_P,     _L|_X,  _L|_X,  _L|_X,  _L|_X,  _L|_X,  _L|_X,  _L, \
	_L,     _L,     _L,     _L,     _L,     _L,     _L,     _L, \
	_L,     _L,     _L,     _L,     _L,     _L,     _L,     _L, \
	_L,     _L,     _L,     _P,     _P,     _P,     _P,     _C

#define _CTYPE_DATA_128_255 \
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

static int error_n = 0;

static const char ctype_table[128 + 256] = {
	_CTYPE_DATA_128_255,
	_CTYPE_DATA_0_127,
	_CTYPE_DATA_128_255
};
const char *__ctype_ptr__ = ctype_table + 127;

static int _putc_sprintf(void *param, char c)
{
	*(*((char**)param))++ = c;
	return 0;
}

static int _puts_sprintf(void *param, const char *s)
{
	char **p = (char**)param;

	strcpy(*p, s);
	*p += strlen(s);
	return 0;
}

int *__errno()
{
	return &error_n;
}

int sprintf(char *dst, const char *fmt, ...)
{
	char *p = dst;
	va_list arg_list;

	va_start(arg_list, fmt);
	vprintf_core(fmt, arg_list, _putc_sprintf, _puts_sprintf, &p);
	va_end(arg_list);
	*p = '\0';

	return p - dst;
}

char *strcat(char *dst, const char *src)
{
	char *ret = dst;

	for (; *dst; ++dst);
	while ((*dst++ = *src++));

	return ret;
}

char *strchr(const char *s, int c)
{
	for (; *s && *s != c; s++);
	return (*s == c) ? (char *)s : NULL;
}

int strcmp(const char *a, const char *b) __attribute__ ((naked));
int strcmp(const char *a, const char *b)
{
	asm(
	"strcmp_lop:                \n"
	"   ldrb    r2, [r0],#1     \n"
	"   ldrb    r3, [r1],#1     \n"
	"   cmp     r2, #1          \n"
	"   it      hi              \n"
	"   cmphi   r2, r3          \n"
	"   beq     strcmp_lop      \n"
		"       sub     r0, r2, r3      \n"
	"   bx      lr              \n"
		:::
	);
}

char *strcpy(char *dest, const char *src)
{
	const char *s = src;
	char *d = dest;
	while ((*d++ = *s++));
	return dest;
}

size_t strlen(const char *s) __attribute__ ((naked));
size_t strlen(const char *s)
{
	asm(
		"       sub  r3, r0, #1                 \n"
	"strlen_loop:               \n"
		"       ldrb r2, [r3, #1]!              \n"
		"       cmp  r2, #0                             \n"
	"   bne  strlen_loop        \n"
		"       sub  r0, r3, r0                 \n"
		"       bx   lr                                 \n"
		:::
	);
}

int strncmp(const char *a, const char *b, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		if (a[i] != b[i])
			return a[i] - b[i];

	return 0;
}

char *strncpy(char *dest, const char *src, size_t n)
{
	const char *s = src;
	char *d = dest;
	while (n-- && (*d++ = *s++));
	return dest;
}
