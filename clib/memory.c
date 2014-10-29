#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"

#define ALIGN (sizeof(size_t))
#define ONES ((size_t)-1/UCHAR_MAX)
#define HIGHS (ONES * (UCHAR_MAX/2+1))
#define HASZERO(x) ((x)-ONES & ~(x) & HIGHS)
#define SS (sizeof(size_t))

void free(void *p)
{
	vPortFree(p);
}

void *malloc(size_t size)
{
	return pvPortMalloc(size);
}

void *memset(void *dest, int c, size_t n)
{
	unsigned char *s = (unsigned char*)dest;

	c = (unsigned char)c;
	for (; ((uintptr_t)s & ALIGN) && n; n--)
		*s++ = c;
	if (n) {
		size_t *w, k = ONES * c;

		for (w = (void *)s; n>=SS; n-=SS, w++)
			*w = k;
		for (s = (void *)w; n; n--, s++)
			*s = c;
	}

	return dest;
}

void *memcpy(void *dest, const void *src, size_t n)
{
	void *ret = dest;

	//Cut rear
	uint8_t *dst8 = dest;
	const uint8_t *src8 = src;
	switch (n % 4) {
		case 3 : *dst8++ = *src8++;
		case 2 : *dst8++ = *src8++;
		case 1 : *dst8++ = *src8++;
		case 0 : ;
	}

	//stm32 data bus width
	uint32_t *dst32 = (void *)dst8;
	const uint32_t *src32 = (void *)src8;
	n >>= 2;
	while (n--)
		*dst32++ = *src32++;

	return ret;
}
