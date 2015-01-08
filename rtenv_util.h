#include <stddef.h>
#include <stdint.h>
#include <limits.h>

void itoa(int num, char *dst, int base);
static char utoa(unsigned int num, char *dst, unsigned int base);
int atoi(const char *argv);
