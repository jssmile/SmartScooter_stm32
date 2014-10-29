#include <ctype.h>
#include <string.h>
#include "clib/__internal__.h"

typedef enum {
	IOFMT_CHAR,
	IOFMT_ERROR,
	IOFMT_INT,
	IOFMT_PTR,
	IOFMT_STR,
	IOFMT_TEXT,
	IOFMT_UINT,
	IOFMT_XINT,
	IOFMT_xINT,
	IOFMT_UNKNOWN
} IOFMT_TOKEN;

typedef struct {
	const char *fw_str;
	IOFMT_TOKEN type;
	char spec;
	char width;
} output_token;

static output_token get_next_output_token(const char *fmt)
{
	output_token ret = {.fw_str = fmt, .type = IOFMT_UNKNOWN, .spec = '\0', .width = 0};

	if (*ret.fw_str == '%') {
		ret.fw_str++;
		if (*ret.fw_str == '-' || *ret.fw_str == '0')
			ret.spec = *ret.fw_str++;
		for (; *ret.fw_str && ret.type == IOFMT_UNKNOWN; ret.fw_str++) {
			switch (*ret.fw_str) {
				case '%':
					if (ret.spec || ret.width > 0) {
						ret.type = IOFMT_ERROR;
						ret.fw_str--;
					}
					else {
						ret.spec = '%';
						ret.type = IOFMT_TEXT;
					}
				break;
				case 'c':
					ret.type = IOFMT_CHAR;
				break;
				case 'd':
				case 'i':
					ret.type = IOFMT_INT;
				break;
				case 'p':
					ret.type = IOFMT_PTR;
				break;
				case 's':
					ret.type = IOFMT_STR;
				break;
				case 'u':
					ret.type = IOFMT_UINT;
				break;
				case 'X':
					ret.type = IOFMT_XINT;
				break;
				case 'x':
					ret.type = IOFMT_xINT;
				break;
				default:
					ret.width = 10 * ret.width + (*ret.fw_str - '0');
				break;
			}
		}
	}
	else {
		ret.type = IOFMT_TEXT;
		ret.spec = *ret.fw_str++;
	}

	return ret;
}

static char *utoa(unsigned int num, char *dst, unsigned int base, int lowercase)
{
	char buf[33] = {0};
	char *p = &buf[32];

	if (num == 0)
		*--p = '0';
	else {
		char *q;

		for (; num; num/=base)
			*--p = "0123456789ABCDEF" [num % base];
		if (lowercase)
			for (q = p; *q; q++)
				*q = (char)tolower((int)*q);
	}

	return strcpy(dst, p);
}

int vprintf_core(const char *fmt, va_list arg_list, putc_t _putc_, puts_t _puts_, void *param)
{
	char buf[12];
	output_token out;
	union {
		int i;
		const char *s;
		unsigned u;
	} argv;

	for (; *fmt; fmt = out.fw_str) {
		out = get_next_output_token(fmt);
		switch (out.type) {
			case IOFMT_CHAR:
				argv.i = va_arg(arg_list, int);
				_putc_(param, (char)argv.i);
				argv.s = NULL;
			break;
			case IOFMT_INT:
				argv.i = va_arg(arg_list, int);
				if (argv.i < 0) {
					buf[0] = '-';
					utoa(-argv.i, buf + 1, 10, 0);
				}
				else
					utoa(argv.i, buf, 10, 0);
				argv.s = buf;
			break;
			case IOFMT_PTR:
				argv.u = va_arg(arg_list, unsigned);
				if (argv.u) {
					strcpy(buf, "0x");
					utoa(argv.u, buf + 2, 16, 1);
					argv.s = buf;
				}
				else
					argv.s = "(nil)";
			break;
			case IOFMT_STR:
				argv.s = va_arg(arg_list, const char *);
				if (!argv.s)
					argv.s = "(null)";
			break;
			case IOFMT_TEXT:
				_putc_(param, out.spec);
				argv.s = NULL;
			break;
			case IOFMT_UINT:
				argv.u = va_arg(arg_list, unsigned);
				utoa(argv.u, buf, 10, 0);
				argv.s = buf;
			break;
			case IOFMT_XINT:
				argv.u = va_arg(arg_list, unsigned);
				utoa(argv.u, buf, 16, 0);
				argv.s = buf;
			break;
			case IOFMT_xINT:
				argv.u = va_arg(arg_list, unsigned);
				utoa(argv.u, buf, 16, 1);
				argv.s = buf;
			break;
			default:
				argv.s = NULL;
			break;
		}
		if (argv.s) {
			int w = out.width - strlen(argv.s);
			if (out.spec == '-') {
				_puts_(param, argv.s);
				while (w-- > 0) {
					_putc_(param, ' ');
				}
			}
			else {
				if (!out.spec)
					out.spec = ' ';
				while (w-- > 0) {
					_putc_(param, out.spec);
				}
				_puts_(param, argv.s);
			}
		}
	}

	return 0;
}
