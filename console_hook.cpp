#include "console_hook.h"

void hook_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    std::ostringstream oss;

    while (*format != '\0')
    {
        if (*format == '%')
        {
            ++format;
            switch (*format)
            {
                case 'd':
                {
                    int i = va_arg(args, int);
                    oss << i;
                    break;
                }
                case 'f':
                {
                    double d = va_arg(args, double);
                    oss << d;
                    break;
                }
                case 'c':
                {
                    int c = va_arg(args, int);
                    oss << static_cast<char>(c);
                    break;
                }
                case 's':
                {
                    char *s = va_arg(args, char *);
                    oss << s;
                    break;
                }
                default:
                {
                    oss << '%' << *format;
                    break;
                }
            }
        }
        else
        {
            oss << *format;
        }
        ++format;
    }

    va_end(args);

    // std::cout에 출력
    std::cout << oss.str();
}
