#ifndef _BACKTRACE_H
#define _BACKTRACE_H

#include <stdarg.h>

// the handler for printing backtrace lines
typedef void (*btprint_t)(const char *prefix, const char *fmt, ...);
typedef int (*btprint1_t)(const char *fmt, ...);

#ifdef __cplusplus
extern "C" {
#endif

    void backtrace_init(const char *name);
    void custom_backtrace(const char *prefix, const char *header,
		   btprint_t print, int skip);
    void custom_backtrace_1(const char *header, btprint1_t print);
    int setstacktracemap(char* pidname);

#ifdef __cplusplus
}
#endif

#endif // _BACKTRACE_H
