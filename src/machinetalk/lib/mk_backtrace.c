#include "mk-backtrace.h"

#include "config.h"

#define HAVE_LIBBACKTRACE
#define BACKTRACE_SUPPORTED
#if defined(HAVE_LIBBACKTRACE)

// custom libbacktrace install
#if  defined(HAVE_LIBBACKTRACE_BACKTRACE_H)
#include <libbacktrace/backtrace.h>
#include <libbacktrace/backtrace-supported.h>
#endif

// gcc-bundled install
#if defined(HAVE_BACKTRACE_H)
#include <backtrace.h>
#include <backtrace-supported.h>
#endif

#if defined(BACKTRACE_SUPPORTED)

#include <string.h>
#include <stdint.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/syscall.h>

#define MLAB_CHANGES

#if !defined(MLAB_CHANGES)
struct bt_ctx {
    struct backtrace_state *state;
    int error;
    btprint_t print;
    const char *prefix;
};

static void error_callback(void *data, const char *msg, int errnum)
{
    struct bt_ctx *ctx = data;
    ctx->print(ctx->prefix, "ERROR decoding backtrace: %s (%d)", msg, errnum);
    ctx->error = 1;
}

static void syminfo_callback (void *data, uintptr_t pc,
			      const char *symname,
			      uintptr_t symval, uintptr_t symsize)
{
    struct bt_ctx *ctx = data;

    if (symname) {
	ctx->print(ctx->prefix, "%lx %s ??:0", (unsigned long)pc, symname);
    } else {
	ctx->print(ctx->prefix, "%lx ?? ??:0", (unsigned long)pc);
    }
}

static int full_callback(void *data, uintptr_t pc, const char *filename, int lineno, const char *function)
{
    struct bt_ctx *ctx = data;
    if (function) {
	ctx->print(ctx->prefix, "%-8.8lx %-16.16s (%s:%d)",
		   (unsigned long)pc,
		   function,
		   filename ? filename : "??",
		   lineno);
    } else {
	backtrace_syminfo (ctx->state, pc,
			   syminfo_callback,
			   error_callback, data);
    }
    return 0;
}

static int simple_callback(void *data, uintptr_t pc)
{
    struct bt_ctx *ctx = data;
    backtrace_pcinfo(ctx->state, pc,
		     full_callback,
		     error_callback, data);
    return 0;
}

static struct backtrace_state *state;

void backtrace_init(const char *name)
{
    state = backtrace_create_state (name, BACKTRACE_SUPPORTS_THREADS,
				    error_callback, NULL);
}
#endif
#define BT_BUF_SIZE 1024

int setstacktracemap(char* pidname)
{
    char buff[64];
    int ret;

    sprintf(buff, "cat /proc/%d/maps > /etc/mlabs/log/%s.map", getpid(), pidname);

    ret = system(buff);
    return ret;
}
void custom_backtrace(const char *prefix, const char *header, btprint_t print, int skip)
{
    int nptrs;
    void *buffer[BT_BUF_SIZE];
    char **strings;
    int j;
    pid_t tid = (pid_t) syscall (SYS_gettid);

    if (prefix == NULL)
	prefix = "";
#if !defined(MLAB_CHANGES)
    struct bt_ctx ctx = {state, 0, print, prefix};
#endif
#if defined(MLAB_CHANGES)
    nptrs = backtrace(buffer, BT_BUF_SIZE);
    print(prefix, "backtrace() returned %d addresses\n", nptrs);
    if (header && strlen(header))
	print(prefix,  " --- %s backtrace start %d ---\n", header, tid);
    for(j = 0; j < nptrs; ++j){
	print(prefix, "0x%lx\n", buffer[j]);
    }
#if 0
    strings = backtrace_symbols(buffer, nptrs);
    if (strings == NULL){
	print(prefix, "backtrace_symbols returns NULL");
	return;
    }
    print(prefix, "backtrace symbols");
    for(j = 0; j < nptrs; ++j){
	print(prefix, "%s", strings[j]);
    }
    free(strings);
#endif
#else
    backtrace_simple(state, skip, simple_callback, error_callback, &ctx);
#endif
    if (header && strlen(header))
	print(prefix,  " --- %s backtrace end ---\n", header);
}


void custom_backtrace_1(const char *header, btprint1_t print)
{
    int nptrs;
    void *buffer[BT_BUF_SIZE];
    char **strings;
    int j;
    pid_t tid = (pid_t) syscall (SYS_gettid);


    nptrs = backtrace(buffer, BT_BUF_SIZE);
    print("backtrace() returned %d addresses\n", nptrs);
    if (header && strlen(header))
	print(" --- %s backtrace start %d ---\n", header, tid);
    for(j = 0; j < nptrs; ++j){
	print("0x%lx\n", buffer[j]);
    }
    if (header && strlen(header))
	print(" --- %s backtrace end ---\n", header);
#if 0
    strings = backtrace_symbols(buffer, nptrs);
    if (strings == NULL){
	print("backtrace_symbols returns NULL\n");
	return;
    }
    print("backtrace symbols\n");
    for(j = 0; j < nptrs; ++j){
	print("%s\n", strings[j]);
    }
    free(strings);
#endif
}

#endif // BACKTRACE_SUPPORTED
#endif // HAVE_LIBBACKTRACE

#if !defined(BACKTRACE_SUPPORTED) || ! defined(HAVE_LIBBACKTRACE)

// dummy versions
void backtrace_init(const char *name) {}
void backtrace(const char *prefix,const char *header, btprint_t print, int skip)
{
    print(prefix,
	  "(backtrace not available - libbacktrace not found during build)");
}

#endif
