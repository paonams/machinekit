/* Copyright (C) 2007 Jeff Epler <jepler@unpythonic.net>
 * Copyright (C) 2003 John Kasunich
 *                     <jmkasunich AT users DOT sourceforge DOT net>
 *
 *  Other contributers:
 *                     Martin Kuhnle
 *                     <mkuhnle AT users DOT sourceforge DOT net>
 *                     Alex Joni
 *                     <alex_joni AT users DOT sourceforge DOT net>
 *                     Benn Lipkowitz
 *                     <fenn AT users DOT sourceforge DOT net>
 *                     Stephen Wille Padnos
 *                     <swpadnos AT users DOT sourceforge DOT net>
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of version 2 of the GNU General
 *  Public License as published by the Free Software Foundation.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 USA
 *
 *  THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
 *  ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
 *  TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
 *  harming persons must have provisions for completely removing power
 *  from all motors, etc, before persons enter any danger area.  All
 *  machinery must be designed to comply with local and national safety
 *  codes, and the authors of this software can not, and do not, take
 *  any responsibility for such compliance.
 *
 *  This code was written as part of the EMC HAL project.  For more
 *  information, go to www.linuxcnc.org.
 */

#include "config.h"
#include "rtapi.h"
#include "hal.h"
#include "hal_priv.h"
#include "halcmd.h"
#include "halcmd_commands.h"
#include "halcmd_completion.h"
#include "halcmd_rtapiapp.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <fnmatch.h>
#include <search.h>
#include <inifile.h>
#include <sys/types.h>

static int get_input(FILE *srcfile, char *buf, size_t bufsize);
static void print_help_general(int showR);
static int release_HAL_mutex(void);
static int propose_completion(char *all, char *fragment, int start);

static const char *inifile;
static FILE *inifp;
extern char *logpath;
/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

int setstacktracemap(char* pidname)
{
    char buff[64];
    int ret;

    sprintf(buff, "cat /proc/%d/maps > /etc/mlabs/log/%s.map", getpid(), pidname);

    ret = system(buff);
    return ret;
}

int main(int argc, char **argv)
{
    int c, fd;
    int keep_going, retval, errorcount;
    int filemode = 0;
    char *filename = NULL;
    FILE *srcfile = NULL;
    char raw_buf[MAX_CMD_LEN+1];
    int linenumber = 1;
    char *cf=NULL, *cw=NULL, *cl=NULL;
    char *uri = NULL; // NULL - use service discovery
    char *service_uuid = NULL; // must have a global uuid
    int ret;

    inifile = getenv("MACHINEKIT_INI");
    inifile = "/etc/mlabs/build/machinekit/etc/linuxcnc/machinekit.ini";
    /* use default if not specified by user */
    if (inifile == NULL) {
	inifile = MACHINEKIT_INI;
    }

    if (argc < 2) {
	/* no args specified, print help */
	print_help_general(0);
	exit(0);
    }
    /* set default level of output - 'quiet' */
    rtapi_set_msg_level(RTAPI_MSG_ERR);
    /* set default for other options */
    keep_going = 0;
    /* start parsing the command line, options first */
   fprintf(stderr,"halcmd main '%s'\n", inifile);
   ret = setstacktracemap("halcmd");
   fprintf(stderr, "setstacktracemap ret %d", ret);
    while(1) {
        c = getopt(argc, argv, "+RCfi:kqQsvVhu:U:P");
        if(c == -1) break;
        switch(c) {
            case 'R':
       		/* force an unlock of the HAL mutex - to be used after a segfault in a hal program */
		if (release_HAL_mutex() < 0) {
			printf("HALCMD: Release Mutex failed!\n");
			return 1;
		}
		return 0;
		break;
	    case 'h':
		/* -h = help */
                if (argc > optind) { /* there are more arguments */
                    do_help_cmd(argv[optind]);
                } else {
		    print_help_general(1);
                }
		return 0;
		break;
	    case 'k':
		/* -k = keep going */
		keep_going = 1;
		break;
	    case 'q':
		/* -q = quiet (default) */
		rtapi_set_msg_level(RTAPI_MSG_ERR);
		break;
	    case 'Q':
		/* -Q = very quiet */
		rtapi_set_msg_level(RTAPI_MSG_NONE);
		break;
	    case 's':
		/* script friendly mode */
		scriptmode = 1;
		break;
	    case 'v':
		/* -v = verbose */
		rtapi_set_msg_level(RTAPI_MSG_INFO);
		break;
	    case 'V':
		/* -V = very verbose */
		rtapi_set_msg_level(RTAPI_MSG_ALL);
		break;
	    case 'e':
                echo_mode = 1;
		break;
	    case 'f':
                filemode = 1;
		break;
	    case 'P':
                proto_debug = 1;
		break;
	    case 'C':
                cl = getenv("COMP_LINE");
                cw = getenv("COMP_POINT");
                if (!cl || !cw) exit(0);
                c = atoi(cw)-strlen(argv[0])-1;
                if (c<0) c=0;
                cl += strlen(argv[0])+1;
                if (c>0 && cl[c]=='\0') c--;    // if at end of line, back up one char
                cf=&cl[c];
                {
                    int n;
                    for (n=c; n>0; n--, cf--) {
                        if (isspace(*cf) || *cf == '=' || *cf == '<' || *cf == '>') {
                            cf++;
                            break;
                        }
                    }
                    halcmd_startup(1, uri, service_uuid);
                    propose_completion(cl, cf, n);
                }
                if (comp_id >= 0) halcmd_shutdown();
                exit(0);
                break;
#ifndef NO_INI
	    case 'i':
		/* -i = allow reading 'setp' values from an ini file */
		if (halcmd_inifile == NULL) {
		    /* it's the first -i (ignore repeats) */
                    /* there is a following arg, and it's not an option */
                    filename = optarg;
                    halcmd_inifile = fopen(filename, "r");
                    if (halcmd_inifile == NULL) {
                        fprintf(stderr,
                            "Could not open ini file '%s'\n",
                            filename);
                        exit(-1);
                    }
                    /* make sure file is closed on exec() */
                    fd = fileno(halcmd_inifile);
                    fcntl(fd, F_SETFD, FD_CLOEXEC);
		}
		break;
#endif /* NO_INI */
	    case 'u':
		uri = optarg;
		break;
	    case 'U':
                service_uuid = optarg;
		break;
	    case '?':
		/* option not in getopt() list
		   getopt already printed an error message */
		exit(-1);
		break;
	    default:
		/* option in getopt list but not in switch statement - bug */
		printf("Unimplemented option '-%c'\n", c);
		exit(-1);
		break;
        }
    }

    if (inifile && ((inifp = fopen(inifile,"r")) == NULL)) {
	fprintf(stderr,"halcmd: cant open inifile '%s'\n", inifile);
    }
    // must have a MKUUID - commandline may override
    if (service_uuid == NULL) {
	const char *s;
	if ((s = iniFind(inifp, "MKUUID", "MACHINEKIT"))) {
	    service_uuid = strdup(s);
	}
    }
    if (service_uuid == NULL) {
	fprintf(stderr, "halcmd: no service UUID (-R <uuid> or MACHINEKIT_INI [GLOBAL]MKUUID) present\n");
	exit(-1);
    }

    if(filemode) {
        /* it's the first -f (ignore repeats) */
        if (argc > optind) {
            /* there is a following arg, and it's not an option */
            filename = argv[optind++];
            srcfile = fopen(filename, "r");
            halcmd_set_filename(filename);
            if (srcfile == NULL) {
                fprintf(stderr,
                    "Could not open command file '%s'\n",
                    filename);
                exit(-1);
            }
            /* make sure file is closed on exec() */
            fd = fileno(srcfile);
            fcntl(fd, F_SETFD, FD_CLOEXEC);
        } else {
            halcmd_set_filename("<stdin>");
            /* no filename followed -f option, use stdin */
            srcfile = stdin;
            prompt_mode = 1;
        }
    }

    fprintf(stderr,"halcmd: before halcmd_startup\n");
    if ( halcmd_startup(0, uri, service_uuid) != 0 ) return 1;

    errorcount = 0;
    /* HAL init is OK, let's process the command(s) */
    if (srcfile == NULL) {
        fprintf(stderr,"halcmd: with srcfile == NULL\n");
#ifndef NO_INI
        if(halcmd_inifile) {
            fprintf(stderr, "-i may only be used together with -f\n");
            errorcount++;
        }
#endif
        if(errorcount == 0 && argc > optind) {
            halcmd_set_filename("<commandline>");
            halcmd_set_linenumber(0);
            retval = halcmd_parse_cmd(&argv[optind]);
            if (retval != 0) {
                errorcount++;
            }
        }
    } else {
        fprintf(stderr,"halcmd: with srcfile != NULL\n");
	/* read command line(s) from 'srcfile' */
	while (get_input(srcfile, raw_buf, MAX_CMD_LEN)) {
	    char *tokens[MAX_TOK+1];
	    halcmd_set_linenumber(linenumber++);
	    /* remove comments, do var substitution, and tokenise */
	    retval = halcmd_preprocess_line(raw_buf, tokens);
        if(echo_mode) { 
            halcmd_echo("%s\n", raw_buf);
        }
	    if (retval == 0) {
		/* the "quit" command is not handled by parse_line() */
		if ( ( strcasecmp(tokens[0],"quit") == 0 ) ||
		     ( strcasecmp(tokens[0],"exit") == 0 ) ) {
		    break;
		}
		/* process command */
		retval = halcmd_parse_cmd(tokens);
	    }
	    /* did a signal happen while we were busy? */
	    if ( halcmd_done ) {
		/* treat it as an error */
		errorcount++;
		/* exit from loop */
		break;
	    }
	    if ( retval != 0 ) {
		errorcount++;
	    }
	    if (( errorcount > 0 ) && ( keep_going == 0 )) {
		/* exit from loop */
		break;
	    }
	}
    }
    /* all done */
    halcmd_shutdown();
    if ( errorcount > 0 ) {
	return 1;
    } else {
	return 0;
    }

}

/* release_HAL_mutex() unconditionally releases the hal_mutex
   very useful after a program segfaults while holding the mutex
*/
static int release_HAL_mutex(void)
{
    int comp_id, mem_id, retval;
    void *mem;
    hal_data_t *hal_data;

    /* do RTAPI init */
    comp_id = rtapi_init("hal_unlocker");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: rtapi init failed\n");
        return -EINVAL;
    }
    /* get HAL shared memory block from RTAPI */
    mem_id = rtapi_shmem_new(HAL_KEY, comp_id, global_data->hal_size);
    if (mem_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "ERROR: could not open shared memory\n");
        rtapi_exit(comp_id);
        return -EINVAL;
    }
    /* get address of shared memory area */
    retval = rtapi_shmem_getptr(mem_id, &mem, 0);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "ERROR: could not access shared memory\n");
        rtapi_exit(comp_id);
        return -EINVAL;
    }
    /* set up internal pointers to shared mem and data structure */
    hal_data = (hal_data_t *) mem;
    /* release mutex  */
    rtapi_mutex_give(&(hal_data->mutex));
    /* release RTAPI resources */
    rtapi_shmem_delete(mem_id, comp_id);
    rtapi_exit(comp_id);
    /* done */
    return 0;

}

static char **completion_callback(const char *text, hal_generator_func cb) {
    int state = 0;
    char *s = 0;
    do {
        s = cb(text, state);
        if(s) printf("%s\n", s);
        state = 1;
    } while(s);
    return NULL;
}

/* completion text starts with "halcmd", so remove that from the string */
static int propose_completion(char *all, char *fragment, int start) {
    int sp=0, len=strlen(all), i=0;
    for(; i<len; i++) if(all[i] == ' ') sp = i;
    if(sp) sp++;
    len = strlen(all);
    halcmd_completer(fragment, sp, len, completion_callback, all);
    return 0;
}

static void print_help_general(int showR)
{
    printf("\nUsage:   halcmd [options] [cmd [args]]\n\n");
    printf("\n         halcmd [options] -f [filename]\n\n");
    printf("options:\n\n");
    printf("  -e             echo the commands from stdin to stderr\n");
    printf("  -f [filename]  Read commands from 'filename', not command\n");
    printf("                 line.  If no filename, read from stdin.\n");
#ifndef NO_INI
    printf("  -i filename    Open .ini file 'filename', allow commands\n");
    printf("                 to get their values from ini file.\n");
#endif
    printf("  -k             Keep going after failed command.  Default\n");
    printf("                 is to exit if any command fails. (Useful with -f)\n");
    printf("  -q             Quiet - print errors only (default).\n");
    printf("  -Q             Very quiet - print nothing.\n");
    if (showR != 0) {
    printf("  -R             Release mutex (for crash recovery only).\n");
    }
    printf("  -s             Script friendly - don't print headers on output.\n");
    printf("  -v             Verbose - print result of every command.\n");
    printf("  -V             Very verbose - print lots of junk.\n");
    printf("  -h             Help - print this help screen and exit.\n\n");
    printf("commands:\n\n");
    printf("  loadrt, loadusr, waitusr, unload, lock, unlock, net, linkps, linksp,\n");
    printf("  unlinkp, newsig, delsig, setp, getp, ptype, sets, gets, stype,\n");
    printf("  addf, delf, show, list, save, status, start, stop, source, echo, unecho, quit, exit\n");
    printf("  help           Lists all commands with short descriptions\n");
    printf("  help command   Prints detailed help for 'command'\n\n");
}

#ifdef HAVE_READLINE
#include "halcmd_completion.h"

static int get_input(FILE *srcfile, char *buf, size_t bufsize) {
    static int first_time = 1;
    char *rlbuf;

    if(!scriptmode && srcfile == stdin && isatty(0)) {
        if(first_time) {
            halcmd_init_readline();
            first_time = 0;
        }
        rlbuf = readline("halcmd: ");
        if(!rlbuf) return 0;
        strncpy(buf, rlbuf, bufsize);
        buf[bufsize-1] = 0;
        free(rlbuf);

        if(*buf) add_history(buf);

        return 1;
    }
    if(prompt_mode) {
	    fprintf(stdout, scriptmode ? "%%\n" : "halcmd: "); fflush(stdout);
    }
    return fgets(buf, bufsize, srcfile) != NULL;
}
#else
static int get_input(FILE *srcfile, char *buf, size_t bufsize) {
    if(prompt_mode) {
	    fprintf(stdout, scriptmode ? "%%\n" : "halcmd: "); fflush(stdout);
    }
    return fgets(buf, bufsize, srcfile) != NULL;
}
#endif

void halcmd_output(const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    vfprintf(stdout, format, ap);
    va_end(ap);
}

void halcmd_warning(const char *format, ...) {
    va_list ap;
    fprintf(stderr, "%s:%d: Warning: ", halcmd_get_filename(), halcmd_get_linenumber());
    va_start(ap, format);
    vfprintf(stderr, format, ap);
    va_end(ap);
}

void halcmd_error(const char *format, ...) {
    va_list ap;
    fprintf(stderr, "%s:%d: ", halcmd_get_filename(), halcmd_get_linenumber());
    va_start(ap, format);
    vfprintf(stderr, format, ap);
    va_end(ap);
}

void halcmd_info(const char *format, ...) {
    va_list ap;
    if(rtapi_get_msg_level() < RTAPI_MSG_INFO) return;
    fprintf(stderr, "%s:%d: ", halcmd_get_filename(), halcmd_get_linenumber());
    va_start(ap, format);
    vfprintf(stderr, format, ap);
    va_end(ap);
}

void halcmd_echo(const char *format, ...) {
    va_list ap;
    fprintf(stderr, "(%d)<echo>: ", halcmd_get_linenumber());
    va_start(ap, format);
    vfprintf(stderr, format, ap);
    va_end(ap);
}
