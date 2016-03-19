/********************************************************************
 * Description: emcsvr.cc
 *   Network server for EMC NML
 *
 *   Derived from a work by Fred Proctor & Will Shackleford
 *
 * Author:
 * License: GPL Version 2
 * System: Linux
 *    
 * Copyright (c) 2004 All rights reserved.
 *
 * Last change:
 ********************************************************************/

#include <stdio.h>		// sscanf()
#include "rtapi_math.h"		// fabs()
#include <stdlib.h>		// exit()
#include <string.h>		// strncpy()
#include <unistd.h>             // _exit()
#include <signal.h>

#include "rcs.hh"		// EMC NML
#include "emc.hh"		// EMC NML
#include "emc_nml.hh"		// EMC NML
#include "emcglb.h"		// emcGetArgs(), EMC_NMLFILE
#include "inifile.hh"
#include "rcs_print.hh"
#include "nml_oi.hh"
#include "timer.hh"
#include "nml_srv.hh"           // run_nml_servers()

#ifdef ENABLE_LOG_FILE
#include <Log.h>
#include <LogClass.h>
LogClass logObject;
#endif

static int tool_channels = 1;

static int iniLoad(const char *filename)
{
    IniFile inifile;
    const char *inistring;

    // open it
    if (inifile.Open(filename) == false) {
        return -1;
    }

    if (NULL != (inistring = inifile.Find("DEBUG", "EMC"))) {
        // copy to global
        if (1 != sscanf(inistring, "%x", &emc_debug)) {
            emc_debug = 0;
        }
    } else {
        // not found, use default
        emc_debug = 0;
    }
    if (emc_debug & EMC_DEBUG_RCS) {
        set_rcs_print_flag(PRINT_EVERYTHING);
        max_rcs_errors_to_print = -1;
    }

    if (NULL != (inistring = inifile.Find("NML_FILE", "EMC"))) {
        // copy to global
        strcpy(emc_nmlfile, inistring);
        rcs_print("iniLoad src/emc/task/emcsvr.cc emc_nmlfile from initfile (%s)\n", emc_nmlfile);
    } else {
        // not found, use default
        rcs_print("iniLoad src/emc/task/emcsvr.cc using default emc_nmlfile (%s)\n", emc_nmlfile);
    }
    inifile.Find(&tool_channels,"TOOL_CHANNELS","EMC");
    // close it
    inifile.Close();

    return 0;
}

static RCS_CMD_CHANNEL *emcCommandChannel = NULL;
static RCS_STAT_CHANNEL *emcStatusChannel = NULL;
static NML *emcErrorChannel = NULL;
static RCS_CMD_CHANNEL *toolCommandChannel = NULL;
static RCS_STAT_CHANNEL *toolStatusChannel = NULL;
#define EMCSVRLOG "/etc/mlabs/log/emcsvrLog"
int main(int argc, char *argv[])
{
    double start_time;

#ifdef ENABLE_LOG_FILE
    //std::string file("/home/sanjit/Documents/workArea/LINUXCNC/MIRROR/emcsvrLog");
    std::string file(EMCSVRLOG);
    logObject.initializeLog(file);
#endif
    // process command line args
    if (0 != emcGetArgs(argc, argv)) {
        rcs_print("src/emc/task/emcsvr.cc main exit\n");
        rcs_print_error("Error in argument list\n");
        exit(1);
    }
    rcs_print("************************************\n");
    rcs_print("*           EMCSERVER              *\n");
    rcs_print("************************************\n");
    rcs_print("src/emc/task/emcsvr.cc main called\n");
    // get configuration information
    iniLoad(emc_inifile);

    //set_rcs_print_destination(RCS_PRINT_TO_NULL);

    rcs_print("after iniLoad()\n");


    start_time = etime();

    while (rtapi_fabs(etime() - start_time) < 10.0 &&
            (emcCommandChannel == NULL || emcStatusChannel == NULL
             || (tool_channels && (toolCommandChannel == NULL || toolStatusChannel == NULL))
             || emcErrorChannel == NULL)
          ) {
        if (NULL == emcCommandChannel) {
            rcs_print("emcCommandChannel==NULL, attempt to create\n");
            emcCommandChannel =
                new RCS_CMD_CHANNEL(emcFormat, "emcCommand", "emcsvr",
                        emc_nmlfile);
        }
        if (NULL == emcStatusChannel) {
            rcs_print("emcStatusChannel==NULL, attempt to create\n");
            emcStatusChannel =
                new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "emcsvr",
                        emc_nmlfile);
        }
        if (NULL == emcErrorChannel) {
            rcs_print("emcErrorChannel==NULL, attempt to create\n");
            emcErrorChannel =
                new NML(nmlErrorFormat, "emcError", "emcsvr", emc_nmlfile);
        }
        if (tool_channels) {
            if (NULL == toolCommandChannel) {
            rcs_print("toolCommandChannel==NULL, attempt to create\n");
                toolCommandChannel =
                    new RCS_CMD_CHANNEL(emcFormat, "toolCmd", "emcsvr",
                            emc_nmlfile);
            }
            if (NULL == toolStatusChannel) {
            rcs_print("toolStatusChannel==NULL, attempt to create\n");
                toolStatusChannel =
                    new RCS_STAT_CHANNEL(emcFormat, "toolSts", "emcsvr",
                            emc_nmlfile);
            }
        }

        if (!emcCommandChannel->valid()) {
            delete emcCommandChannel;
            emcCommandChannel = NULL;
        }
        if (!emcStatusChannel->valid()) {
            delete emcStatusChannel;
            emcStatusChannel = NULL;
        }
        if (!emcErrorChannel->valid()) {
            delete emcErrorChannel;
            emcErrorChannel = NULL;
        }
        if (tool_channels) {
            if (!toolCommandChannel->valid()) {
                delete toolCommandChannel;
                toolCommandChannel = NULL;
            }
            if (!toolStatusChannel->valid()) {
                delete toolStatusChannel;
                toolStatusChannel = NULL;
            }
        }
        esleep(0.200);
    }

    set_rcs_print_destination(RCS_PRINT_TO_STDERR);
    // why creating two times ???
    if (NULL == emcCommandChannel) {
        emcCommandChannel =
            new RCS_CMD_CHANNEL(emcFormat, "emcCommand", "emcsvr",
                    emc_nmlfile);
    }
    if (NULL == emcStatusChannel) {
        emcStatusChannel =
            new RCS_STAT_CHANNEL(emcFormat, "emcStatus", "emcsvr",
                    emc_nmlfile);
    }
    if (NULL == emcErrorChannel) {
        emcErrorChannel =
            new NML(nmlErrorFormat, "emcError", "emcsvr", emc_nmlfile);
    }
    if (tool_channels) {
        if (NULL == toolCommandChannel) {
            toolCommandChannel =
                new RCS_CMD_CHANNEL(emcFormat, "toolCmd", "emcsvr",
                        emc_nmlfile);
        }
        if (NULL == toolStatusChannel) {
            toolStatusChannel =
                new RCS_STAT_CHANNEL(emcFormat, "toolSts", "emcsvr",
                        emc_nmlfile);
        }
    }
    run_nml_servers();

    return 0;
}

// vim:sw=4:sts=4:et
