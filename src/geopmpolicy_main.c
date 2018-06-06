/*
 * Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <getopt.h>
#include <libgen.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "geopm_policy.h"
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include "geopm_platform_io.h"
#include "geopm_platform_topo.h"
#include "geopm_version.h"
#include "geopm_error.h"
#include "config.h"

#ifdef __INTEL_COMPILER
#pragma warning disable 1786
#else   /// GNU build
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

enum geopmpolicy_const {
    GEOPMPOLICY_EXEC_MODE_CREATE = 0,
    GEOPMPOLICY_EXEC_MODE_ENFORCE = 1,
    GEOPMPOLICY_EXEC_MODE_SAVE = 2,
    GEOPMPOLICY_EXEC_MODE_RESTORE = 3,
    GEOPMPOLICY_EXEC_MODE_WHITELIST = 4,
    GEOPMPOLICY_STRING_LENGTH = 128,
};


static int _geopm_policy_mode_parse(struct geopm_policy_c *policy, const char *mode_str);
static int _geopm_policy_dict_parse(struct geopm_policy_c *policy, const char *options);

///@todo free memory
///@todo need equiv in gepm_platform_topo.h
#define M_NUM_DOMAIN 11///src/PlatformTopo.hpp
int test_platform_topo()
{
    int domain_type, domain_type2;
    int num_domain = 0;
    int domain_idx, domain_idx2;
    int num_cpu = 0;
    int cur_cpu_idx;
    int *cpu_idx = NULL;
    bool is_within;
    const int str_len = 512;//NAME_MAX;
    char domain_name[str_len];
    int ret = 0;
    for (int inner_domain = 1; inner_domain < M_NUM_DOMAIN; ++inner_domain) {
        for (int outer_domain = M_NUM_DOMAIN - 1; outer_domain >= 0; --outer_domain) {
            ret = geopm_platform_topo_is_domain_within(inner_domain, outer_domain, &is_within);
            ///@todo assert proper is_within val
        }
    }
    for (domain_type = 0; domain_type < M_NUM_DOMAIN; ++domain_type) {
        ret = geopm_platform_topo_num_domain(domain_type, &num_domain);
        if (domain_type == 0) {
            /// assert ret != 0
            continue;
        }
        for (domain_idx = 0; domain_idx < num_domain; ++domain_idx) {
            if (!ret) {
                ret = geopm_platform_topo_domain_cpus(domain_type, domain_idx, &num_cpu, &cpu_idx);
            }
            for (cur_cpu_idx = 0; cur_cpu_idx < num_cpu; ++cur_cpu_idx) {
                if (!ret) {
                    ret = geopm_platform_topo_domain_idx(domain_type, *(cpu_idx + cur_cpu_idx), &domain_idx2);
                    /// assert domain_idx == domain_idx2
                }
            }
        }
        if (!ret) {
            ///@todo return str_len?
            ret = geopm_platform_topo_domain_type_to_name(domain_type, str_len, domain_name);
        }
        if (!ret) {
            ret = geopm_platform_topo_domain_name_to_type(strlen(domain_name), domain_name, &domain_type2);
            /// assert domain_type == domain_type2
        }
#if 0
    if (!ret) {
        ret = geopm_platform_topo_define_cpu_group(int num_cpu, const int *cpu_domain_idx, int *cpu_group_idx);
    }
#endif
    }
    return ret;
}

int test_platform_io_controls()
{
    int ret = 0;
    int domain_type;
    int num_domain;
    int dom_idx;
    int push_idx;
    int num_controls;
    int num_pushed_controls;
    int *control_lens = NULL;
    char **control_names = NULL;
    int ctl_idx;
    ret = geopm_platform_io_save_control();
    ret = geopm_platform_io_control_names(&num_controls, &control_lens, &control_names);
    if (!ret)
        for (ctl_idx = 0; ctl_idx < num_controls; ++ctl_idx) {
            printf("%s\n", control_names[ctl_idx]);
            ret = geopm_platform_io_control_domain_type(control_lens[ctl_idx], control_names[ctl_idx], &domain_type);
            if (!ret) {
                ret = geopm_platform_topo_num_domain(domain_type, &num_domain);
                /// @todo?
            }
            if (!ret) {
                for (dom_idx = 0; dom_idx < num_domain; ++dom_idx) {
                    if (!ret) {
                        ret = geopm_platform_io_push_control(control_lens[ctl_idx], control_names[ctl_idx], domain_type, dom_idx, &push_idx);
                    }
                    if (!ret) {
                        ///@todo write good value?  thinking figuring out a valid value per ctl is integration-y as controls in general are not
                        ///      assured unless on a valid platform
                        //ret = geopm_platform_io_write_control(control_lens[ctl_idx], control_names[ctl_idx], domain_type, dom_idx, 0.0);
                    }
                    if (!ret) {
                        ///@todo write good value?  thinking figuring out a valid value per ctl is integration-y as controls in general are not
                        ///      assured unless on a valid platform
                        //ret = geopm_platform_io_adjust(push_idx, 0.0);
                    }
                }
            }
        }
    ret = geopm_platform_io_num_pushed_control(&num_pushed_controls);
    //ret = geopm_platform_io_write_batch();

    geopm_platform_io_restore_control();
    return ret;
}

int test_platform_io_signals()
{
    int ret = 0;
    int domain_type;
    int num_domain;
    int dom_idx;
    int num_signals;
    int num_pushed_signals;
    int push_idx;
    /// @todo determine whether or not this feature is targeted for C interfaces
    //const int num_sub_signal = 2;
    //const char *comb_sig_name = "SIGNAL1 + SIGNAL2";
    //const int comb_sig_name_len = strlen(comb_sig_name);
    //int sub_sig_idx[num_sub_signal];
    //int comb_sig_idx;
    int *signal_lens = NULL;
    char **signal_names = NULL;
    int sig_idx;
    ret = geopm_platform_io_signal_names(&num_signals, &signal_lens, &signal_names);
    if (!ret) {
        for (sig_idx = 0; sig_idx < num_signals; ++sig_idx) {
            printf("%s\n", signal_names[sig_idx]);
            ret = geopm_platform_io_signal_domain_type(signal_lens[sig_idx], signal_names[sig_idx], &domain_type);
            if (!ret) {
                ret = geopm_platform_topo_num_domain(domain_type, &num_domain);
            }
            for (dom_idx = 0; dom_idx < num_domain; ++dom_idx) {
                if (!ret) {
                    ret = geopm_platform_io_push_signal(signal_lens[sig_idx], signal_names[sig_idx], domain_type, dom_idx, &push_idx);
                }
            }
            //if (domain_type == 1 && num_signals >= 2 && sig_idx == 0) {
                //sub_sig_idx[sig_idx] = push_idx;
            //}
            //else if (domain_type == 1 && num_signals >= 2 && sig_idx == 1) {
                //sub_sig_idx[sig_idx] = push_idx;
            //}
        }
    }
    //if (num_signals >= 2) {
        //ret = geopm_platform_io_push_combined_signal(comb_sig_name_len, comb_sig_name, domain_type, 0,
                //num_sub_signal, sub_sig_idx, &comb_sig_idx);
    //}
    ret = geopm_platform_io_num_pushed_signal(&num_pushed_signals);
#if 0
int geopm_platform_io_sample(int signal_idx, double *value);
int geopm_platform_io_read_batch(void);
int geopm_platform_io_read_signal(int signal_str_len, const char *signal_name, int domain_type, int domain_idx, double *value);
#endif
    return ret;
}

int main(int argc, char** argv)
{
    int opt = 0;
    int err = 0;
    int exec_mode = 0;
    char file[GEOPMPOLICY_STRING_LENGTH] = {0};
    char mode_string[GEOPMPOLICY_STRING_LENGTH] = {0};
    char option_string[GEOPMPOLICY_STRING_LENGTH] = {0};
    char copy_string[GEOPMPOLICY_STRING_LENGTH] = {0};
    char error_string[GEOPMPOLICY_STRING_LENGTH] = {0};
    FILE *infile = NULL;
    FILE *outfile = NULL;
    char *arg_ptr = NULL;
    struct geopm_policy_c *policy = NULL;

    const char *usage = "   geopmpolicy --version | --help\n"
                        "   geopmpolicy -c -f output -m mode -d key0:value0,key1:value1...\n"
                        "   geopmpolicy -e (-f input | -m mode -d key0:value0,key1:value1...)\n"
                        "   geopmpolicy -s [-f output]\n"
                        "   geopmpolicy -r [-f input]\n"
                        "   geopmpolicy -w [-f output]\n"
                        "\n"
                        "   --version\n"
                        "      Print version of geopm to standard file, then exit.\n"
                        "\n"
                        "   --help\n"
                        "       Print  brief   summary  of   the  command   line  usage\n"
                        "       information, then exit.\n"
                        "\n"
                        "   -c\n"
                        "       Create a geopm(3) configuration file, -f must be specified\n"
                        "       when using this option which gives the path to the output\n"
                        "       configuration file.\n"
                        "\n"
                        "   -e\n"
                        "       Enforce a static power mode, this mode can be specified\n"
                        "       with the -m and -d options or the -f option.\n"
                        "\n"
                        "   -s\n"
                        "       Create an in MSR save state file for all MSR values that\n"
                        "       geopm(3)  may modify.  The file can be specified with -f\n"
                        "       and will be stored in /tmp (default is .geopm_msr_restore.log).\n"
                        "       If -f is used, the output file will also be in /tmp.\n"
                        "\n"
                        "   -r\n"
                        "       Restore the MSR values that are recorded in an existing\n"
                        "       MSR save state file.  The input file can be  specified\n"
                        "       with the -f option.\n"
                        "\n"
                        "   -w\n"
                        "       Create a Linux MSR driver whitelist file for the current\n"
                        "       platform, -f must  be  specified when using this option which\n"
                        "       gives the path to the output whitelist file.\n"
                        "\n"
                        "   -m mode\n"
                        "       Power management mode, must be one of those described\n"
                        "       in the MODES section of geopmpolicy(3). The static modes do not\n"
                        "       require the geopm runtime to be running concurrently\n"
                        "       with the primary computational application, where as\n"
                        "       dynamic modes do have a runtime requirement on geopm.\n"
                        "\n"
                        "   -d key0:value0,key1:value1...\n"
                        "       Specifies a dictionary of key value pairs which modify\n"
                        "       the behavior of a mode. The key and value options for each\n"
                        "       mode are described in the MODES sections of geopmpolicy(3).\n"
                        "\n"
                        "   -f file_path\n"
                        "       When used with -c or -s file_path is an output file.  When\n"
                        "       used with -e or -r file_path is an input file.  This is a\n"
                        "       geopm(3) configuration file when used with -c or -e and an\n"
                        "       MSR save state file when used with -s or -r.\n"
                        "\n"
                        "     Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation. All rights reserved.\n"
                        "\n";

    err = test_platform_topo();
    if (!err) {
        err = test_platform_io_controls();
    }
    //if (!err) {
        err = test_platform_io_signals();
    //}
    return err;

    if (argc < 2) {
        fprintf(stderr, "Error: No arguments specified\n");
        fprintf(stderr, usage, argv[0]);
        return EINVAL;
    }
    if (strncmp(argv[1], "--version", strlen("--version") + 1) == 0) {
        printf("%s\n",geopm_version());
        printf("\n\nCopyright (c) 2015, 2016, 2017, 2018, Intel Corporation. All rights reserved.\n\n");
        return 0;
    }
    if (strncmp(argv[1], "--help", strlen("--help") + 1) == 0) {
        printf("%s\n",usage);
        return 0;
    }

    while (!err && (opt = getopt(argc, argv, "hcesrwm:d:f:")) != -1) {
        arg_ptr = NULL;
        switch (opt) {
            case 'c':
                exec_mode = GEOPMPOLICY_EXEC_MODE_CREATE;
                break;
            case 'e':
                exec_mode = GEOPMPOLICY_EXEC_MODE_ENFORCE;
                break;
            case 's':
                exec_mode = GEOPMPOLICY_EXEC_MODE_SAVE;
                break;
            case 'r':
                exec_mode = GEOPMPOLICY_EXEC_MODE_RESTORE;
                break;
            case 'w':
                exec_mode = GEOPMPOLICY_EXEC_MODE_WHITELIST;
                break;
            case 'm':
                arg_ptr = mode_string;
                break;
            case 'd':
                arg_ptr = option_string;
                break;
            case 'f':
                arg_ptr = file;
                break;
            case 'h':
                printf("%s\n",usage);
                return 0;
            default:
                fprintf(stderr, "Error: unknown parameter \"%c\"\n", opt);
                fprintf(stderr, usage, argv[0]);
                err = EINVAL;
                break;
        }
        if (!err && optarg != NULL && arg_ptr != NULL) {
            strncpy(arg_ptr, optarg, GEOPMPOLICY_STRING_LENGTH);
            if (arg_ptr[GEOPMPOLICY_STRING_LENGTH - 1] != '\0') {
                fprintf(stderr, "Error: option string too long\n");
                err = EINVAL;
            }
        }
    }

    if (!err && optind != argc) {
        fprintf(stderr, "Error: %s does not take positional arguments\n", argv[0]);
        fprintf(stderr, usage, argv[0]);
        err = EINVAL;
    }

    if (!err && (exec_mode == GEOPMPOLICY_EXEC_MODE_CREATE &&
                 (strlen(mode_string) == 0 || strlen(option_string) == 0))) {
        fprintf(stderr, "Error: In execute mode create, -m and -d are not optional\n");
        err = EINVAL;
    }

    if (!err && (exec_mode == GEOPMPOLICY_EXEC_MODE_ENFORCE && strlen(file) == 0 &&
                 (strlen(mode_string) == 0 || strlen(option_string) == 0))) {
        fprintf(stderr, "Error: In execute mode enforce, either -i or -m and -d must be specified\n");
        err = EINVAL;
    }

    if (!err && exec_mode == GEOPMPOLICY_EXEC_MODE_ENFORCE && strlen(file) > 0) {
        infile = fopen(file, "r");
        if (infile == NULL) {
            fprintf(stderr, "Error: Cannot open specified file for reading: %s\n", file);
            err = EINVAL;
        }
        else {
            fclose(infile);
        }
    }

    if (!err && exec_mode == GEOPMPOLICY_EXEC_MODE_RESTORE) {
        if (strlen(file) == 0) {
            snprintf(file, GEOPMPOLICY_STRING_LENGTH, "/tmp/.geopm_msr_restore.log");
        }
        else {
            //Make sure we are using tempfs to keep these files local to the machine
            if (strncmp(file, "/tmp/", 5)) {
                if (strlen(file) >= (GEOPMPOLICY_STRING_LENGTH - strlen("/tmp/"))) {
                    fprintf(stderr, "Error: Specified file path too long\n");
                    err = EINVAL;
                }
                if (!err) {
                    if (file[0] == '/') {
                        snprintf(copy_string, GEOPMPOLICY_STRING_LENGTH, "/tmp%s", file);
                        strncpy(file, copy_string, GEOPMPOLICY_STRING_LENGTH);
                    }
                    else {
                        snprintf(copy_string, GEOPMPOLICY_STRING_LENGTH, "/tmp/%s", file);
                        strncpy(file, copy_string, GEOPMPOLICY_STRING_LENGTH);
                    }
                }
            }
        }
        if (!err) {
            infile = fopen(file, "r");
        }
        if (infile == NULL) {
            fprintf(stderr, "Error: Cannot open file for reading: %s\n", file);
            err = EINVAL;
        }
        else {
            fclose(infile);
        }
    }


    if (!err && exec_mode == GEOPMPOLICY_EXEC_MODE_CREATE) {
        outfile = fopen(file, "w");
        if (outfile == NULL) {
            fprintf(stderr, "Error: Cannot open specified file for writing: %s\n", file);
            err = EINVAL;
        }
        else {
            fclose(outfile);
        }
    }

    if (!err && exec_mode == GEOPMPOLICY_EXEC_MODE_SAVE) {
        struct stat statbuffer;
        char* pdir;

        if (strlen(file) == 0) {
            snprintf(file, GEOPMPOLICY_STRING_LENGTH, "/tmp/.geopm_msr_restore.log");
        }
        else {
            //Make sure we are using tempfs to keep these files local to the machine
            if (strncmp(file, "/tmp/", 5)) {
                if (strlen(file) > (GEOPMPOLICY_STRING_LENGTH - strlen("/tmp/"))) {
                    fprintf(stderr, "Error: Specified file path too long\n");
                    err = EINVAL;
                }
                if (!err) {
                    if (file[0] == '/') {
                        snprintf(copy_string, GEOPMPOLICY_STRING_LENGTH, "/tmp%s", file);
                        strncpy(file, copy_string, GEOPMPOLICY_STRING_LENGTH);
                    }
                    else {
                        printf("file = %s\n",file);
                        snprintf(copy_string, GEOPMPOLICY_STRING_LENGTH, "/tmp/%s", file);
                        strncpy(file, copy_string, GEOPMPOLICY_STRING_LENGTH);
                        printf("file = %s\n",file);
                    }
                }
            }
        }
        //make sure path exists, if not, create
        pdir = dirname(copy_string);
        for (int i = 1; i < strlen(pdir); i++) {
            if (pdir[i] == '/') {
                pdir[i] = 0;
                if (stat(pdir, &statbuffer) == -1) {
                    if (mkdir(pdir, S_IRWXU)) {
                        fprintf(stderr, "Error: Could not create directory %s\n", dirname(file));
                        return errno ? errno : GEOPM_ERROR_RUNTIME;
                    }
                }
                pdir[i] = '/';
            }
        }
        if (stat(pdir, &statbuffer) == -1) {
            if (mkdir(pdir, S_IRWXU)) {
                fprintf(stderr, "Error: Could not create directory %s\n", dirname(file));
                return errno ? errno : GEOPM_ERROR_RUNTIME;
            }
        }
        outfile = fopen(file, "w");
        if (outfile == NULL) {
            fprintf(stderr, "Error: Cannot open file for writing: %s\n", file);
            err = EINVAL;
        }
        else {
            fclose(outfile);
        }
    }

    if (!err) {
        FILE *fd;
        switch (exec_mode) {
            case GEOPMPOLICY_EXEC_MODE_CREATE:
                err = geopm_policy_create("", file, &policy);
                if (!err) {
                    err = _geopm_policy_mode_parse(policy, mode_string);
                }
                if (!err) {
                    err = _geopm_policy_dict_parse(policy, option_string);
                }
                if (!err) {
                    err = geopm_policy_write(policy);
                }
                if (policy) {
                    (void) geopm_policy_destroy(policy);
                }
                break;
            case GEOPMPOLICY_EXEC_MODE_ENFORCE:
                if (strlen(file) == 0) {
                    err = geopm_policy_create("", "/tmp/geopmpolicy_tmp", &policy);
                    if (!err) {
                        err = _geopm_policy_mode_parse(policy, mode_string);
                    }
                    if (!err) {
                        err = _geopm_policy_dict_parse(policy, option_string);
                    }
                }
                else {
                    err = geopm_policy_create(file, "", &policy);
                }
                if (!err) {
                    err = geopm_policy_enforce_static(policy);
                }
                if (policy) {
                    (void) geopm_policy_destroy(policy);
                }
                break;
            case GEOPMPOLICY_EXEC_MODE_SAVE:
                err = geopm_platform_msr_save(file);
                break;
            case GEOPMPOLICY_EXEC_MODE_RESTORE:
                err = geopm_platform_msr_restore(file);
                break;
            case GEOPMPOLICY_EXEC_MODE_WHITELIST:
                if (strlen(file) == 0) {
                    fd = stdout;
                }
                else {
                    fd = fopen(file, "w");
                    if (fd == NULL) {
                        err = errno ? errno : GEOPM_ERROR_RUNTIME;
                    }
                }
                if (!err) {
                    err = geopm_platform_msr_whitelist(fd);
                    fclose(fd);
                }
                break;
            default:
                fprintf(stderr, "Error: Invalid execution mode.\n");
                err = EINVAL;
                break;
        };
    }

    if (err) {
        geopm_error_message(err, error_string, GEOPMPOLICY_STRING_LENGTH);
        fprintf(stderr, "Error: %s.\n", error_string);
    }
    return err;
}


static int _geopm_policy_mode_parse(struct geopm_policy_c *policy, const char *mode_str)
{
    int err = 0;
    int mode = -1;

    if (strncmp(mode_str, "tdp_balance_static", strlen("tdp_balance_static") + 1) == 0) {
        mode = GEOPM_POLICY_MODE_TDP_BALANCE_STATIC;
    }
    else if (strncmp(mode_str, "freq_uniform_static", strlen("freq_uniform_static") + 1) == 0) {
        mode = GEOPM_POLICY_MODE_FREQ_UNIFORM_STATIC;
    }
    else if (strncmp(mode_str, "freq_hybrid_static", strlen("freq_hybrid_static") + 1) == 0) {
        mode = GEOPM_POLICY_MODE_FREQ_HYBRID_STATIC;
    }
    else if (strncmp(mode_str, "perf_balance_dynamic", strlen("perf_balance_dynamic") + 1) == 0) {
        mode = GEOPM_POLICY_MODE_PERF_BALANCE_DYNAMIC;
    }
    else if (strncmp(mode_str, "freq_uniform_dynamic", strlen("freq_uniform_dynamic") + 1) == 0) {
        mode = GEOPM_POLICY_MODE_FREQ_UNIFORM_DYNAMIC;
    }
    else if (strncmp(mode_str, "freq_hybrid_dynamic", strlen("freq_hybrid_dynamic") + 1) == 0) {
        mode = GEOPM_POLICY_MODE_FREQ_HYBRID_DYNAMIC;
    }
    else if (strncmp(mode_str, "dynamic", strlen("dynamic") + 1) == 0) {
        mode = GEOPM_POLICY_MODE_DYNAMIC;
    }
    else if (strncmp(mode_str, "static", strlen("static") + 1) == 0) {
        mode = GEOPM_POLICY_MODE_STATIC;
    }
    else {
        fprintf(stderr, "Error: Invalid power mode: %s\n", mode_str);
        err = EINVAL;
    }
    if (!err) {
        geopm_policy_mode(policy, mode);
    }

    return err;
}

static int _geopm_policy_dict_parse(struct geopm_policy_c *policy, const char *options)
{
    int err = 0;
    char options_cpy[GEOPMPOLICY_STRING_LENGTH] = {0};
    char *key, *value;

    strncpy(options_cpy, options, GEOPMPOLICY_STRING_LENGTH);
    if (options_cpy[GEOPMPOLICY_STRING_LENGTH - 1] != '\0') {
        err = EINVAL;
    }
    if (!err) {
        key = strtok(options_cpy, ":");
        if (key == NULL) {
            fprintf(stderr, "Error: Invalid execution mode.\n");
            err = EINVAL;
        }
        do {
            value = strtok(NULL, ",");
            if (value == NULL) {
                fprintf(stderr, "Error: Invalid execution mode.\n");
                err = EINVAL;
            }
            if (!err) {
                if (strncmp(key, "tdp_percent", strlen("tdp_percent") + 1) == 0) {
                    err = geopm_policy_tdp_percent(policy, atof(value));
                }
                else if (strncmp(key, "cpu_hz", strlen("cpu_hz") + 1) == 0) {
                    err = geopm_policy_cpu_freq(policy, atof(value));
                }
                else if (strncmp(key, "num_cpu_max_perf", strlen("num_cpu_max_perf") + 1) == 0) {
                    err = geopm_policy_full_perf(policy, atoi(value));
                }
                else if (strncmp(key, "affinity", strlen("affinity") + 1) == 0) {
                    if (strncmp(value, "compact", strlen("compact") + 1) == 0) {
                        err = geopm_policy_affinity(policy, GEOPM_POLICY_AFFINITY_COMPACT);
                    }
                    else if (strncmp(value, "scatter", strlen("scatter") + 1) == 0) {
                        err = geopm_policy_affinity(policy, GEOPM_POLICY_AFFINITY_SCATTER);
                    }
                    else {
                        fprintf(stderr, "Error: invalid affinity value: %s\n", value);
                        err = EINVAL;
                    }
                }
                else if (strncmp(key, "power_budget", strlen("power_budget") + 1) == 0) {
                    err = geopm_policy_power(policy, atoi(value));
                }
                else if (strncmp(key, "tree_decider", strlen("tree_decider") + 1) == 0) {
                    err = geopm_policy_tree_decider(policy, value);
                }
                else if (strncmp(key, "leaf_decider", strlen("leaf_decider") + 1) == 0) {
                    err = geopm_policy_leaf_decider(policy, value);
                }
                else if (strncmp(key, "platform", strlen("platform") + 1) == 0) {
                    err = geopm_policy_platform(policy, value);
                }
                else {
                    fprintf(stderr, "Error: invalid option: %s\n", key);
                    err = EINVAL;
                }
            }
        }
        while (!err && ((key = strtok(NULL, ":")) != NULL));
    }

    return err;
}
