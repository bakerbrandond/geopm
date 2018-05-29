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

#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <dlfcn.h>
#ifndef u_short
#define u_short unsigned short
#endif

#include "geopm_env.h"

#include <set>
#include <algorithm>
/// @todo remove
#include <iostream>

#include "config.h"

static int geopm_name_begins_with(std::string str, std::string key)
{
    int result = str.find(key) == 0;
    std::cout << "geopm_name_begins_with(" << str << ", " << key << ") returning: " << result << std::endl;
    return result;
}

static int geopm_name_ends_with(std::string str, std::string key)
{
    std::reverse(str.begin(), str.end());
    std::reverse(key.begin(), key.end());
    int result = geopm_name_begins_with(str, key);
    //std::cout << "geopm_name_ends_with(" << str << ", " << key << ") returning: " << result << std::endl;
    return result;
}

/// @todo make sure to document
/// @todo remember to deprecate legacy plugin interfaces and structs
#define GEOPM_AGENT_PLUGIN_PREFIX "libgeopmagent"
#define GEOPM_IOGROUP_PLUGIN_PREFIX "libgeopmiogroup"
#define GEOPM_COMM_PLUGIN_PREFIX "libgeopmcomm"
static void __attribute__((constructor)) geopmpolicy_load(void)
{
    int err = 0;
    char err_msg[NAME_MAX];
    std::string env_plugin_path_str;
    const char *env_plugin_path = geopm_env_plugin_path();
    DIR *did = NULL;
    std::set<std::string> plugin_paths;
    std::set<std::string> iogroup_plugins;
    std::set<std::string> agent_plugins;
    std::string so_suffix = ".so." GEOPM_ABI_VERSION;

    if (env_plugin_path) {
        for (auto it = so_suffix.begin(); it != so_suffix.end(); ++it) {
            if (*it == ':') {
                so_suffix.replace(it, it + 1, ".");
            }
        }
        env_plugin_path_str = std::string(env_plugin_path);
        size_t start_pos = 0;
        size_t del_pos = env_plugin_path_str.find(":");
        if (del_pos == std::string::npos) {
            plugin_paths.insert(env_plugin_path_str);
        }
        else {
            while (del_pos != std::string::npos) {
                plugin_paths.insert(env_plugin_path_str.substr(start_pos, del_pos));
                start_pos = del_pos + 1;
                env_plugin_path_str = env_plugin_path_str.substr(start_pos, std::string::npos);
                del_pos = env_plugin_path_str.find(":");
            }
            /// add the last path
            plugin_paths.insert(env_plugin_path_str);
        }
        std::cout << plugin_paths.size() << " paths set in env_var" << std::endl;
        for (auto &path : plugin_paths) {
            std::cout << "parsing path: " << path << " {" << std::endl;
            did = opendir(path.c_str());
            if (did) {
                struct dirent *entry;
                char plugin_path[NAME_MAX];
                plugin_path[NAME_MAX - 1] = '\0';
                while ((entry = readdir(did))) {
                    std::cout << " " << entry->d_name;
                    /// @todo add branch here to search for legacy named plugins
                    if (geopm_name_ends_with(entry->d_name, so_suffix) ||
                        geopm_name_ends_with(entry->d_name, ".dylib")) {
                        std::cout << "ends with proper postfix";
                        if (geopm_name_begins_with(std::string(entry->d_name), GEOPM_IOGROUP_PLUGIN_PREFIX)) {
                            iogroup_plugins.insert(path + "/" + std::string(entry->d_name));
                            std::cout << " is an iogroup plugin,";
                        }
                        else if (geopm_name_begins_with(std::string(entry->d_name), GEOPM_AGENT_PLUGIN_PREFIX)) {
                            agent_plugins.insert(path + "/" +std::string(entry->d_name));
                            std::cout << " is an agent plugin,";
                        }
                    }
                }
                closedir(did);
                std::cout << " }" << std::endl;
            }
        }
        /// first load iogroups such that signals and controls are present for agents
        /// @todo how to handle interdependent iogroups (combined signals/controls via plugins)
        for (auto &iogroup : iogroup_plugins) {
            if (dlopen(iogroup.c_str(), RTLD_NOLOAD) == NULL) {
                if (NULL == dlopen(iogroup.c_str(), RTLD_LAZY)) {
#ifdef GEOPM_DEBUG
                    std::cerr << "Warning: failed to dlopen iogroup plugin " << iogroup << std::endl;
#endif
                }
            }
        }
        for (auto &agent : agent_plugins) {
            if (dlopen(agent.c_str(), RTLD_NOLOAD) == NULL) {
                if (NULL == dlopen(agent.c_str(), RTLD_LAZY)) {
#ifdef GEOPM_DEBUG
                    std::cerr << "Warning: failed to dlopen agent plugin " << agent << std::endl;
#endif
                }
            }
        }
    }
}
