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

#ifndef PLATFORMIO_H_INCLUDE
#define PLATFORMIO_H_INCLUDE

int geopm_platform_io_signal_names(int *num_signals, int **signal_lens, char ***signal_names);
int geopm_platform_io_control_names(int *num_controls, int **control_lens, char ***control_names);
int geopm_platform_io_signal_domain_type(int signal_str_len, const char *signal_name, int *domain_type);
int geopm_platform_io_control_domain_type(int control_str_len, const char *control_name, int *domain_type);
int geopm_platform_io_push_signal(int signal_str_len, const char *signal_name, int domain_type, int domain_idx, int *signal_idx);
int geopm_platform_io_push_combined_signal(int signal_str_len, const char *signal_name, int domain_type, int domain_idx,
                                           int num_sub_signal, const int *sub_signal_idx, int *combined_signal_idx);
int geopm_platform_io_push_control(int control_str_len, const char *control_name, int domain_type, int domain_idx, int *control_idx);
int geopm_platform_io_num_pushed_signal(int *num_signals);
int geopm_platform_io_num_pushed_control(int *num_controls);
int geopm_platform_io_sample(int signal_idx, double *value);
int geopm_platform_io_adjust(int control_idx, double setting);
int geopm_platform_io_read_batch(void);
int geopm_platform_io_write_batch(void);
int geopm_platform_io_read_signal(int signal_str_len, const char *signal_name, int domain_type, int domain_idx, double *value);
int geopm_platform_io_write_control(int control_str_len, const char *control_name, int domain_type, int domain_idx, double setting);
int geopm_platform_io_save_control(void);
int geopm_platform_io_restore_control(void);

#endif
