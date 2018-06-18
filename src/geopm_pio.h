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

#ifndef GEOPM_PIO_H_INCLUDE
#define GEOPM_PIO_H_INCLUDE

int geopm_pio_num_signal(int *num_signal);

int geopm_pio_num_control(int *num_control);

int geopm_pio_signal_name(int signal_idx,
                          size_t *str_len,
                          char *signal_name);

int geopm_pio_control_name(int control_idx,
                           size_t *str_len,
                           char *control_name);

int geopm_pio_signal_domain_type(const char *signal_name,
                                 int *domain_type);

int geopm_pio_control_domain_type(const char *control_name,
                                  int *domain_type);

int geopm_pio_push_signal(const char *signal_name,
                          int domain_type,
                          int domain_idx,
                          int *signal_stack_idx);

int geopm_pio_push_combined_signal(const char *signal_name,
                                   int domain_type,
                                   int domain_idx,
                                   int num_sub_signal,
                                   const int *sub_signal_idx,
                                   int *signal_stack_idx);

int geopm_pio_push_control(const char *control_name,
                           int domain_type,
                           int domain_idx,
                           int *control_idx);

int geopm_pio_num_pushed_signal(int *num_signal);

int geopm_pio_num_pushed_control(int *num_controls);

int geopm_pio_sample(int signal_idx,
                     double *value);

int geopm_pio_adjust(int control_idx,
                     double setting);

int geopm_pio_read_batch(void);

int geopm_pio_write_batch(void);

int geopm_pio_read_signal(const char *signal_name,
                          int domain_type,
                          int domain_idx,
                          double *value);

int geopm_pio_write_control(const char *control_name,
                            int domain_type,
                            int domain_idx,
                            double setting);

int geopm_pio_save_control(void);

int geopm_pio_restore_control(void);

#endif
