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

#include <stdio.h>
#include <iostream>
#include <map>
#include <string.h>
#include <stdlib.h>

#include "geopm_message.h"
#include "mpi.h"

struct win_ctx_s {
    size_t size;
    void *base;
};

std::map<uint64_t, struct win_ctx_s> g_win_ctx_map;

extern "C"
{
    // all MPI_ should be able to return !MPI_SUCCESS based on flag for unit testing
    // uninteresting for unit testing
    int MPI_Init(int *argc, char **argv[])
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // uninteresting for unit testing
    int MPI_Finalize(void)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // need to cover 2 scenarios for unit testing
    // 0 and 1 (root/non-root)
    int MPI_Comm_rank(MPI_Comm comm, int *rank)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        *rank = 0;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // fixture specific
    int MPI_Comm_size(MPI_Comm comm, int *size)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        *size = 2;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // uninteresting for unit testing?
    int MPI_Cart_create(MPI_Comm comm_old, int ndims, const int dims[], const int periods[], int reorder, MPI_Comm *comm_cart)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        *comm_cart = comm_old + 64;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // need to cover 2 scenarios for unit testing
    // 0 and 1 (root/non-root)
    int MPI_Cart_rank(MPI_Comm comm, const int coords[], int *rank)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        *rank = 0;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // uninteresting for unit testing
    int MPI_Cart_coords(MPI_Comm comm, int rank, int maxdims, int coords[])
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // uninteresting for unit testing
    int MPI_Comm_split(MPI_Comm comm, int color, int key, MPI_Comm *newcomm)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        *newcomm = comm + color + key;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    int MPI_Comm_free(MPI_Comm *comm)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // uninteresting for unit testing
    int PMPI_Barrier(MPI_Comm comm)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // uninteresting for unit testing
    int MPI_Comm_dup(MPI_Comm comm, MPI_Comm *newcomm)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        *newcomm = comm;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // uninteresting for unit testing
    int MPI_Win_unlock(int rank, MPI_Win win)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // fixture specific action on buffer refernced by win handle
    int MPI_Win_lock(int lock_type, int rank, int assert, MPI_Win win)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        static struct geopm_policy_message_s policy = {0};
        static struct geopm_sample_message_s sample = {0};
        int scenario = -1;
        char *env_var = getenv(__func__);
        if (env_var) {
            scenario = std::atoi(env_var);
        }

        switch (scenario) {
            case 0:  //send_policy_down
                memcpy(&win, &policy, sizeof(policy));
                break;
            case 1:  //send_sample_up
                sample.region_id = 1;
                struct win_ctx_s ctx = g_win_ctx_map[(uint64_t)win];
                void *base = ctx.base;
                size_t size = ctx.size;
                for (size_t x = 0; x < size; x++) {
                    memcpy((base + x), &sample, sizeof(sample));
                    //std::fill((geopm_sample_message_s *)&win, (geopm_sample_message_s *)(&win + size), &sample);
                }
                break;
            default:
                break;
        }

        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    int MPI_Win_free(MPI_Win *win)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // associate base with win and record size
    int MPI_Win_create(void *base, MPI_Aint size, int disp_unit, MPI_Info info, MPI_Comm comm, MPI_Win *win)
    {
        static uint64_t win_count = 1;
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        struct win_ctx_s ctx;
        ctx.base = base;
        ctx.size = size;
        *win = (MPI_Win)win_count;
        g_win_ctx_map[win_count] = ctx;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        win_count++;
        return MPI_SUCCESS;
    }

    int MPI_Alloc_mem(MPI_Aint size, MPI_Info info, void *baseptr)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        *((void **) baseptr) = (void *) malloc(size);
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    int MPI_Free_mem(void *base)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        free(base);
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // uninteresting for unit testing
    int MPI_Error_string(int errorcode, char *string, int *resultlen)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }

    // fixture specific action on buffer referenced by win
    int MPI_Put(const void *origin_addr, int origin_count, MPI_Datatype origin_datatype,
            int target_rank, MPI_Aint target_disp, int target_count,
            MPI_Datatype target_datatype, MPI_Win win) MPICH_ATTR_POINTER_WITH_TYPE_TAG(1,3)
    {
        std::cerr << "<mock_entry>" << __func__ << std::endl;
        std::cerr << "<mock_exit>" << __func__ << std::endl;
        return MPI_SUCCESS;
    }
}

