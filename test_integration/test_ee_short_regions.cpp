/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
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

#include <string.h>
#include <limits.h>
#include <mpi.h>
#include <string>
#include <vector>
#include <memory>

#include "geopm.h"
#include "Exception.hpp"
#include "ModelRegion.hpp"
#include "Profile.hpp"


int main(int argc, char **argv)
{
    int err = 0;
    int comm_rank;
    int comm_size;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &comm_size);
    MPI_Comm_rank(MPI_COMM_WORLD, &comm_rank);
    bool is_verbose = false;
    if (comm_rank == 0) {
        for (int arg_idx = 1; arg_idx < argc; ++arg_idx) {
            if (strncmp(argv[arg_idx], "--verbose", strlen("--verbose")) == 0 ||
                strncmp(argv[arg_idx], "-v", strlen("-v")) == 0) {
                is_verbose = true;
            }
        }
    }

    std::vector<double> big_o {0.001, 0.002, 0.004, 0.008, 0.016, 0.032, 0.064, 0.128};
    std::vector<int> repeat {12800, 6400, 3200, 1600, 800, 400, 200, 100};
    size_t num_trial = big_o.size();
    geopm::Profile &prof = geopm::Profile::default_profile();

    for (size_t trial_idx = 0; trial_idx != num_trial; ++trial_idx) {
        std::string scaling_name = "scaling_" + std::to_string(trial_idx);
        std::string spin_name = "network_spin_" + std::to_string(trial_idx);
        std::unique_ptr<geopm::ModelRegion> scaling_model(
            geopm::ModelRegion::model_region("scaling", big_o[trial_idx], is_verbose));
        std::unique_ptr<geopm::ModelRegion> spin_model(
            geopm::ModelRegion::model_region("spin", big_o[trial_idx], is_verbose));
        uint64_t scaling_rid = prof.region(scaling_name, GEOPM_REGION_HINT_UNKNOWN);
        uint64_t spin_rid = prof.region(spin_name, GEOPM_REGION_HINT_NETWORK);
        for (int rep_idx = 0; rep_idx != repeat[trial_idx]; ++rep_idx) {
            prof.enter(scaling_rid);
            scaling_model->run();
            prof.exit(scaling_rid);
            prof.enter(spin_rid);
            spin_model->run();
            prof.exit(spin_rid);
        }
        MPI_Barrier(MPI_COMM_WORLD);
    }
    MPI_Finalize();
    return err;
}
