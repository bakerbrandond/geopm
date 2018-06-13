from cffi import FFI

ffibuilder = FFI()
ffibuilder.set_source("_libgeopm_cffi", None)
ffibuilder.cdef("""
    /// from src/geopm_platform_topo.h
    int geopm_platform_topo_num_domain(int domain_type, int *num_domain);
    int geopm_platform_topo_domain_cpus(int domain_type, int domain_idx, int *num_cpu, int **cpu_idx);
    int geopm_platform_topo_domain_idx(int domain_type, int cpu_idx, int *domain_idx);
    int geopm_platform_topo_define_cpu_group(int num_cpu, const int *cpu_domain_idx, int *cpu_group_idx);
    int geopm_platform_topo_is_domain_within(int inner_domain, int outer_domain, bool *is_within);
    ///@todo return str_len?
    int geopm_platform_topo_domain_type_to_name(int domain_type, int str_len, char *domain_name);
    int geopm_platform_topo_domain_name_to_type(int str_len, const char *domain_name, int *domain_type);
    /// from src/geopm_platform_io.h
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
""")

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)
