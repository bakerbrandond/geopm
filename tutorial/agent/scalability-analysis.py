from __future__ import division
import matplotlib
#Agg is needed for non-GUI operation, such as when running on mcfly
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import glob 
import statistics as stats

#geopm_file = glob.glob('prof_freq_1000000000.0_0.trace-mcfly13')
#geopm_file = glob.glob('prof_freq_1000000000.0_0.trace-mcfly13')
#
freqs = list(range(10,29))
xticks = list(range(19))
ranges = [0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]
ranges_yaxis = ['0.0-0.1','0.1-0.2','0.2-0.3','0.3-0.4','0.4-0.5','0.5-0.6','0.6-0.7','0.7-0.8','0.8-0.9','0.9-1.0']
yticks = list(range(10))

df_scal_avg_cores_avg_trials_list = []
df_scal_single_core_avg_trials_list = []
df_scal_single_core_single_trial_list = []
df_power_pkg_avg_trials_list = []
df_scal_avg_count_avg_trials_list = []
df_scal_avg_cores_avg_trials_region_list = []

############
####TEST####
############
#df = pd.read_csv(geopm_file[0], delimiter='|', skiprows=5)
#print(df)

if(len(glob.glob('prof_freq*')) != 0): #Frequency Sweep analysis
    ###########################################
    # ALL CORE SCAL, MULTI TRIAL, SINGLE NODE #
    ###########################################
    times_avg = []
    energies_avg = []
    actual_scal = []

    per_freq_low_pperf_times = []
    per_freq_low_pperf_avg = []
    low_pperf_times_scal = []

    per_freq_high_pperf_times = []
    per_freq_high_pperf_avg = []
    high_pperf_times_scal = []

    for idx_f,f in enumerate(freqs):
        low_pperf_times = []
        high_pperf_times = []
        low_pperf_avg = []
        high_pperf_avg = []
        report_list = glob.glob('prof_freq_' + str(f*100000000) + '*report')
        times = []
        energies = []
        powers = []
        for r in report_list:
            with open(r, 'r') as fp:
                line = fp.readline()
                while line:
                    if('Application Totals' in line):
                        line = fp.readline() #runtime
                        times.append(float(line.replace('runtime (sec): ', '').replace('\n', '').strip()))
                        line = fp.readline() #PKG Energy
                        energies.append(float(line.replace('package-energy (joules): ', '').replace('\n', '').strip()))
                        line = fp.readline() #DRAM Energy
                        line = fp.readline() #Power
                        powers.append(float(line.replace('power (watts): ', '').replace('\n', '').strip()))
                        #print(times)
                    else:
                        line = fp.readline()
        if(len(times) > 0):
            times_avg.append(stats.mean(times))
            energies_avg.append(stats.mean(energies))
            print("Frequency{} Avg Time: {} +/- {}".format(f, stats.mean(times), stats.stdev(times)))
            print("Frequency{} Avg Pkg Power: {} +/- {}".format(f, stats.mean(powers), stats.stdev(powers)))
            print("Frequency{} Avg Pkg Energy: {} +/- {}".format(f, stats.mean(energies), stats.stdev(energies)))

        #print('building dataframes for ratio ' + str(f))
        file_list = glob.glob('prof_freq_' + str(f*100000000) + '.*trace-mcfly13')
        #Handle duplicates based on time column, and reset the index from 0 to max #
        df_list = [(pd.read_csv(file, delimiter='|', skiprows=5).drop_duplicates(subset='TIME')).reset_index(drop=True) for file in file_list]
        df_stats_list = []
   
        #DEBUG
        #print(len(df_list[0]))
    
        #Iterate over the trials
        for idx,df in enumerate(df_list):
            #df = df.reset_index(drop=True)
            for core in list(range(44)):
    
                ###############
                # SCALABILITY #
                ###############
                #Generate scalability if we don't hvae it and do have PCNT and ACNT
                if 'scalability-core'+str(core) not in df.columns:
                    #Get PCNT and ACNT Delta
                    df["PCNT_DELTA-core-" + str(core)] = df["MSR::RESEARCH_PPERF:PCNT-core-" + str(core)] - df["MSR::RESEARCH_PPERF:PCNT-core-" + str(core)].shift()
                    df["ACNT_DELTA-core-" + str(core)] = df["MSR::APERF:ACNT-core-" + str(core)] - df["MSR::APERF:ACNT-core-" + str(core)].shift()
                    #Calculate Scalability and add it to the dataframe
                    df['scalability-core-' + str(core)] = (df[ "PCNT_DELTA-core-" + str(core)]/df[ "ACNT_DELTA-core-" + str(core)]).fillna(0)
    
            ###############
            # SCALABILITY #
            ###############
            #generate the average scalability for all cores in this trial
            df['scalability-avg'] = (df[[i for i in df.columns if 'scalability-core' in i]]).mean(axis=1)
    
            #use this to toss out any samples above a certain PPERF.  Good for some fun data representations for actual scal vs pperf scal 
            #df_list[idx] = df[df["scalability-avg"] < 0.7].reset_index(drop=True)
            low_pperf_times.append(len(df[df["scalability-avg"] < 0.7].reset_index(drop=True))*0.005)
            high_pperf_times.append(len(df[df["scalability-avg"] >= 0.7].reset_index(drop=True))*0.005)
            low_pperf_avg.append( (df[df["scalability-avg"] < 0.7].reset_index(drop=True))["scalability-avg"].mean() )
            high_pperf_avg.append( (df[df["scalability-avg"] >= 0.7].reset_index(drop=True))["scalability-avg"].mean() )
             
            df_stats_list.append(df['scalability-avg'].groupby(pd.cut(df['scalability-avg'], ranges, include_lowest=True)).count())
    
    
            ######################
            # PER REGION PARSING # 
            ######################
    
            df_region_sort = df.sort_values(by=['REGION_HASH','TIME'])
            df_region_sort.reset_index(drop=True)
            #df_list[idx] = df.sort_values(by=['REGION_HASH'])
            #df_list[idx].reset_index(drop=True)
    
            #build a dataframe containing all of the trials dataframes
            regions=df_region_sort['REGION_HASH'].unique().tolist()
            
            #df = df_region_sort.loc[df['REGION_HASH'] == regions[0]]
    
            #DEBUG! 
            #print('df regions:')
            #print(regions)
            #print('df_list['+str(idx)+']:')
            #print(df_list[idx])
    
            #print('df_list['+str(idx)+'] avg_scal:')
            #print(df_list[idx]['scalability-avg'])
    
            #print('df_stats_list['+str(idx)+']:')
            #print(df_stats_list[idx])
    
            #if (idx == 0 and f == 28):
            #    df.to_csv(r'_trace_with_scal.csv')
    
        #This is used for per region parsing currently
        #for idx,df in enumerate(df_list):
        #    #df_list[idx] = df.sort_values(by=['REGION_HASH'])
        #    df_list[idx] = df.loc[df['REGION_HASH'] == regions[1]]
        #    df_list[idx].reset_index(drop=True)
    
        df_concat = pd.concat(df_list, axis=1)#.fillna(0)
        df_concat_stats = pd.concat(df_stats_list, axis=1)#.fillna(0)
    
        df.to_csv(r'_trace_with_scal_concat.csv')
   
        per_freq_low_pperf_times.append(stats.mean(low_pperf_times))
        per_freq_high_pperf_times.append(stats.mean(high_pperf_times))

        per_freq_low_pperf_avg.append(stats.mean(low_pperf_avg))
        per_freq_high_pperf_avg.append(stats.mean(high_pperf_avg))

        if(idx_f != 0):
            if(len(times_avg) > 1):
                delta_t = (times_avg[idx_f-1]-times_avg[idx_f])/times_avg[idx_f-1]
                delta_f = (f - freqs[idx_f-1])/(freqs[idx_f-1])
                print("\t%_delta_t: {}, %_delta_f: {}".format(delta_t, delta_f))
                actual_scal.append(delta_t/delta_f)
                print("\tActual Scal: {}".format(actual_scal[-1]))

            if(len(per_freq_low_pperf_times) > 1):
                delta_t = (per_freq_low_pperf_times[idx_f-1]-per_freq_low_pperf_times[idx_f])/per_freq_low_pperf_times[idx_f-1]
                delta_f = (f - freqs[idx_f-1])/(freqs[idx_f-1])
                print("\tPPERF samples < 0.7 %_delta_t: {}, %_delta_f: {}".format(delta_t, delta_f))
                low_pperf_times_scal.append(delta_t/delta_f)
                print("\tPPERF samples < 0.7 Actual Scal: {}".format(low_pperf_times_scal[-1]))
            if(len(per_freq_high_pperf_times) > 1):
                delta_t = (per_freq_high_pperf_times[idx_f-1]-per_freq_high_pperf_times[idx_f])/per_freq_high_pperf_times[idx_f-1]
                delta_f = (f - freqs[idx_f-1])/(freqs[idx_f-1])
                print("\tPPERF samples > 0.7 %_delta_t: {}, %_delta_f: {}".format(delta_t, delta_f))
                high_pperf_times_scal.append(delta_t/delta_f)
                print("\tPPERF samples > 0.7 Actual Scal: {}".format(high_pperf_times_scal[-1]))


        ###############
        # SCALABILITY #
        ###############
        df_scal_avg_cores_avg_trials = (df_concat[[i for i in df_concat.columns if 'scalability-avg' in i]]).mean(axis=1)
        df_scal_avg_cores_avg_trials_list.append(df_scal_avg_cores_avg_trials)
    
        #This is normal count 
        #df_scal_avg_count_avg_trials = df_concat_stats.mean(axis=1)
        
        #Percentages help
        df_scal_avg_count_avg_trials = df_concat_stats.mean(axis=1)
        df_scal_avg_count_avg_trials = df_scal_avg_count_avg_trials.div(df_scal_avg_count_avg_trials.sum(axis=0),axis=0)
        #print(df_scal_avg_count_avg_trials)
        #print(len(df_scal_avg_count_avg_trials))
    
        df_scal_avg_count_avg_trials_list.append(df_scal_avg_count_avg_trials)
    
        #generate the average scalability for a single core in this trial
        df_scal_single_core_avg_trials = (df_concat[[i for i in df_concat.columns if 'scalability-core-8' in i]]).mean(axis=1)
        df_scal_single_core_avg_trials_list.append(df_scal_single_core_avg_trials)
    
        #generate the scalability for a single core in this a single trial
        df_scal_single_core_single_trial_list.append(df_list[0]['scalability-core-8'])
    
        #########
        # POWER #
        #########
        df_power_pkg_avg_trials = (df_concat[[i for i in df_concat.columns if 'POWER_PACKAGE' in i]]).mean(axis=1)
        df_power_pkg_avg_trials_list.append(df_power_pkg_avg_trials)
    
        #DEBUG
        #if (f == 10):
        #    df_scal_avg_cores_avg_trials.to_csv(r'df_scal_avg_cores_avg_trials.csv')

    df_scal_avg_cores_avg_trials_heatmap = pd.concat(df_scal_avg_cores_avg_trials_list, axis=1)#.fillna(0)
    df_scal_avg_cores_avg_trials_heatmap.columns = freqs
   
    pperf_avg_scal_per_freq = df_scal_avg_cores_avg_trials_heatmap.mean(axis=0)

    df_scal_avg_count_avg_trials_heatmap = pd.concat(df_scal_avg_count_avg_trials_list, axis=1)
    df_scal_avg_count_avg_trials_heatmap.columns = freqs
    
    df_scal_single_core_avg_trials_heatmap = pd.concat(df_scal_single_core_avg_trials_list, axis=1)#.fillna(0)
    df_scal_single_core_avg_trials_heatmap.columns = freqs
    
    df_scal_single_core_single_trial_heatmap = pd.concat(df_scal_single_core_single_trial_list, axis=1)#.fillna(0)
    df_scal_single_core_single_trial_heatmap.columns = freqs
    
    df_power_pkg_avg_trials_heatmap = pd.concat(df_power_pkg_avg_trials_list, axis=1)#.fillna(0)
    df_power_pkg_avg_trials_heatmap.columns = freqs
    
    
    
    #DEBUG
    df_scal_avg_cores_avg_trials_heatmap.to_csv(r'df_scal_avg_cores_avg_trials_heatmap.csv')
    df_scal_single_core_avg_trials_heatmap.to_csv(r'df_scal_single_core_avg_trials_heatmap.csv')
    df_scal_single_core_single_trial_heatmap.to_csv(r'df_scal_single_core_single_trial_heatmap.csv')
    df_power_pkg_avg_trials_heatmap.to_csv(r'df_power_pkg_avg_trials_heatmap.csv')
    
    
    #print('df_scal_single_core_avg_trials_list:')
    #print(df_scal_single_core_avg_trials_list)
    #print(len(df_scal_single_core_avg_trials_list))
    #print('df_list[0]:')
    #print(df_list[0])
    #print('df_scal_avg_cores_avg_trials:')
    #print(df_scal_avg_cores_avg_trials)
    #print('df_heatmap:')
    #print(df_scal_avg_cores_avg_trials_heatmap)
                  
            #if(f == 28): for debug
                #print("df is:")
                #print(df)
            #print('df list: ')
            #print(df_list)
    
    #print('file list: ')
    #print(file_list)
    
    
    ############################################################
    ##
    ## CHARTING STARTS HERE
    ##
    ############################################################
    
    ###########################################################################
    # SCALABILITY HEATMAP - ALL CORE AVERAGE SCAL vs SINGLE CORE AVERAGE SCAL #
    ###########################################################################
    fig1 = plt.figure(figsize=(20,10))
    fig1.suptitle("All core avg PPERF (multi trial) vs Single core avg PPERF (multi trial)")
    
    ax1 = fig1.add_subplot(121)
    im = ax1.pcolormesh(df_scal_avg_cores_avg_trials_heatmap, cmap='gnuplot')
    ax1.set_title("All Core Avg PPERF vs Freq & Time (multi trial)")
    fig1.colorbar(im, ax=ax1).set_label("PPERF")
    ax1.set_xlabel("Freq")
    ax1.set_ylabel("Timestep (5mS)")
    ax1.set_xticklabels(freqs)
    ax1.set_xticks(xticks)
    ax1.plot()
    
    ax2 = fig1.add_subplot(122)
    im = ax2.pcolormesh(df_scal_single_core_avg_trials_heatmap, cmap='gnuplot')
    ax2.set_title("Single Core Avg PPERF vs Freq & Time (multi trial)")
    fig1.colorbar(im, ax=ax2).set_label("PPERF")
    ax2.set_xlabel("Freq")
    ax2.set_ylabel("Timestep (5mS)")
    ax2.set_xticklabels(freqs)
    ax2.set_xticks(xticks)
    ax2.plot()
    
    fig1.savefig('Figure1.png', bbox_inches = 'tight')
    
    ######################################################################
    # SCALABILITY HEATMAP - SINGLE CORE AVERAGE SCAL vs SINGLE CORE SCAL #
    ######################################################################
    fig2 = plt.figure(figsize=(20,10))
    fig2.suptitle("Single core avg PPERF (multi trial) vs Single core PPERF (single trial)")
    
    ax2 = fig2.add_subplot(121)
    im = ax2.pcolormesh(df_scal_single_core_avg_trials_heatmap, cmap='gnuplot')
    ax2.set_title("Single Core Avg PPERF vs Freq & Time (multi trial)")
    fig2.colorbar(im, ax=ax2).set_label("PPERF")
    ax2.set_xlabel("Freq")
    ax2.set_ylabel("Timestep (5mS)")
    ax2.set_xticklabels(freqs)
    ax2.set_xticks(xticks)
    ax2.plot()
    
    ax4 = fig2.add_subplot(122)
    im = ax4.pcolormesh(df_scal_single_core_single_trial_heatmap, cmap='gnuplot')
    ax4.set_title("Single Core PPERF vs Freq & Time (single trial)")
    fig2.colorbar(im, ax=ax4).set_label("PPERF")
    ax4.set_xlabel("Freq")
    ax4.set_ylabel("Timestep (5mS)")
    ax4.set_xticklabels(freqs)
    ax4.set_xticks(xticks)
    ax4.plot()
    
    fig2.savefig('Figure2.png', bbox_inches = 'tight')
    
    ######################################################
    # POWER HEATMAP - ALL CORE AVG SCAL vs PKG POWER AVG #
    ######################################################
    fig3 = plt.figure(figsize=(20,10))
    fig3.suptitle("All core avg PPERF (multi trial) vs Package Power AVG (multi trial)")
    
    ax5 = fig3.add_subplot(121)
    im = ax5.pcolormesh(df_scal_avg_cores_avg_trials_heatmap, cmap='gnuplot')
    ax5.set_title("All Core Avg PPERF vs Freq & Time (multi trial)")
    fig3.colorbar(im, ax=ax5).set_label("PPERF")
    ax5.set_xlabel("Freq")
    ax5.set_ylabel("Timestep (5mS)")
    ax5.set_xticklabels(freqs)
    ax5.set_xticks(xticks)
    ax5.plot()
    
    ax6 = fig3.add_subplot(122)
    im = ax6.pcolormesh(df_power_pkg_avg_trials_heatmap, cmap='gnuplot')
    ax6.set_title("Power Package Average (multi trial)")
    fig3.colorbar(im, ax=ax6).set_label("Power")
    ax6.set_xlabel("Freq")
    ax6.set_ylabel("Timestep (5mS)")
    ax6.set_xticklabels(freqs)
    ax6.set_xticks(xticks)
    ax6.plot()
    
    fig3.savefig('Figure3.png', bbox_inches = 'tight')
    
    #########################
    # PPERF RESIDENCY STATS #
    #########################
    
    fig4 = plt.figure(figsize=(20,10))
    fig4.suptitle("PPERF Residency Stats")
    
    ax6 = fig4.add_subplot(121)
    im = ax6.pcolormesh(df_scal_avg_count_avg_trials_heatmap, cmap='gnuplot')
    ax6.set_title("Average Scalability Range Residency (multi-trial)")
    fig4.colorbar(im, ax=ax6).set_label("% time in range")
    ax6.set_xlabel("Freq")
    ax6.set_ylabel("PPERF range")
    ax6.set_xticklabels(freqs)
    ax6.set_xticks(xticks)
    ax6.set_yticklabels(ranges_yaxis)
    ax6.set_yticks(yticks)
    ax6.plot()
    
    #TODO: add average power over entire workload
    #TODO: add average scalability over entire workload
    
    ax7 = fig4.add_subplot(122)
    ax7.set_title("Average Scalability Range Residency (multi-trial)")
    
    #for i in df_scal_avg_count_avg_trials_list:
    #    print(list(i))
    #    ax7.plot(list(i))
    
    ax7.plot(list(df_scal_avg_count_avg_trials_list[0]), marker ='x', label='freq ' + str(freqs[0]))
    ax7.plot(list(df_scal_avg_count_avg_trials_list[6]), marker ='o', label='freq ' + str(freqs[6]))
    ax7.plot( list(df_scal_avg_count_avg_trials_list[12]), marker ='', label='freq ' + str(freqs[12]))
    ax7.plot(list(df_scal_avg_count_avg_trials_list[18]), marker ='', linestyle='dashed', label='freq ' + str(freqs[18]))
    ax7.legend()
    
    ax7.set_xlabel("PPERF Range")
    ax7.set_ylabel("Percent of time in PPERF range")
    ax7.set_xticklabels(ranges_yaxis)
    ax7.set_xticks(yticks)
    #ax7.set_yticks(ticks)
    fig4.savefig('Figure4.png', bbox_inches = 'tight')



    fig8 = plt.figure(figsize=(20,10))
    fig8.suptitle("PPERF vs Actual Scalability (ROI)")


    ax13 = fig8.add_subplot(121)
    ax13.set_xticklabels(freqs)
    ax13.set_xlabel("Freq")
    ax13.set_xticks(xticks)

    color = 'tab:blue'
    ax13.set_title("Run Aggregate Core PPERF vs Actual Scalability")
    
    ax13.plot(list(pperf_avg_scal_per_freq), label='Socket PPERF', marker='o')

    color = 'tab:red'
    ax13.plot(actual_scal, color=color, label='% delta t/% delta f')
    ax13.legend()

    ax14 = fig8.add_subplot(122)

    ax14.set_title("Average Runtime vs Time when PPERF above/below 0.7")
    ax14.plot(times_avg, label='Avg Runtime', marker='x')
    ax14.plot(list(per_freq_low_pperf_times), label='Time when PPERF < 0.7', marker='o')
    ax14.plot(list(per_freq_high_pperf_times), label='Time when PPERF > 0.7', marker='.')
    ax14.set_xticklabels(freqs)
    ax14.set_xlabel("Freq")
    ax14.set_xticks(xticks)
    ax14.set_ylabel("Time (S)")
    ax14.set_ylim(bottom=0, top=max(times_avg))
    ax14.legend()

    fig8.savefig('Figure8.png', bbox_inches = 'tight')

    fig9 = plt.figure(figsize=(20,10))
    fig9.suptitle("Low/High Scalability Time")

    ax15 = fig9.add_subplot(121)
    ax15.set_xticklabels(freqs)
    ax15.set_xlabel("Freq")
    ax15.set_xticks(xticks)
    ax15.set_ylabel("Time (S)")
    ax15.set_ylim(bottom=0, top=max(per_freq_low_pperf_times))

    ax15.set_title("Time spent in samples with PPERF < 0.7")
    
    ax15.plot(list(per_freq_low_pperf_times), label='Time of Socket PPERF < 0.7', marker='o')

    ax16 = fig9.add_subplot(122)
    ax16.set_xticklabels(freqs)
    ax16.set_xlabel("Freq")
    ax16.set_xticks(xticks)
    ax16.set_ylabel("Time (S)")
    ax16.set_ylim(bottom=0, top=max(per_freq_high_pperf_times))

    ax16.set_title("Time spent in samples with PPERF > 0.7")

    ax16.plot(list(per_freq_high_pperf_times), label='Time of Socket PPERF > 0.7', marker='o')

    fig9.savefig('Figure9.png', bbox_inches = 'tight')

    fig10 = plt.figure(figsize=(20,10))
    fig10.suptitle("Low/High Scalability Sample Analysis")
    ax17 = fig10.add_subplot(121)
    ax17.set_xticklabels(freqs)
    ax17.set_xlabel("Freq")
    ax17.set_xticks(xticks)
    ax17.set_ylabel("Scalability")

    ax17.set_title("Scalability of samples with PPERF < 0.7")
    
    #per_freq_low_pperf_times.append(stats.mean(low_pperf_times))
    ax17.plot(list(low_pperf_times_scal), label='% delta t/% delta f for PPERF < 0.7', marker='x')
    ax17.plot(list(per_freq_low_pperf_avg), label='Avg of PPERF < 0.7', marker='x')
    ax17.legend()

    ax18 = fig10.add_subplot(122)
    ax18.set_xticklabels(freqs)
    ax18.set_xlabel("Freq")
    ax18.set_xticks(xticks)
    ax18.set_ylabel("Scalability")

    ax18.set_title("Scalability of samples with PPERF > 0.7")

    #per_freq_low_pperf_times.append(stats.mean(low_pperf_times))
    ax18.plot(list(high_pperf_times_scal), label='% delta t/% delta f for PPERF > 0.7', marker='x')
    ax18.plot(list(per_freq_high_pperf_avg), label='Avg of PPERF > 0.7', marker='x')
    ax18.legend()
    fig10.savefig('Figure10.png', bbox_inches = 'tight')

else: #PCNT Agent Sweeps
    file_list = glob.glob('pcnt_agent_trace*')

    if(len(file_list) > 0):
        #Handle duplicates based on time column, and reset the index from 0 to max #
        df_list = [(pd.read_csv(file, delimiter='|', skiprows=5).drop_duplicates(subset='TIME')).reset_index(drop=True) for file in file_list]
        df_stats_list = []
        
        #DEBUG
        #print(df_list[0])
        
        #Iterate over the trials
        for idx,df in enumerate(df_list):
            #df = df.reset_index(drop=True)
            #for core in list(range(44)):
            ###############
            # SCALABILITY #
            ###############
            #generate the average scalability for all cores in this trial
            df['scalability-avg'] = (df[[i for i in df.columns if 'SCALABILITY_CORE' in i]]).mean(axis=1)
            df['frequency-req-avg'] = (df[[i for i in df.columns if 'FREQUENCY-core' in i]]).mean(axis=1)
        
            df_stats_list.append(df['scalability-avg'].groupby(pd.cut(df['scalability-avg'], ranges, include_lowest=True)).count())

        df_concat = pd.concat(df_list, axis=1)#.fillna(0)
        df_concat_stats = pd.concat(df_stats_list, axis=1)#.fillna(0)
        df.to_csv(r'_trace_with_scal_concat.csv')

        ###############
        # SCALABILITY #
        ###############
        df_scal_avg_cores_avg_trials = (df_concat[[i for i in df_concat.columns if 'scalability-avg' in i]]).mean(axis=1)
        #print(df_scal_avg_cores_avg_trials)

        df_freq_avg_cores_avg_trials = (df_concat[[i for i in df_concat.columns if 'frequency-req-avg' in i]]).mean(axis=1)
        #print(df_freq_avg_cores_avg_trials)
        
        #This is normal count 
        #df_scal_avg_count_avg_trials = df_concat_stats.mean(axis=1)
        #Percentages help
        df_scal_avg_count_avg_trials = df_concat_stats.mean(axis=1)
        df_scal_avg_count_avg_trials = df_scal_avg_count_avg_trials.div(df_scal_avg_count_avg_trials.sum(axis=0),axis=0)
        #print(df_scal_avg_count_avg_trials)
        #print(len(df_scal_avg_count_avg_trials))
        
        #########
        # POWER #
        #########
        df_power_pkg_avg_trials = (df_concat[[i for i in df_concat.columns if 'POWER_PACKAGE' in i]]).mean(axis=1)

        #########################
        # PPERF RESIDENCY STATS #
        #########################
        
        fig5 = plt.figure(figsize=(20,10))
        fig5.suptitle("PPERF Stats")
        
        ax8 = fig5.add_subplot(121)
        ax8.set_title("Average Core PPERF Over Time")
        
        ax8.plot(list(df_scal_avg_cores_avg_trials), marker='', label='All Core Average')
        ax8.legend()
        ax8.set_xlabel("Timestep (5mS)")
        ax8.set_ylabel("PPERF Scalability")
        
        ax9 = fig5.add_subplot(122)
        ax9.set_title("Average Scalability Range Residency (multi-trial)")
        
        ax9.plot(list(df_scal_avg_count_avg_trials), label='All Core Average')
        ax9.legend()
        ax9.set_xlabel("PPERF Range")
        ax9.set_ylabel("Percent of time in PPERF range")
        ax9.set_xticklabels(ranges_yaxis)
        ax9.set_xticks(yticks)

        fig5.savefig('Figure5.png', bbox_inches = 'tight')
        

        fig6 = plt.figure(figsize=(20,10))
        fig6.suptitle("?")

        ax10 = fig6.add_subplot(121)
        ax10.set_title("Average Core Freq over time")
        ax10.plot(list(df_freq_avg_cores_avg_trials), marker='', label='Average Freq (all cores)')
        ax10.legend()
        ax10.set_xlabel("Timestep (5mS)")
        ax10.set_ylabel("Frequency")

        ax11 = fig6.add_subplot(122)
        ax11.set_title("Power Package Average (multi trial)")
        ax11.plot(list(df_power_pkg_avg_trials), marker='', label='Package Power')
        ax11.set_xlabel("Timestep (5mS)")
        ax11.set_ylabel("Power (W)")
        ax11.plot()
        
        fig6.savefig('Figure6.png', bbox_inches = 'tight')
   


        fig7 = plt.figure(figsize=(20,10))
        fig7.suptitle("PPERF Stats")
        
        ax12_a = fig7.add_subplot(111)
        color = 'tab:blue'
        ax12_a.set_title("Average Core PPERF Over Time")
        
        ax12_a.plot(list(df_scal_avg_cores_avg_trials), marker='.', linewidth=0.1, label='All Core Average')
        ax12_a.set_xlabel("Timestep (5mS)")
        ax12_a.set_ylabel("PPERF Scalability")
        ax12_a.hlines(0.7, 0, df_scal_avg_cores_avg_trials.last_valid_index(), linestyles='dashed')
        ax12_a.hlines(0.5, 0, df_scal_avg_cores_avg_trials.last_valid_index(), linestyles='dashed')

        ax12_b = ax12_a.twinx()  # instantiate a second axes that shares the same x-axis
        color = 'tab:red'
        ax12_b.set_ylabel('Frequency', color=color)  # we already handled the x-label with ax12_a
        ax12_b.plot(list(df_freq_avg_cores_avg_trials), color=color, label='Frequency')
        ax12_b.tick_params(axis='y', labelcolor=color)

        fig7.savefig('Figure7.png', bbox_inches = 'tight')


    file_list = glob.glob('*report*')
    times = []
    powers = []
    energies = []
    for f in file_list:
        with open(f, 'r') as fp:
            line = fp.readline()
            while line:
                if('Application Totals' in line):
                    line = fp.readline()
                    times.append(float(line.replace('runtime (sec): ', '').replace('\n', '').strip()))
                    line = fp.readline() #PKG Energy
                    energies.append(float(line.replace('package-energy (joules): ', '').replace('\n', '').strip()))
                    line = fp.readline() #DRAM Energy
                    line = fp.readline() #Power
                    powers.append(float(line.replace('power (watts): ', '').replace('\n', '').strip()))
                    #print(times)
                    #print(times)
                else:
                    line = fp.readline()
    if(len(times) > 0):
        print("PCNT Agent Avg Time: {} +/- {}".format(stats.mean(times), stats.stdev(times)))
        print("PCNT Agent Avg Pkg Power: {} +/- {}".format(stats.mean(powers), stats.stdev(powers)))
        print("PCNT Agent Avg Pkg Energy: {} +/- {}".format(f, stats.mean(energies), stats.stdev(energies)))


#plt.show()
