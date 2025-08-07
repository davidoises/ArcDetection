import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from csv import writer
import sys

##### Some constants and important definitions #####

def _range(series):
    return series.max() - series.min()

aggregate_dict = {'time': ['mean'],
                 'adc_data': ['mean', _range],
                 'adc_diff': ['mean', _range]}


START_INDEX = 5000
ORIGINAL_ARC_EVENT_TIME = 0.15

# To be used with 1024 samples and 500kHz sampling rate
# NUMBER_OF_SAMPLES_PER_TIME_DOMAIN_GROUP = 902
# SAMPLING_RATE = 500e3
# FFT_BUFFER_SIZE = 1024

# To be used with 512 samples and 250kHz sampling rate
NUMBER_OF_SAMPLES_PER_TIME_DOMAIN_GROUP = 388
SAMPLING_RATE = 250e3
FFT_BUFFER_SIZE = 512

LOWER_BOUND = 10000
UPPER_BOUND = 50000

BIN_RESOLUTION = 488

SAMPLES_PRE_AND_POS_ARC = 15

SCALE = 1


##### Helper function to get different resolutions for FFT #####

def reshape_spectrogram(bin_width, orig_freqs, data):

    number_of_bins = int(np.max(orig_freqs)/bin_width)

    _, number_of_samples = data.shape

    new_freqs = np.empty((number_of_bins, 1))
    new_data = np.empty((number_of_bins, number_of_samples))

    freqs_bins_index = 0
    for i in range(number_of_bins):
        frequency_bin = (i+1)*bin_width

        current_bin_indexes = []
        while True:

            if freqs_bins_index >= len(orig_freqs):
                break

            if orig_freqs[freqs_bins_index] > frequency_bin:
                break

            current_bin_indexes.append(freqs_bins_index)
            freqs_bins_index += 1

        start = min(current_bin_indexes)
        end = max(current_bin_indexes)
        slice_of_data = data[start:end+1, :]
        new_data[i,:] = np.mean(slice_of_data, axis=0)

        new_freqs[i] = frequency_bin

    return (new_freqs, new_data)


##### Start processing the files #####

def process_files(data_folder, files_list, output_filename):

    counter = 0
    column_names = []

    # Iterate through each single file in the folder
    for file in files_list:

        ##### START 1. Loading and formatting data #####

        # Load the raw data
        df = pd.read_csv(data_folder + file, header=None)
        df.columns = ['time', 'adc_data', 'currentA', 'currentB', 'currentC', 'currentD']

        df.drop(['currentA', 'currentB', 'currentC', 'currentD'], axis=1, inplace=True)

        # Add the time domain derivative
        df['adc_diff'] = df['adc_data'].diff()

        # Remove the first 5000 samples to remove the noise due to initial conditions in the simulation
        start_time_offset = df.iloc[START_INDEX]['time']
        df = df.iloc[START_INDEX:]

        # Define when the arc event happened. From simulation settings this was set to 0.15 secs.
        # Subtract the time offset due to removing samples
        arc_event_time = ORIGINAL_ARC_EVENT_TIME - start_time_offset

        # Offset the whole time column with the extracted offset due to sample removal.
        # This ensures the reamining data starts with time = 0
        df['time'] = df['time']-start_time_offset
        df.reset_index(inplace = True)

        ##### END 1. Loading and formatting data #####

        ##### START 2. Time domain data extraction #####

        # Create bins of the time domain signal and get some statistics for each group of N samples. This is equivalent to downsampling the data + calculating statistics
        grouped_adc_df = df.groupby(np.arange(len(df.index))//NUMBER_OF_SAMPLES_PER_TIME_DOMAIN_GROUP)
        binned_df = grouped_adc_df.aggregate(aggregate_dict)

        ##### END 2. Time domain data extraction #####

        ##### START 3. Frequency domain data extraction #####

        # Get spectrogram
        Pxx, freqs, time_bins, im = plt.specgram(df['adc_data'], Fs=SAMPLING_RATE, NFFT=FFT_BUFFER_SIZE)

        # Resample spectrogram if needed (Changing resolution of frequency bins)
        new_freqs, new_pxx = reshape_spectrogram(BIN_RESOLUTION, freqs, Pxx)

        # Empty dataframe to store contents of spectrogram in final format
        freq_data_df = pd.DataFrame()

        # Iterate through the different frequency bins and add their data as columns in the new dataframe
        for i in range(len(new_pxx)):

            # Select only the frequencies within the selected range of interest
            if new_freqs[i,0] > LOWER_BOUND and new_freqs[i,0] < UPPER_BOUND:

                # Create a new column for each bin
                bin_name = 'bin_' + str(new_freqs[i-1,0])
                freq_data_df[bin_name] = new_pxx[i,:]

        # Calculate the sum of all the magnitudes of the FFT at different frequencies
        freq_data_df["abs_sum"] = freq_data_df.iloc[:, 1:].abs().sum(axis=1)

        ##### END 3. Frequency domain data extraction #####

        ##### START 4. Splitting time and frequency data into pre-arc section and during-arc section

        # At this point both the frequency and tome domain have similar formats and bin widths, now they can be merged into single dataframe
        final_df = pd.concat([binned_df, freq_data_df], axis=1)
        final_df = final_df.rename(columns={('time','mean'): 'time',
                                        ('adc_data','mean'): 'adc_data_TD_mean',
                                        ('adc_data','_range'): 'adc_data_TD_range',
                                        ('adc_diff','mean'): 'adc_data_TD_deriv_mean',
                                        ('adc_diff','_range'): 'adc_data_TD_deriv_range',
                                        'abs_sum': 'adc_data_FD_abs_sum',})
        
        # Find the time at which the arc happened in terms of dataframe index
        arc_event_index = -1
        for ind in final_df.index:
            if final_df['time'][ind] >= arc_event_time:
                # print(final_df['time'][ind])
                # print(ind)
                arc_event_index = ind
                break

        # Generate 1 window centered around the arc event
        window_df = final_df.iloc[arc_event_index-1-SAMPLES_PRE_AND_POS_ARC:arc_event_index+SAMPLES_PRE_AND_POS_ARC,:]

        # APPLY SCALING HERE
        if SCALE == 1:
            window_scaled_df = (window_df - window_df.min())/(window_df.max() - window_df.min())
            window_scaled_df['time'] = window_df['time']
            window_scaled_df['adc_data_TD_deriv_range'] = window_df['adc_data_TD_deriv_range']
            window_scaled_df['adc_data_TD_range'] = window_df['adc_data_TD_range']

            arc_features = window_scaled_df.aggregate('mean')
        else:
            arc_features = window_df.aggregate('mean')

        features_delta = arc_features

        ##### END 4. Splitting time and frequency data into pre-arc section and during-arc section

        ##### START 5. Writing the processed data in a final file

        if counter == 0:
            # Extract column names and add to header of file
            column_names = features_delta.index.to_list()
            column_names.append("conditions")
            column_names.append("arc")
            with open(output_filename, 'a', newline='') as output_file:
                writer_object = writer(output_file)
                writer_object.writerow(column_names)

        with open(output_filename, 'a', newline='') as output_file:
            conditions = file[:-9]
            arc = int(file[-5:-4])
            feats_list = features_delta[:].to_list()
            feats_list.append(conditions)
            feats_list.append(arc)

            writer_object = writer(output_file)
            writer_object.writerow(feats_list)

        ##### END 5. Wiriting the processed data in a final file

            counter += 1


def start_processing(input, output):


    # data_folder = 'C:/energy_system_modeling_outputs/LongRunMultipleParameters/'
    data_folder = input + '/'

    # Get all the filenames in the data folder
    dir_list = os.listdir(data_folder)
    files_list = [f for f in dir_list if os.path.isfile(data_folder+'/'+f)]

    # OUTPUT_FOLDER = 'C:/energy_system_modeling_outputs/'
    output_folder = output
    OUTPUT_FILENAME = 'processed_data.csv'

    output_filename = output_folder + "/" + OUTPUT_FILENAME
    if os.path.exists(output_filename):
        os.remove(output_filename)

    process_files(data_folder, files_list, output_filename)

    print("Results stored in ", output_filename)

    return output_filename

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("No args were provided")
        exit()

    if len(sys.argv) < 3:
        print("You need to provide an input and output folder")
        exit()

    start_processing(sys.argv[1], sys.argv[2])
