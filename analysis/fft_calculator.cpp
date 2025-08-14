#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

// FFTW library
#include <fftw3.h>

#include <stdio.h> // printf
#include <vector> // std::vector
#include <cmath> // std::log10, std::cos
#include <tuple> // std::tuple, std::make_tuple

pybind11::array_t<double> get_frequency_bins(const float sampling_rate, const size_t buffer_size)
{
    // Structure to store the FFT frequencies
    pybind11::ssize_t freq_rows = buffer_size/2u + 1u;
    pybind11::array_t<double> frequency_bins(
        pybind11::buffer_info(
            nullptr,                                    // no data yet, pybind will allocate
            sizeof(double),                             // size of one scalar
            pybind11::format_descriptor<double>::format(),
            1,                                          // ndim
            {freq_rows},                                // shape
            {sizeof(double)}                            // strides
        )
    );
    auto freq_buf = frequency_bins.mutable_unchecked<1>();

    // Calculate the frequency bins
    for (size_t i = 0u; i < freq_buf.shape(0u); i++)
    {
        const double freq = static_cast<double>(i)*sampling_rate/static_cast<double>(buffer_size);
        freq_buf(i) = freq;
    }

    return frequency_bins;
}

std::tuple<pybind11::array_t<double>, pybind11::array_t<double>> get_spectrogram(const pybind11::array_t<double>& input_array, const float sampling_rate, const size_t buffer_size, const size_t noverlap)
{
    auto frequency_bins = get_frequency_bins(sampling_rate, buffer_size);
    
    // Request buffer information from the input NumPy array
    auto input_buf = input_array.request();
    // Access the data
    double *data_ptr = static_cast<double *>(input_buf.ptr);

    // Prepare a Hann window
    std::vector<double> hann_window(buffer_size);
    for (size_t i = 0u; i < buffer_size; i++)
    {
        hann_window[i] = 0.5 - 0.5 * std::cos(2.0 * M_PI * static_cast<double>(i) / static_cast<double>(buffer_size - 1u));
    }

    // Buffer of samples on which the FFT will be processed
    std::vector<double> fft_buffer(buffer_size);

    // Used by FFTW to calculate fft
    std::vector<fftw_complex> fft_res(buffer_size);
    fftw_plan fft_algo_plan_ = fftw_plan_dft_r2c_1d(buffer_size, fft_buffer.data(), fft_res.data(), FFTW_ESTIMATE);

    // Properties of the spectrogram
    const size_t step = buffer_size - noverlap;
    const size_t num_segments = (input_buf.shape[0u] - buffer_size) / step + 1;
    
    // Structure to store the spectrogram
    pybind11::ssize_t rows = buffer_size/2u + 1u;
    pybind11::ssize_t cols = num_segments;
    pybind11::array_t<double> fft_results(
        pybind11::buffer_info(
            nullptr,                                        // no data yet, pybind will allocate
            sizeof(double),                                 // size of one scalar
            pybind11::format_descriptor<double>::format(),
            2,                                              // ndim
            {rows, cols},                                   // shape
            {sizeof(double)*cols, sizeof(double)}           // strides
        )
    );
    auto res_buf = fft_results.mutable_unchecked<2>();

    // Start processing the data
    for (size_t segment_index = 0u; segment_index < num_segments; segment_index++)
    {

        // Fill the buffer with input data
        // The start index depends on the current segment and the segment step
        // Apply the Hann window to the buffer
        const size_t start_index = segment_index*step;
        for (size_t i = 0u; i < buffer_size; i++)
        {
            // Store in buffer
            fft_buffer[i] = data_ptr[start_index + i] * hann_window[i];
        }

        // Perform FFT
        fftw_execute(fft_algo_plan_);

        for (size_t row = 0u; row < buffer_size/2u; row++)
        {
            const double real = fft_res[row][0];
            const double imag = fft_res[row][1];
            const double magnitude = (real * real + imag * imag)/static_cast<double>(buffer_size * buffer_size);
            // const double magnitude_db = 10.0 * std::log10(magnitude + 1e-12); // Avoid log(0)
            res_buf(row, segment_index) = magnitude;
        }
    }
    
    fftw_destroy_plan(fft_algo_plan_);

    return std::make_tuple(fft_results, frequency_bins);
}

PYBIND11_MODULE(fft_calculator, m)
{
    m.doc() = "C++ spectrogram calculator using FFTW library"; // Optional module docstring

    m.def("get_spectrogram", &get_spectrogram, "A function that returns an spectrogram",
          pybind11::arg("input_array"), pybind11::arg("sampling_rate"), pybind11::arg("buffer_size"), pybind11::arg("noverlap"));
}