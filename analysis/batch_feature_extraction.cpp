// FFTW library
#include <fftw3.h>

#include <stdio.h> // printf
#include <string> // std::string
#include <filesystem> // std::filesystem::path, std::filesystem::exists, std::filesystem::remove, std::filesystem::directory_iterator, std::filesystem::directory_entry
#include <vector> // std::vector
#include <array> // std::array
#include <fstream> // std::ifstream
#include <sstream> // std::stringstream, std::ostringstream

// Some parameters that define the model behavior
static constexpr std::size_t BUFFER_SIZE = 512u;
static constexpr std::size_t WINDOW_SIZE = 30u;
static constexpr float BINS_LOWER_LIMIT = 10000;
static constexpr float BINS_UPPER_LIMIT = 50000;
static constexpr float SAMPLING_RATE = 250000;
static constexpr float BIN_WIDTH = SAMPLING_RATE/static_cast<float>(BUFFER_SIZE);

std::vector<std::vector<float>> load_file(const std::string& file_name)
{
    // Open the file
    std::ifstream file(file_name);

    // Check if opened correctly
    if (!file.is_open())
    {
        printf("Skipping file. Could not open: %s\r\n", file_name.c_str());
        return {};
    }

    // Resulting container with file's contents
    std::vector<std::vector<float>> file_rows;

    std::string line;
    while (std::getline(file, line))
    {
        // Create a row from a csv line
        std::vector<float> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ','))
        {
            // Safe conversion to float
            try
            {
                const float value = std::stof(cell);
                row.push_back(value);
            }
            catch(const std::invalid_argument& e)
            {
                printf("Error parsing file in '%s' for cell %s at line %li", e.what(), cell.c_str(), file_rows.size());
                return {};
            }
        }

        // Store the row
        file_rows.push_back(row);

    }

    return file_rows;
}

void show_file(const std::vector<std::vector<float>>& file_rows, const size_t number_of_lines = 5u)
{
    // Display file structure
    const size_t number_of_rows = file_rows.size();
    const size_t number_of_columns = (number_of_rows > 0u) ? file_rows[0u].size() : 0u;
    printf("File contains %li rows and %li columns\r\n", number_of_rows, number_of_columns);

    // Print line by line
    size_t line_count = 0u;
    for (const std::vector<float>& row : file_rows)
    {

        // Stop after reaching the specified number of lines
        if (line_count == number_of_lines)
        {
            break;
        }

        // Formate the line into a human readable string
        std::ostringstream oss;
        for (const float value : row) {
            oss << std::fixed << std::setprecision(6) << std::setw(10) << value << " | ";
        }

        line_count++;

        printf("Line %li: %s\r\n", line_count, oss.str().c_str());
    }
}

void process_file(const std::vector<std::vector<float>>& file_rows)
{
    // Used by FFTW to calculate fft
    std::array<double, BUFFER_SIZE> fft_buffer_;
    std::array<fftw_complex, BUFFER_SIZE> fft_res_;
    fftw_plan fft_algo_plan_ = fftw_plan_dft_r2c_1d(BUFFER_SIZE, fft_buffer_.data(), fft_res_.data(), FFTW_ESTIMATE);
    fftw_destroy_plan(fft_algo_plan_);
}

void process_files(const std::string& data_folder, const std::vector<std::string>& files_list, const std::string& output_file)
{
    bool showed_first = false;
    for (const std::string& file_name : files_list)
    {
        // This will be the equivalent of lading the dataframe from the file
        std::vector<std::vector<float>> file_rows = load_file(data_folder + file_name);

        if (!showed_first)
        {
            show_file(file_rows);
            showed_first = true;
        }

        process_file(file_rows);

        break;
    }
}

int main(int argc, char * argv[])
{
    printf("Starting\r\n");

    // Find path of current directory to process the input files
    const std::string file_path = __FILE__;
    const std::string dir_path = file_path.substr(0, file_path.find_last_of("/\\"));
    const std::string input_path = dir_path + "/../arc_data/raw/";

    // Check ig input path exists
    std::filesystem::path input_path_fs(input_path);
    if (!std::filesystem::exists(input_path_fs))
    {
        printf("Input folder not found: %s\r\n", input_path.c_str());
        return -1;
    }

    // Define the output path
    const std::string output_file = dir_path + "/preprocessed_data/processed_data.csv";
    std::filesystem::path output_file_fs(output_file);
    if (std::filesystem::exists(output_file_fs))
    {
        std::filesystem::remove(output_file_fs);
    }

    // Get a list of the file names in the input folder
    std::vector<std::string> files_list;
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(input_path_fs))
    {
        files_list.push_back(entry.path().filename().string());
    }

    printf("Processing %li files from input folder: %s\r\n", files_list.size(), input_path.c_str());

    process_files(input_path, files_list, output_file);

    printf("Output file: %s\r\n", output_file.c_str());


    return 0;
}