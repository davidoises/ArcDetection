cmake_minimum_required(VERSION 3.28)

project(model_training)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define source files 
set(SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/batch_feature_extraction.cpp
)

set(EXECUTABLE ${CMAKE_PROJECT_NAME})
add_executable(${EXECUTABLE}
    ${SRCS}
)

# Link FFTW
find_library(FFTW_LIB fftw3 REQUIRED)
target_link_libraries(${EXECUTABLE} ${FFTW_LIB})