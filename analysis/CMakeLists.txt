cmake_minimum_required(VERSION 3.28)

project(fft_calculator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

# Define source files 
set(SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/fft_calculator.cpp
)

set(EXECUTABLE ${CMAKE_PROJECT_NAME})

pybind11_add_module(${EXECUTABLE} ${SRCS})

# Link FFTW
find_library(FFTW_LIB fftw3 REQUIRED)
target_link_libraries(${EXECUTABLE} PRIVATE ${FFTW_LIB})

install(TARGETS ${EXECUTABLE} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})