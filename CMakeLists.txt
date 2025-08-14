cmake_minimum_required(VERSION 3.8)
project(arc_detection)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

########### Messages ###########

set(MSG_FILES
  "interfaces/msg/FFT.msg"
  "interfaces/msg/FFTBuffer.msg"
  "interfaces/msg/MLFeatures.msg"
  "interfaces/msg/MLResult.msg"
  "interfaces/msg/IndexedSeries.msg"
  "interfaces/msg/TimeSeries.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
)

ament_export_dependencies(rosidl_default_runtime)

########### ADC Node ###########

set(ADC_EXECUTABLE adc_model)

set(ADC_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/adc/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/adc/adc_model.cpp
)

set(ADC_INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/common/
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/adc/
)

add_executable(${ADC_EXECUTABLE}
    ${ADC_SRC_FILES}
)
target_include_directories(${ADC_EXECUTABLE} PRIVATE  
    ${ADC_INC_DIRS}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${ADC_EXECUTABLE} "${cpp_typesupport_target}")

ament_target_dependencies(${ADC_EXECUTABLE} rclcpp std_msgs)

########### DMA Node ###########

set(DMA_EXECUTABLE dma_model)

set(DMA_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/dma/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/dma/dma_model.cpp
)

set(DMA_INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/common/
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/dma/
)

add_executable(${DMA_EXECUTABLE}
    ${DMA_SRC_FILES}
)
target_include_directories(${DMA_EXECUTABLE} PRIVATE  
    ${DMA_INC_DIRS}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${DMA_EXECUTABLE} "${cpp_typesupport_target}")

ament_target_dependencies(${DMA_EXECUTABLE} rclcpp std_msgs)

########### Arc detection Node ###########

set(DETECTION_EXECUTABLE detection_model)

set(DETECTION_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/detection/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/detection/detection_model.cpp
)

set(DETECTION_INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/common/
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_sources/detection/
)

add_executable(${DETECTION_EXECUTABLE}
    ${DETECTION_SRC_FILES}
)
target_include_directories(${DETECTION_EXECUTABLE} PRIVATE  
    ${DETECTION_INC_DIRS}
)
# Link FFTW
find_library(FFTW_LIB fftw3 REQUIRED)
target_link_libraries(${DETECTION_EXECUTABLE} ${FFTW_LIB})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${DETECTION_EXECUTABLE} "${cpp_typesupport_target}")

ament_target_dependencies(${DETECTION_EXECUTABLE} rclcpp std_msgs)

########### Installing ###########
install(TARGETS
  ${ADC_EXECUTABLE}
  ${DETECTION_EXECUTABLE}
  ${DMA_EXECUTABLE}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
