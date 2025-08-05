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

########### ADC Node ###########

set(ADC_EXECUTABLE adc_model)

set(ADC_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/adc/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/adc/model_adc.cpp
)

set(ADC_INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/adc/
)

add_executable(${ADC_EXECUTABLE}
    ${ADC_SRC_FILES}
)
target_include_directories(${ADC_EXECUTABLE} PRIVATE  
    ${ADC_INC_DIRS}
)
ament_target_dependencies(${ADC_EXECUTABLE} rclcpp std_msgs)

########### Arc detection Node ###########

set(DETECTION_EXECUTABLE detection_model)

set(DETECTION_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/detection/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/detection/model_detection.cpp
)

set(DETECTION_INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/detection/
)

add_executable(${DETECTION_EXECUTABLE}
    ${DETECTION_SRC_FILES}
)
target_include_directories(${DETECTION_EXECUTABLE} PRIVATE  
    ${DETECTION_INC_DIRS}
)
ament_target_dependencies(${DETECTION_EXECUTABLE} rclcpp std_msgs)

########### Installing ###########
install(TARGETS
  ${ADC_EXECUTABLE}
  ${DETECTION_EXECUTABLE}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
