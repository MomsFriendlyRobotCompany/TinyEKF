cmake_minimum_required(VERSION 3.16)
project(tinyekf VERSION 0.1.0 LANGUAGES C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
# cmake_host_system_information(RESULT CORES QUERY NUMBER_OF_LOGICAL_CORES)
cmake_host_system_information(RESULT OSN QUERY OS_NAME)
cmake_host_system_information(RESULT OS_VERSION QUERY OS_RELEASE)
cmake_host_system_information(RESULT PROC QUERY PROCESSOR_DESCRIPTION)

# set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "" FORCE)

message(STATUS "-------------------------------------")
message(STATUS "  Project: ${PROJECT_NAME}")
message(STATUS "  C ${CMAKE_C_STANDARD}")
message(STATUS "-------------------------------------")
message(STATUS " ${HOST}")
message(STATUS " ${OSN}: ${OS_VERSION}")
message(STATUS " ${PROC}")
message(STATUS " ${CMAKE_BUILD_TYPE}")
message(STATUS "-------------------------------------")

# Library ---------------------------------------------------------------
add_library(${PROJECT_NAME}
    src/tiny_ekf.c
)
target_include_directories(${PROJECT_NAME} PUBLIC ./src)
target_link_libraries(${PROJECT_NAME} PUBLIC m)

# Example --------------------------------------------------------------
add_executable(gps examples/gps/gps_ekf.c)
target_include_directories(gps PUBLIC ./examples/gps ./src)
target_link_libraries(gps PUBLIC ${PROJECT_NAME})
