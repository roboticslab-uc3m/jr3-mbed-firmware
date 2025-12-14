cmake_minimum_required(VERSION 3.19)

project(jr3 LANGUAGES C CXX ASM)

add_library(${PROJECT_NAME} OBJECT)

target_sources(${PROJECT_NAME} PRIVATE Jr3.hpp
                                       Jr3Controller.hpp
                                       Jr3Controller.cpp
                                       utils.hpp
                                       overclocking.hpp)

target_sources(${PROJECT_NAME} PRIVATE AccurateWaiter/AccurateWaiter.h
                                       AccurateWaiter/AccurateWaiter.cpp)

target_sources(${PROJECT_NAME} PRIVATE fixedpoint/fixed_class.h
                                       fixedpoint/fixed_func.h
                                       fixedpoint/fixed_func.cpp
                                       fixedpoint/fixsintab.h
                                       fixedpoint/stdint.h)

target_link_libraries(${PROJECT_NAME} PUBLIC mbed-os)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
