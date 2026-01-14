cmake_minimum_required(VERSION 3.10)
project(Epos4Controller VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

find_path(SOEM_INCLUDE_DIR NAMES soem/soem.h)
find_library(SOEM_LIBRARY NAMES soem)

if (NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIBRARY)
    message(FATAL_ERROR "SOEM library not found. Install it with: sudo apt install libsoem-dev (or build from source)")
endif()

add_library(Epos4Controller src/epos4_controller.cpp)

target_include_directories(Epos4Controller PUBLIC 
    include 
    ${SOEM_INCLUDE_DIR}
)

target_link_libraries(Epos4Controller PUBLIC 
    ${SOEM_LIBRARY} 
    Threads::Threads
)

