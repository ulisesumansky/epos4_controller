cmake_minimum_required(VERSION 3.10)
project(Epos4Controller VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)

find_package(Threads REQUIRED)

find_path(SOEM_INCLUDE_DIR 
    NAMES soem/soem.h
    PATH_SUFFIXES soem
)

find_library(SOEM_LIBRARY 
    NAMES soem
)

if (NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIBRARY)
    message(FATAL_ERROR "SOEM library not found. Please provide CMAKE_PREFIX_PATH.")
endif()

add_library(Epos4Controller SHARED 
    src/epos4_controller.cpp 
)

target_include_directories(Epos4Controller PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> # Path for installed headers
    ${SOEM_INCLUDE_DIR}
)

target_link_libraries(Epos4Controller PUBLIC 
    ${SOEM_LIBRARY} 
    Threads::Threads
)

install(TARGETS Epos4Controller
    EXPORT Epos4ControllerTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

message(STATUS "Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Lib Install Dir: ${CMAKE_INSTALL_LIBDIR}")
