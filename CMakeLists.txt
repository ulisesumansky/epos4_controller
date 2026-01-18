cmake_minimum_required(VERSION 3.10)
project(Epos4Controller VERSION 1.0.0 LANGUAGES CXX)

# --- Standard Configuration ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Includes standard paths variables (CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_INCLUDEDIR, etc.)
# Critical for Yocto/Cross-compilation
include(GNUInstallDirs)

# --- Dependencies ---
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

# --- Build Target ---
add_library(Epos4Controller SHARED 
    src/epos4_controller.cpp 
)

# --- Include Directories ---
target_include_directories(Epos4Controller PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> # Path for installed headers
    ${SOEM_INCLUDE_DIR}
)

target_link_libraries(Epos4Controller PUBLIC 
    ${SOEM_LIBRARY} 
    Threads::Threads
)

# --- INSTALLATION RULES (Required for Yocto/Downstream) ---

# 1. Install the Shared Library (.so)
install(TARGETS Epos4Controller
    EXPORT Epos4ControllerTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 2. Install the Headers (.hpp)
# This copies everything from your local 'include' folder to /usr/include/ (or prefix)
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# --- Debug Info ---
message(STATUS "Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Lib Install Dir: ${CMAKE_INSTALL_LIBDIR}")
