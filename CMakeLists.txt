cmake_minimum_required(VERSION 3.21)
project(mith-atomas
    VERSION 0.1.0
    DESCRIPTION "Swarm robotics orchestration runtime"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# --- Options (see ARCHITECTURE.md §11) ---
option(MITH_BUILD_EXAMPLES "Build example programs"    ON)
option(MITH_BUILD_TESTS    "Build test suite"          ON)
option(MITH_BUILD_SIM      "Build simulation harness"  ON)
option(MITH_ENABLE_UDP     "Build UDP transport"       ON)
option(MITH_ENABLE_SERIAL  "Build serial transport"    OFF)
option(MITH_ENABLE_AUTH    "Build cryptographic identity (Ed25519, signed mode)" OFF)

# --- Core static library ---
add_library(mith STATIC
    src/identity/uuid.cpp
    src/identity/hierarchical_id.cpp
    src/core/registry.cpp
    src/core/scheduler.cpp
)
add_library(mith::mith ALIAS mith)

target_include_directories(mith PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(mith PUBLIC cxx_std_17)
target_compile_options(mith PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wshadow -Wconversion>
)

# --- Subdirectories (enabled as content lands) ---
if(MITH_BUILD_EXAMPLES AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt)
    add_subdirectory(examples)
endif()

if(MITH_BUILD_TESTS AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    enable_testing()
    add_subdirectory(tests)
endif()
