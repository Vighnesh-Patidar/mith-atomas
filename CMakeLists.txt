cmake_minimum_required(VERSION 3.21)
project(mith-atomas
    VERSION 0.1.0
    DESCRIPTION "Swarm robotics orchestration runtime"
    LANGUAGES C CXX
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
    src/core/json_writer.cpp
    src/core/trace_sink.cpp
    src/core/world.cpp
    src/comms/neighbour_table.cpp
    src/comms/beacon_system.cpp
    src/systems/flocking_system.cpp
    src/systems/kinematics_system.cpp
    src/systems/fault_monitor_system.cpp
    src/sim/sim_clock.cpp
    src/sim/sim_bus.cpp
)

# World::run() uses std::this_thread::sleep_until → needs pthreads on Linux.
find_package(Threads REQUIRED)
target_link_libraries(mith PUBLIC Threads::Threads)
add_library(mith::mith ALIAS mith)

# --- Cryptographic identity (§3.3) — gated by MITH_ENABLE_AUTH ---
if(MITH_ENABLE_AUTH)
    target_sources(mith PRIVATE
        third_party/tweetnacl/tweetnacl.c
        src/identity/chacha20.cpp       # provides extern "C" randombytes() too
        src/identity/ed25519.cpp
    )
    target_include_directories(mith PRIVATE third_party/tweetnacl)
    target_compile_definitions(mith PUBLIC MITH_AUTH_ENABLED)
    # TweetNaCl is C; suppress noisy warnings from C-style code in vendor.
    set_source_files_properties(third_party/tweetnacl/tweetnacl.c
        PROPERTIES COMPILE_OPTIONS "-w")
endif()

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

# --- Install / package config ---
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS mith
    EXPORT  mith-targets
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/mith
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
)

install(EXPORT mith-targets
    FILE      mith-atomas-targets.cmake
    NAMESPACE mith::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mith-atomas
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/mith-atomas-config-version.cmake"
    VERSION       ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mith-atomas-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/mith-atomas-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mith-atomas
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mith-atomas-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/mith-atomas-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mith-atomas
)
