# Require a reasonably modern CMake version so the project can use the
# features used below.
cmake_minimum_required(VERSION 3.16)

# Name the project and declare that it uses the C++ language.
project(UltimateGPS_Library LANGUAGES CXX)

# Use the C++17 language standard for the build.
set(CMAKE_CXX_STANDARD 17)

# Require that C++17 is available; fail if the compiler does not support it.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Disable compiler-specific language extensions to keep the build more portable.
set(CMAKE_CXX_EXTENSIONS OFF)

# Collect all C++ source files under src/ into a list. The CONFIGURE_DEPENDS
# option causes CMake to re-run the glob when files change.
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    *.cpp
)

# Define the library target named UltimateGPS using the discovered source files.
add_library(UltimateGPS ${PROJECT_SOURCES})

# Make the project root, src/, and lib/ directories available to the compiler
# so headers can be included from those locations.

find_library(GPIOD_LIB gpiod REQUIRED)
target_link_libraries(UltimateGPS PRIVATE ${GPIOD_LIB})

# Turn on common warning flags to help catch issues during compilation.
target_compile_options(UltimateGPS PRIVATE -Wall -Wextra -Wpedantic)

# ============================================================================
# DEBUG
# ============================================================================
# Check if the user explicitly requested a Debug build via -DCMAKE_BUILD_TYPE=Debug
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Debug build detected. Adding full debug symbols.")

    # -g adds debug symbols; -O0 disables optimizations so gdb can step accurately
    target_compile_options(UltimateGPS PRIVATE -g -O0)
# ===========================================================================
# Release
# ==========================================================================
else()
    message(STATUS "Standard build detected.")
endif()
# ============================================================================
