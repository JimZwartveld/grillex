cmake_minimum_required(VERSION 3.20)
project(grillex VERSION 2.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# On macOS, ensure C++ standard library is found
if(APPLE)
    # Add explicit include path for libc++ headers
    include_directories(SYSTEM /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include/c++/v1)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

# Use FetchContent to get pybind11 (ensures compatible version)
include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(pybind11)

# Find Eigen (should be installed via system package manager)
# Accept any version of Eigen3 (3.x or 5.x)
find_package(Eigen3 REQUIRED NO_MODULE)

# Add subdirectory for C++ sources
add_subdirectory(cpp)
