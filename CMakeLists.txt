cmake_minimum_required(VERSION 3.16)
project(orthanc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(Threads REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# Include directories
include_directories(${CURL_INCLUDE_DIRS})

# Source files
add_executable(orthanc_tracker
    src/cpp/main.cpp
    src/cpp/Turret.cpp
    src/cpp/ServoController.cpp
    src/cpp/SignalGenerator.cpp
)

# Link libraries
target_link_libraries(orthanc_tracker
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
    Threads::Threads
    pigpiod_if2
)

# Build interactive menu (for testing)
add_executable(orthanc_menu
    tests/cpp/menu.cpp
    src/cpp/Turret.cpp
    src/cpp/ServoController.cpp
    src/cpp/SignalGenerator.cpp
)

target_link_libraries(orthanc_menu
    Threads::Threads
    pigpiod_if2
)