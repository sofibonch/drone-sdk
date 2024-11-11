cmake_minimum_required(VERSION 3.8)
project(flight-controller)

# Set C++ standard (keeping it on the bleeding edge with C++23)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add strict compile options to catch all your mistakes
add_compile_options(-Wformat -Wall -Wextra -pedantic -Werror -Wconversion -Wshadow -Wunreachable-code -Wunused -Wunused-function)

# Include directories for headers
include_directories(include)

# Add source files for the FlightController
set(SOURCE_FILES
    src/flight_controller.cpp
)

# Define the shared library target named after the project
add_library(flight-controller SHARED ${SOURCE_FILES})

# Specify where the headers are (using the actual library target name)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Add an executable for testing the library
add_executable(flight_controller_demo src/demo.cpp)
target_link_libraries(flight_controller_demo flight-controller)

# Install the library with the project name as target
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Set up an export for find_package functionality
install(EXPORT ${PROJECT_NAME}
    FILE flight-controllerConfig.cmake
    DESTINATION lib/cmake/flight-controller
)

# Install the headers
install(DIRECTORY include/ DESTINATION include)

# Install the demo executable
install(TARGETS flight_controller_demo
    RUNTIME DESTINATION bin
)
