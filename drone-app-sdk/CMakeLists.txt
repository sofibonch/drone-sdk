# Set the C++ standard for the drone-app-sdk target
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add strict compile options to catch all your mistakes
add_compile_options(-Wformat -Wall -Wextra -Werror -Wconversion -Wshadow -Wunreachable-code -Wunused -Wunused-function)

# Include directories for headers
include_directories(include)
include_directories(
    ${PROJECT_SOURCE_DIR}/external        # For custom external libs like sml.hpp
    ${Boost_INCLUDE_DIRS}                 # For Boost headers (including signals2)
)
# Add source files for your target
set(SOURCE_FILES
    src/drone_app_sdk.cpp
    
)

# Define the executable target for the drone app SDK
add_executable(drone-app-sdk ${SOURCE_FILES})

# Link the drone-app-sdk target with the sub libraries
target_link_libraries(drone-app-sdk PRIVATE flight-controller gps link)

# Include dirs for drone-app-sdk
target_include_directories(drone-app-sdk PRIVATE 
    ${PROJECT_SOURCE_DIR}/flight-controller/include
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/link/include
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include  # Include drone-app-sdk headers
    ${Boost_INCLUDE_DIRS}  # Include Boost headers (if available)
)

message(STATUS "External include directory: ${PROJECT_SOURCE_DIR}/external")
message(STATUS "Boost include directories: ${Boost_INCLUDE_DIRS}")
# Define the executable target for the Drone SDK demo
add_executable(drone-demo tests/drone_demo.cpp src/drone_sdk.cpp src/drone_controller.cpp)

# Include directories for drone-demo (including drone-sdk headers and Boost)
target_include_directories(drone-demo PRIVATE 
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${PROJECT_SOURCE_DIR}/flight-controller/include
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/link/include
    ${Boost_INCLUDE_DIRS}  # Include Boost headers
)

# Link drone-demo with the required libraries
target_link_libraries(drone-demo PRIVATE 
    flight-controller 
    gps 
    link
)

# Define the executable target for the GPS handler demo
add_executable(gps-handler-demo tests/gps_handler_demo.cpp)

# Include dirs for GPS handler demo and link gps
target_include_directories(gps-handler-demo PRIVATE 
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${Boost_INCLUDE_DIRS}  # Include Boost headers
)
target_link_libraries(gps-handler-demo PRIVATE gps)


# Define the executable target for the Link handler demo
add_executable(link-handler-demo tests/link_handler_demo.cpp)

# Include dirs for link handler demo and link link
target_include_directories(link-handler-demo PRIVATE 
    ${PROJECT_SOURCE_DIR}/link/include
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${Boost_INCLUDE_DIRS}  # Include Boost headers
)
target_link_libraries(link-handler-demo PRIVATE link)


# Define the executable target for the HardwareMonitor demo
add_executable(hw-monitor-demo tests/hw_monitor_demo.cpp)

# Include directories and link libraries for hw-monitor-demo (including gps and link)
target_include_directories(hw-monitor-demo PRIVATE 
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/link/include  # Include Link handler headers
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${Boost_INCLUDE_DIRS}  # Include Boost headers
)

# Link libraries for hw-monitor-demo (link gps and link)
target_link_libraries(hw-monitor-demo PRIVATE gps link)
