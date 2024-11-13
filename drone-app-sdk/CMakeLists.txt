# Set the C++ standard for the drone-app-sdk target
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add strict compile options to catch all your mistakes
add_compile_options(-Wformat -Wall -Wextra -pedantic -Werror -Wconversion -Wshadow -Wunreachable-code -Wunused -Wunused-function)

# Include directories for headers
include_directories(include)

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
    ${Boost_INCLUDE_DIRS}  # Include Boost headers
)

# Define the executable target for the GPS handler demo
add_executable(drone-gps-handler-demo tests/gps_handler_demo.cpp)

# Include dirs for GPS handler demo and link gps
target_include_directories(drone-gps-handler-demo PRIVATE 
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${Boost_INCLUDE_DIRS}  # Include Boost headers
)
target_link_libraries(drone-gps-handler-demo PRIVATE gps)
