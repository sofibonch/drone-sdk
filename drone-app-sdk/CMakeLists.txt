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
target_link_libraries(drone-app-sdk PRIVATE flight-controller)
target_link_libraries(drone-app-sdk PRIVATE gps)
target_link_libraries(drone-app-sdk PRIVATE link)

# include dirs
target_include_directories(drone-app-sdk PRIVATE ${PROJECT_SOURCE_DIR}/flight-controller/include)
target_include_directories(drone-app-sdk PRIVATE ${PROJECT_SOURCE_DIR}/gps/include)
target_include_directories(drone-app-sdk PRIVATE ${PROJECT_SOURCE_DIR}/link/include)