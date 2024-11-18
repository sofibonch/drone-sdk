# Set the C++ standard for the drone-app-sdk target
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add strict compile options to catch all your mistakes
add_compile_options(-Wformat -Wall -Wextra -Werror -Wconversion -Wshadow -Wunreachable-code -Wunused -Wunused-function)

# Include directories for headers
include_directories()
include_directories(
    ${PROJECT_SOURCE_DIR}/external # For custom external libs like sml.hpp
    ${Boost_INCLUDE_DIRS} # For Boost headers (including signals2)
)

# Add Google Test
add_subdirectory(external/googletest)
include_directories(external/googletest/include)

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
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include # Include drone-app-sdk headers
    ${Boost_INCLUDE_DIRS} # Include Boost headers (if available)
)

message(STATUS "External include directory: ${PROJECT_SOURCE_DIR}/external")
message(STATUS "Boost include directories: ${Boost_INCLUDE_DIRS}")


#---demos---


# Define the executable target for the Drone SDK demo
add_executable(drone-demo
    demo/drone_demo.cpp
    src/drone_sdk.cpp
    src/drone_controller.cpp
    src/command_controller.cpp
    src/state_machines/safety_state_machine.cpp
    src/state_machines/flight_state_machine.cpp
    src/state_machines/command_state_machine.cpp)

# Include directories for drone-demo (including drone-sdk headers and Boost)
target_include_directories(drone-demo PRIVATE
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${PROJECT_SOURCE_DIR}/flight-controller/include
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/link/include
    ${Boost_INCLUDE_DIRS} # Include Boost headers
)

# Link drone-demo with the required libraries
target_link_libraries(drone-demo PRIVATE
    flight-controller
    gps
    link
)

# Define the executable target for the GPS handler demo
add_executable(gps-handler-demo demo/gps_handler_demo.cpp)

# Include dirs for GPS handler demo and link gps
target_include_directories(gps-handler-demo PRIVATE
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${Boost_INCLUDE_DIRS} # Include Boost headers
)
target_link_libraries(gps-handler-demo PRIVATE gps)


# Define the executable target for the Link handler demo
add_executable(link-handler-demo demo/link_handler_demo.cpp)

# Include dirs for link handler demo and link link
target_include_directories(link-handler-demo PRIVATE
    ${PROJECT_SOURCE_DIR}/link/include
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${Boost_INCLUDE_DIRS} # Include Boost headers
)
target_link_libraries(link-handler-demo PRIVATE link)


# Define the executable target for the HardwareMonitor demo
add_executable(hw-monitor-demo demo/hw_monitor_demo.cpp)

# Include directories and link libraries for hw-monitor-demo (including gps and link)
target_include_directories(hw-monitor-demo PRIVATE
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/link/include # Include Link handler headers
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${Boost_INCLUDE_DIRS} # Include Boost headers
)

# Link libraries for hw-monitor-demo (link gps and link)
target_link_libraries(hw-monitor-demo PRIVATE gps link)


# Define the executable target for the Flight Controller handler demo
add_executable(flight-controller-handler-demo demo/flight_controller_handler_demo.cpp)

# Include directories and link libraries for flight-controller-handler-demo
target_include_directories(flight-controller-handler-demo PRIVATE
    ${PROJECT_SOURCE_DIR}/flight-controller/include
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${Boost_INCLUDE_DIRS} # Include Boost headers
)

# Link libraries for flight-controller-handler-demo (link flight-controller)
target_link_libraries(flight-controller-handler-demo PRIVATE flight-controller)


#---test----

# Add mocks for testing
add_library(mocks STATIC
    tests/mocks/mock_gps.hpp
    tests/mocks/mock_link.hpp
    tests/mocks/dummy_cpp.cpp
)


#---flight state machine test---
add_executable(flight_sm_test 
    tests/unit/flight_sm_test.cpp
    src/state_machines/flight_state_machine.cpp)

# Include directories and link libraries for the SafetyStateMachine test
target_include_directories(flight_sm_test PRIVATE
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/link/include
    ${Boost_INCLUDE_DIRS}
    external/googletest/include
)

target_link_libraries(flight_sm_test PRIVATE
    gtest
    gtest_main
    flight-controller
    gps
    link
    mocks
)


#--- command state macghine test---
add_executable(command_sm_test
    tests/unit/command_sm_test.cpp
    src/state_machines/command_state_machine.cpp
)

# Include directories for the CommandStateMachine test
target_include_directories(command_sm_test PRIVATE
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/link/include
    ${Boost_INCLUDE_DIRS}
    external/googletest/include
)

# Link libraries for the CommandStateMachine test
target_link_libraries(command_sm_test PRIVATE
    gtest
    gtest_main
    flight-controller
    gps
    link
    mocks
)

#---safety state machine test---

add_executable(safety_sm_test
    tests/unit/safety_sm_test.cpp
    src/state_machines/safety_state_machine.cpp)

# Include directories and link libraries for the SafetyStateMachine test
target_include_directories(safety_sm_test PRIVATE
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/link/include
    ${Boost_INCLUDE_DIRS}
    external/googletest/include
)

target_link_libraries(safety_sm_test PRIVATE
    gtest
    gtest_main
    flight-controller
    gps
    link
    mocks
)


#---sm manager test---
add_executable(manager_sm_test
    tests/unit/manager_sm_test.cpp
    src/state_machines/safety_state_machine.cpp
    src/state_machines/command_state_machine.cpp
    src/state_machines/flight_state_machine.cpp)

# Include directories and link libraries for the SafetyStateMachine test
target_include_directories(manager_sm_test PRIVATE
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/link/include
    ${Boost_INCLUDE_DIRS}
    external/googletest/include
)

target_link_libraries(manager_sm_test PRIVATE
    gtest
    gtest_main
    flight-controller
    gps
    link
    mocks
)


#---vommand controller test---
add_executable(command_controller_test
    tests/unit/command_controller_test.cpp
    src/command_controller.cpp
    )

# Include directories and link libraries for the SafetyStateMachine test
target_include_directories(command_controller_test PRIVATE
    ${PROJECT_SOURCE_DIR}/flight-controller/include
    ${PROJECT_SOURCE_DIR}/drone-app-sdk/include
    ${PROJECT_SOURCE_DIR}/gps/include
    ${PROJECT_SOURCE_DIR}/link/include
    ${Boost_INCLUDE_DIRS}
    external/googletest/include
)
target_link_libraries(command_controller_test PRIVATE gtest gmock gtest_main gmock_main)

target_link_libraries(command_controller_test PRIVATE
    gtest
    gtest_main
    flight-controller
    gps
    link
    mocks
    gmock
    gmock_main
)
