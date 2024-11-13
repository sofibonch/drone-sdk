# drone-sdk/CMakeLists.txt

cmake_minimum_required(VERSION 3.10)

project(drone-sdk-proj)

find_package(Boost) 
# find_package(Boost COMPONENTS sml signals2) 
message(Boost_found="${Boost_FOUND}")
message(Boost_INCLUDE_DIRS="${Boost_INCLUDE_DIRS}")
message(Boost_LIBRARIES="${Boost_LIBRARIES}")

# Include subdirectories
add_subdirectory(drone-app-sdk)
add_subdirectory(hw-sdk-mock/flight-controller)
add_subdirectory(hw-sdk-mock/gps)
add_subdirectory(hw-sdk-mock/link)

