# HW SDK Mock Library

`HW SDK Mock Library` provides a mock interface for GPS, Link, and Flight Controller systems, complete with randomized responses to simulate hardware SDK behavior.

## Table of Contents
- [Installation](#installation)
- [Build](#build)
- [Run](#run)
- [Structure](#structure)
- [Usage](#usage)

## Installation

### Prerequisites
You’ll need:
- **CMake** 3.8 or higher
- A **C++ compiler** supporting C++23

### Clone the Repository
First, clone this repo (assuming you know what that means):
```bash
git clone [<repository-url>](https://github.com/PavelGuzenfeld/hw-sdk-mock)
cd hw-sdk-mock
```

## Build

1. **Create a Build Directory**
   Set up a `build` directory to keep things organized:
   ```bash
   mkdir build
   cd build
   ```

2. **Configure with CMake**
   Run `cmake` to configure the build files:
   ```bash
   cmake ..
   ```

3. **Compile the Library and Demo**
   Use `make` to build both the library and an optional demo executable:
   ```bash
   make
   sudo make install
   ```

   By default, this will generate:
   - `libflight-controller.so` (shared library for the `flight-controller` SDK)
   - `flight_controller_demo` (optional demo executable)

## Run

### Running the Demo

If the build succeeded (a big if), you’ll find the demo executable in `build/`. Run it like so:

```bash
./flight_controller_demo
```

This demo executable uses the mock GPS, Link, and Flight Controller classes to produce randomized output. You’ll see messages simulating location, signal quality, and flight command responses in the console.

## Structure

- **include/**: Contains headers for `Gps`, `Link`, and `FlightController` mock classes.
- **src/**: Contains the implementation files for the mock classes.
- **build/**: Build output directory (created when you run CMake).
- **CMakeLists.txt**: CMake configuration file for building the library and demo.

## Usage

After building, link against `libflight-controller.so` to use these mocks in your project. If you want to link against it, here’s a quick example:

```cpp
#include "gps/gps.hpp"
#include "link/link.hpp"
#include "flight-controller/flight_controller.hpp"

// Usage example for Gps, Link, and FlightController classes
```

### Linking with Your Own CMake Project

If you’re adding this library to your own CMake project, link it like this:

```cmake
find_package(flight-controller REQUIRED)
target_link_libraries(your_target PRIVATE flight-controller)
```

### Testing

Feel free to mess with the demo or extend the library for your testing needs.
