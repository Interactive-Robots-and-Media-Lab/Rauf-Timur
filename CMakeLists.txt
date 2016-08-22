cmake_minimum_required(VERSION 3.5)
project(Doorplate)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)
set(SOURCE_FILES main5.cpp)
add_executable(Doorplate ${SOURCE_FILES})

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(Doorplate ${OpenCV_LIBS} lept tesseract)

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)

# Find freenect, to set necessary variables
find_package(freenect2 REQUIRED)

# Include directories to get freenect headers
include_directories($ENV{HOME}/freenect2/include)

# Link freenect libraries with the project
target_link_libraries(${PROJECT_NAME} ${freenect2_LIBRARIES})

include_directories($ENV{HOME}/ClionProjects/Doorplate)