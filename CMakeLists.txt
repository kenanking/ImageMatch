cmake_minimum_required(VERSION 3.0.0)
project(ImageMatch VERSION 0.1.0)

# set(CMAKE_CXX_FLAGS "-std=c++17 -O2")

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    "./thirdparty/eigen/3.4.0_1/include"
)

add_executable(ImageMatch main/main.cpp src/detector.cpp src/matcher.cpp)
target_link_libraries(ImageMatch ${OpenCV_LIBS})