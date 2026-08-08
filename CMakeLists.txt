cmake_minimum_required(VERSION 3.16)

project(MachineVision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "/opt/homebrew/opt/opencv@4/lib/cmake/opencv4")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(MachineVision
    main.cpp
)

target_link_libraries(MachineVision
    ${OpenCV_LIBS}
)

add_executable(Calibration
    calibration.cpp
)

target_link_libraries(Calibration
    ${OpenCV_LIBS}
)

add_executable(Homography
    homography.cpp
)

target_link_libraries(Homography
    ${OpenCV_LIBS}
)