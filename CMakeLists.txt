cmake_minimum_required(VERSION 2.8)
project(visa-visp)

set(CMAKE_BUILD_TYPE Debug)

set (CMAKE_CXX_STANDARD 11)

include_directories(SYSTEM
    3rdparty/
    src/
)

file(GLOB SOURCES
    3rdparty/cpp-base64/base64.cpp
    src/vpVisaAdapter.cpp
    src/vpVisaAdapter.h
)

find_package( OpenCV QUIET )
if(OpenCV_FOUND)
    message("With OpenCV")
    add_definitions(-DWITH_OPENCV)
    include_directories(${OPENCV_INCLUDE_DIRS})
else()
    message("OpenCV not found")
endif()

find_package(VISP QUIET)
if(VISP_FOUND)
    message("With ViSP")
    add_definitions(-DWITH_VISP)
    include_directories(${VISP_INCLUDE_DIRS})
else()
    message("ViSP not found")
endif()

add_executable(image-grab-desired-position ${SOURCES} tests/image-grab-desired-position.cpp)
target_link_libraries(image-grab-desired-position ${OpenCV_LIBS} ${VISP_LIBRARIES})

add_executable(visa-ibvs ${SOURCES} tests/visa-ibvs.cpp)
target_link_libraries(visa-ibvs ${OpenCV_LIBS} ${VISP_LIBRARIES})

add_executable(visa-controller ${SOURCES} tests/visa-controller.cpp)
target_link_libraries(visa-controller ${OpenCV_LIBS} ${VISP_LIBRARIES})

add_executable(visa-jacobian ${SOURCES} tests/visa-jacobian.cpp)
target_link_libraries(visa-jacobian ${OpenCV_LIBS} ${VISP_LIBRARIES})
