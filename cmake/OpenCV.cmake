
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/lib)
#message(STATUS "Opencv Version: " ${OpenCV_VERSION})
#message(STATUS "Opencv headers: " ${OpenCV_INCLUDE_DIRS})
#message(STATUS "Opencv lib dirs: " ${OpenCV_LIBRARY_DIRS})

