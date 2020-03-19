
find_package(OpenCV REQUIRED)
include_directories(/usr/include/opencv2)
link_directories(${OpenCV_LIBRARY_DIRS} /usr/lib/x86_64-linux-gnu)
#message(STATUS "Opencv Version: " ${OpenCV_VERSION})
message(STATUS "Opencv headers: " ${OpenCV_INCLUDE_DIRS})
message(STATUS "Opencv lib dirs: " ${OpenCV_LIBRARY_DIRS})

