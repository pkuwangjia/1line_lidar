cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_C_COMPILER /usr/bin/gcc CACHE PATH "")
set(CMAKE_CXX_COMPILER /usr/bin/g++ CACHE PATH "")
project(1line_algo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -Wall -Wno-deprecated-declarations")
set(CMAKE_BUILD_TYPE Release)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O0 -Wall -g -ggdb")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -O0 -Wall -g -ggdb")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O2")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -O2")
endif ()
set(WORK_ROOT ${CMAKE_SOURCE_DIR})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)
include(${CMAKE_SOURCE_DIR}/cmake/Boost.cmake)
# # Cyber
include(${CMAKE_SOURCE_DIR}/cmake/Cyber.cmake)
# # Common
include(${CMAKE_SOURCE_DIR}/cmake/Common.cmake)

set(PROTOBUF_ROOT "../gears/x86_64" CACHE INTERNAL "PROTOBUF_ROOT")
include(${CMAKE_SOURCE_DIR}/cmake/Protobuf.cmake)
# # GLog
include(${CMAKE_SOURCE_DIR}/cmake/GLog.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/GFlags.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/PCL.cmake)
# include(${CMAKE_SOURCE_DIR}/cmake/OpenCV.cmake)

message("pcllibrary:${PCL_LIBRARY_DIRS}")
message("pcllibrary:${PCL_INCLUDE_DIRS}")
add_definitions(-DMODULE_NAME="${PROJECT_NAME}")

include_directories(include)

aux_source_directory(${PROJECT_SOURCE_DIR} PROJ_SRCS)

add_executable(${PROJECT_NAME}  ${PROJ_SRCS})
target_link_libraries(${PROJECT_NAME} 
/home/caros/Desktop/CyberRT/opt/lib/libcyber.so
    /home/caros/Desktop/CyberRT/opt/lib/libneolix_common.so
    /home/caros/Desktop/CyberRT/gears/x86_64/lib/libprotobuf.so
    glog
    gflags
    boost_system
    boost_thread
    ${VTK_LIBRARIES}
    pcl_common
    pcl_io
    pcl_visualization
    X11   
        # opencv_bgsegm
        # opencv_bioinspired
        # opencv_calib3d
        # opencv_ccalib
        # opencv_core
        # opencv_datasets
        # opencv_dpm
        # opencv_face
        # opencv_features2d
        # opencv_flann
        # opencv_fuzzy
        # opencv_hdf
        # opencv_highgui
        # opencv_imgcodecs
        # opencv_imgproc
        # opencv_line_descriptor
        # opencv_ml
        # opencv_objdetect
        # opencv_optflow
        # opencv_phase_unwrapping
        # opencv_photo
        # opencv_plot
        # opencv_reg
        # opencv_rgbd
        # opencv_saliency
        # opencv_shape
        # opencv_stereo
        # opencv_stitching
        # opencv_structured_light
        # opencv_superres
        # opencv_surface_matching
        # opencv_text
        # opencv_video
        # opencv_videoio
        # opencv_videostab
        # opencv_viz
        # opencv_ximgproc
        # opencv_xobjdetect
        # opencv_xphoto
    )

install(TARGETS ${PROJECT_NAME} 
 DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
 install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/conf/
 DESTINATION ${CMAKE_INSTALL_PREFIX}/conf)