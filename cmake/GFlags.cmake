find_package(GFlags REQUIRED)
message (STATUS "Gflag: ${GFLAGS_INCLUDE_DIRS}")
include_directories(${GFLAGS_INCLUDE_DIRS})
