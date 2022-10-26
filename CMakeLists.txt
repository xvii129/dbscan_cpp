cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(dbscan)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS}  -O3 -Wall -w -g -ggdb -std=c++14 -lpthread")
set(BOOST_THREAD_LIBRARY boost_thread)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS})

# find_package(catkin REQUIRED COMPONENTS
    # roscpp
    # std_msgs
    # geometry_msgs
    # message_generation
# )

# find_package(gflags REQUIRED)
# find_package(Protobuf REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(absl REQUIRED)
# find_package(Torch REQUIRED)
# find_package(CUDA 11.1 REQUIRED)

# catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES apollo
    # CATKIN_DEPENDS  message_runtime
    # DEPENDS system_lib
# )

message("-- project source dir: " ${PROJECT_SOURCE_DIR})

include_directories(
    ${PROJECT_SOURCE_DIR}    
    ${PROJECT_SOURCE_DIR}/src/
    ${PROJECT_SOURCE_DIR}/src/dbscan/
)

add_library(cluster SHARED
    ${PROJECT_SOURCE_DIR}/src/dbscan/pedestrian_cluster.cc
)

add_executable(dbscan_bin
    ${PROJECT_SOURCE_DIR}/src/obstacles_process.cc
    ${PROJECT_SOURCE_DIR}/src/main.cc
)

target_link_libraries(dbscan_bin
    cluster
    # ${catkin_LIBRARIES}
    # ${PROJECT_NAME}

    # /usr/lib/gcc/x86_64-linux-gnu/7.5.0/libatomic.so
    # /usr/local/lib/libabsl_strings_internal.a

    # /usr/local/lib/libprotoc.a
    # /usr/local/lib/libglog.so
)
