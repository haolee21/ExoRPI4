cmake_minimum_required(VERSION 3.10)
project(osqp_test)

set(CMAKE_SYSTEM_NAME Linux) 
# set(CMAKE_SYSTEM_PROCESSOR arm)
# set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc) //setting these are useless, you should do it when init cmake 
# set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

set(osqp_INCLUDE_DIRS /home/lowlimb/tools/x86/osqp/include)
set(osqp_LIBRARIES /home/lowlimb/tools/x86/osqp/lib)
# set(osqp_DIR home/lowlimb/tools/ARM/osqp/lib/cmake/osqp)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -o3")
set(CMAKE_C_FLAGS "-Wall -o3")


include_directories(
    ${osqp_INCLUDE_DIRS}
)
add_library(
    lib STATIC
    MPC.h
)
set_target_properties(lib PROPERTIES LINKER_LANGUAGE CXX)
#find_package(osqp REQUIRED)

add_executable(osqp_ex osqp_demo.cpp)

find_library(osqp_LIB osqp ${osqp_LIBRARIES})
target_link_libraries(osqp_ex PRIVATE ${osqp_LIB})