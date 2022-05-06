cmake_minimum_required(VERSION 3.0.0)
project(ExoController)
set(CMAKE_SYSTEM_NAME Linux) 
set(CMAKE_SYSTEM_PROCESSOR arm)

#setup boost library
set(Boost_INCLUDE_DIR /home/lowlimb/tools/ARM/boost_1_75_0)
set(Boost_LIBRARY_DIR /home/lowlimb/tools/ARM/boost_1_75_0/stage/lib)

# #setup osqp library
# set(OSQP_)

set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

set(Boost_USE_STATIC_LIBS ON)


set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -o3 -pthread -lrt -o3") #you can set with add_definitions("-Wall -std=c+11, etc")
set(CMAKE_C_FLAGS   "-std=c++17 -Wall -o3 -pthread -lrt -o3")

# Search for programs only in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Search for libraries and headers only in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(Include_DIR include)
set(COMMON_DIR /home/lowlimb/cdrive/UCLA/lab/Exoskeleton/Controller/ExoRPI4/RPI_Teensy)
find_package(Boost 1.74.0 COMPONENTS serialization date_time filesystem REQUIRED)
#set(Eigen_INCLUDE_DIR /home/pi/Lib/eigen-eigen-323c052e1731)







include_directories(
    ${Include_DIR}
    src
    ${Boost_INCLUDE_DIR}
    ${COMMON_DIR}
    #${Eigen_INCLUDE_DIR}

    
)
file(GLOB HEAD_FILES 
    ${Include_DIR}/*.h 
    ${Include_DIR}/*.hpp
    ${COMMON_DIR}/*.h)
add_library(
    lib STATIC
    ${HEAD_FILES}
    
)

# message(STATUS ${BASE_DIR})
set_target_properties(lib PROPERTIES LINKER_LANGUAGE CXX)
# link_directories(${Boost_LIBRARY_DIR}) #unnecessary 
enable_testing()
file(GLOB SRC_FILES 
    ${Include_DIR}/*.cpp
    src/*.cpp)
add_executable(exo_o ${SRC_FILES})

target_link_libraries(exo_o PRIVATE ${Boost_LIBRARIES}) 

