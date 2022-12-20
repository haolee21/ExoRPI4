cmake_minimum_required(VERSION 3.0.0)
project(ExoController)
set(CMAKE_SYSTEM_NAME Linux) 
set(CMAKE_SYSTEM_PROCESSOR arm)

#setup boost library
set(Boost_INCLUDE_DIR /home/lowlimb/tools/ARM/boost_1_75_0)
set(Boost_LIBRARY_DIR /home/lowlimb/tools/ARM/boost_1_75_0/stage/lib)

# include eigen
set(EIGEN_INCLUDE_DIR /home/lowlimb/tools/eigen/eigen-master)

# include osqp
set(osqp_INCLUDE_DIRS /home/lowlimb/tools/ARM/osqp/include)
set(osqp_LIBRARIES /home/lowlimb/tools/ARM/osqp/lib)

# include nlohmann (json lib)
set(JSON_INCLUDE_DIRS /home/lowlimb/tools/ARM/nlohmann)

set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

set(Boost_USE_STATIC_LIBS ON)


set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -pthread -lrt -O3") #you can set with add_definitions("-Wall -std=c+11, etc")
set(CMAKE_C_FLAGS   "-std=c++17 -Wall -pthread -lrt -O3")

# Search for programs only in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Search for libraries and headers only in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(Include_DIR include)
set(COMMON_DIR ../RPI_Teensy)
find_package(Boost 1.75.0 COMPONENTS serialization date_time filesystem REQUIRED)
find_library(osqp_LIB osqp ${osqp_LIBRARIES})







include_directories(
    ${Include_DIR}
    # src
    ${Boost_INCLUDE_DIR}
    ${COMMON_DIR}
    ${EIGEN_INCLUDE_DIR}
    ${osqp_INCLUDE_DIRS}
    ${JSON_INCLUDE_DIRS}
    
)
# file(GLOB HEAD_FILES 
#     ${Include_DIR}/*.h 
#     ${Include_DIR}/*.hpp
#     ${COMMON_DIR}/*.h)
# add_library(
#     llexo STATIC
#     ${HEAD_FILES}
    
# )

# message(STATUS ${BASE_DIR})
# set_target_properties(llexo PROPERTIES LINKER_LANGUAGE CXX)
# link_directories(${Boost_LIBRARY_DIR}) #unnecessary 

file(GLOB SRC_FILES 
    # ${Include_DIR}/*.cpp
    src/*.cpp)
add_executable(exo_o ${SRC_FILES})

target_link_libraries(exo_o PRIVATE ${Boost_LIBRARIES} ${osqp_LIB}) 
target_precompile_headers(exo_o 
    PUBLIC
        include/ADC.hpp
        include/DigitalFilter.hpp
        include/Encoder.hpp
        include/Pin.hpp
        include/Recorder.hpp
        include/Teensy.hpp
        ${JSON_INCLUDE_DIRS}/json.hpp
)

