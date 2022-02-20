cmake_minimum_required (VERSION 3.15)
project (ekf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Python2 COMPONENTS Development Interpreter NumPy)

find_package(Eigen3 REQUIRED)

#find_package(PythonInterp 2.7 REQUIRED)
#find_package(PythonLibs 2.7 REQUIRED)

include_directories(
    include
    /usr/include/python2.7/
    /usr/bin/
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ekf src/ekf.cpp)

target_include_directories(ekf PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})

target_link_libraries(ekf Python2::Python Python2::NumPy)
