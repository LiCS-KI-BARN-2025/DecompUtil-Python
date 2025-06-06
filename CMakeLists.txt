cmake_minimum_required(VERSION 3.5)
project(decomp_util)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-deprecated-declarations")

set(BUILD_SHARED_LIBS ON)

# Find required packages
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 REQUIRED)

# Add pybind11
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.11.1
)
FetchContent_MakeAvailable(pybind11)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

include(CTest)

add_executable(test_seed_decomp test/test_seed_decomp.cpp)
target_link_libraries(test_seed_decomp ${Boost_LIBRARIES})
add_test(test_seed_decomp test_seed_decomp)

add_executable(test_line_segment test/test_line_segment.cpp)
target_link_libraries(test_line_segment ${Boost_LIBRARIES})
add_test(test_line_segment test_line_segment ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

add_executable(test_ellipsoid_decomp test/test_ellipsoid_decomp.cpp)
target_link_libraries(test_ellipsoid_decomp ${Boost_LIBRARIES})
add_test(test_ellipsoid_decomp test_ellipsoid_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

add_executable(test_iterative_decomp test/test_iterative_decomp.cpp)
target_link_libraries(test_iterative_decomp ${Boost_LIBRARIES})
add_test(test_iterative_decomp test_iterative_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

# Python bindings
pybind11_add_module(pydecomp_util src/python_bindings.cpp)
target_link_libraries(pydecomp_util PRIVATE Eigen3::Eigen)

install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/decomp_basis include/decomp_geometry include/decomp_util
  DESTINATION include)
