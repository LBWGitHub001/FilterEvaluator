cmake_minimum_required(VERSION 3.8)
project(filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#导入一些CMake函数
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Function)


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###################项目目录设置######################
set(KALMAN_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/kalman)
set(KALMAN_SRC_DIR ${PROJECT_SOURCE_DIR}/src/kalman)
set(MLS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/mls)
set(MLS_SRC_DIR ${PROJECT_SOURCE_DIR}/src/mls)
set(LMS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/lms)
set(LMS_SRC_DIR ${PROJECT_SOURCE_DIR}/src/lms)
set(EXEC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/exec)
set(EXEC_SRC_DIR ${PROJECT_SOURCE_DIR}/src/exec)

file(GLOB_RECURSE EXEC_SRCS ${EXEC_SRC_DIR}/*.cpp)
file(GLOB_RECURSE KALMAN_SRCS ${KALMAN_SRC_DIR}/*.cpp)
file(GLOB_RECURSE MLS_SRCS ${MLS_SRC_DIR}/*.cpp)
file(GLOB_RECURSE LMS_SRCS ${LMS_SRC_DIR}/*.cpp)

list(APPEND ALL_INCLUDE
        ${KALMAN_INCLUDE_DIR} ${MLS_INCLUDE_DIR} ${EXEC_INCLUDE_DIR} ${LMS_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

print_var(ALL_INCLUDE)

print_var(EXEC_SRCS)
print_var(KALMAN_SRCS)
print_var(MLS_SRCS)
print_var(LMS_SRCS)

include_directories(${ALL_INCLUDE})

##############静态库编译##################
#add_library(NCURSES_LIBS "/usr/lib/x86_64-linux-gnu")
add_library(filter_kalman ${KALMAN_SRCS})
add_library(filter_mls ${MLS_SRCS})
add_library(filter_lms ${LMS_SRCS})
list(APPEND FILTER_LIBS
        ${filter_kalman} ${filter_mls} ${filter_lms}
)
print_var(FILTER_LIBS)
############项目编译构建##########################
ament_auto_add_executable(filter ${EXEC_SRCS})

target_link_libraries(filter ${OpenCV_LIBS} ${FILTER_LIBS} "/usr/lib/x86_64-linux-gnu/libncurses.so")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
