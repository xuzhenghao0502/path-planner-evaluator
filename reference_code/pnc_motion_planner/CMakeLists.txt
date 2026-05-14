cmake_minimum_required(VERSION 3.14...3.27)

# 默认 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 软件版本
file(STRINGS version.txt PNC_MOTION_PLANNER_VERSION)
string(TIMESTAMP PNC_MOTION_PLANNER_TIMESTAMP "%Y-%m-%d %H:%M:%S")
execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE PNC_MOTION_PLANNER_COMMIT_ID
  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET
)

# 工程相关
project(
  pnc_motion_planner
  VERSION ${PNC_MOTION_PLANNER_VERSION}
  DESCRIPTION "Geometrical-Pal motion planner"
  LANGUAGES CXX
)
set(PROJECT_NAMESPACE gpal)

# 依赖项
find_package(eka-rt REQUIRED COMPONENTS ALL)
find_package(gpal-interface CONFIG REQUIRED)
find_package(pnc_common_tool REQUIRED)
find_package(pnc_common_util REQUIRED)
find_package(pnc_common_interface REQUIRED)
find_package(pnc_solver REQUIRED)
find_package(world_model REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system)
find_package(OsqpEigen REQUIRED)
find_package(fmt REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(eka-diag REQUIRED)
find_package(eka-tsync-client REQUIRED)


# 通用设置
ert_check()
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized")

add_subdirectory(config_proto)
add_subdirectory(path_planner)
add_subdirectory(speed_planner)
add_subdirectory(spatiotemporal_planner)
add_subdirectory(redundant_planner)

set(${PROJECT_NAME}_OBJECTS
  $<TARGET_OBJECTS:config_proto>
  $<TARGET_OBJECTS:path_planner>
  $<TARGET_OBJECTS:speed_planner>
  $<TARGET_OBJECTS:spatiotemporal_planner>
  $<TARGET_OBJECTS:redundant_planner>
)

pnc_add_library(
  ${PROJECT_NAME}
  SHARED
  ${${PROJECT_NAME}_OBJECTS}
  INCLUDE_DIR
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${CMAKE_PROJECT_NAME}>
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  PUBLIC_HEADER DESTINATION include
)

# export
ert_export(${CMAKE_PROJECT_NAME} ${PROJECT_NAME} ${PROJECT_NAMESPACE}:: ${CMAKE_PROJECT_VERSION})


# gtest
# 单元测试开关
set(BUILD_TESTING OFF)
if(BUILD_TESTING)
  enable_testing()
  target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -fprofile-arcs -ftest-coverage)
  target_compile_options(${PROJECT_NAME} PRIVATE -fno-exceptions -fno-signaling-nans -fno-trapping-math -O2)
  ert_test_coverage(
    ${CMAKE_PROJECT_NAME}
    TYPE
    lcov
    EXCLUDES
    */tests/*
    */config_proto/*
    */speed_planner/include/*
    */speed_planner/src/empirical_speed_planner/*
    */path_planner/include/*
    */path_planner/src/openspace_path_planner/*
    */speed_planner/src/speed_planner/park_speed_planner
    */speed_planner/src/speed_planner/acc_speed_planner
    */spatiotemporal_planner/include/*
    */spatiotemporal_planner/src/*
    */redundant_planner/include/*
    */redundant_planner/src/*
  )

  add_subdirectory(tests)
endif()

# message
message("")
message("┌──────── PNC motion planner Options ────────────────────────────────────────")
message("│ Project:               ${CMAKE_PROJECT_NAME}")
message("│ Version:               ${CMAKE_PROJECT_VERSION}")
message("│ Time stamp:            ${PNC_MOTION_PLANNER_TIMESTAMP}")
message("│ Git commit-id:         ${PNC_MOTION_PLANNER_COMMIT_ID}")
message("│ Install prefix:        ${CMAKE_INSTALL_PREFIX}")
message("│ Compiler:              ${CMAKE_CXX_COMPILER}")
message("│ CMAKE_BUILD_TYPE:      ${CMAKE_BUILD_TYPE}")
message("└────────────────────────────────────────────────────────────────────────────")
message("")
