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
  world_model
  VERSION ${PNC_MOTION_PLANNER_VERSION}
  DESCRIPTION "Geometrical-Pal pnc world model"
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
find_package(Boost REQUIRED COMPONENTS filesystem thread system)
find_package(Protobuf CONFIG REQUIRED)
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
add_subdirectory(center_lines)
add_subdirectory(memorized_route)
add_subdirectory(navigation_data)
add_subdirectory(model)
add_subdirectory(reference_line)
add_subdirectory(obstacle)
add_subdirectory(local_view)
add_subdirectory(decision_data)
add_subdirectory(ellipse_object_info)
add_subdirectory(path_data)
add_subdirectory(speed_data)
add_subdirectory(reference_line_info)
# add_subdirectory(freespace_utils)
add_subdirectory(adas_data)
add_subdirectory(trajectory_boundary)
add_subdirectory(trajectory_data)
add_subdirectory(road_instance)

pnc_add_library(
  ${PROJECT_NAME}
  SHARED
  $<TARGET_OBJECTS:config_proto>
  $<TARGET_OBJECTS:center_lines>
  $<TARGET_OBJECTS:memorized_route>
  $<TARGET_OBJECTS:model>
  $<TARGET_OBJECTS:reference_line>
  $<TARGET_OBJECTS:obstacle>
  $<TARGET_OBJECTS:local_view>
  $<TARGET_OBJECTS:decision_data>
  $<TARGET_OBJECTS:ellipse_object_info>
  $<TARGET_OBJECTS:path_data>
  $<TARGET_OBJECTS:speed_data>
  # $<TARGET_OBJECTS:freespace_utils>
  $<TARGET_OBJECTS:navigation_data>
  $<TARGET_OBJECTS:adas_data>
  $<TARGET_OBJECTS:trajectory_data>
  $<TARGET_OBJECTS:trajectory_boundary>
  $<TARGET_OBJECTS:road_instance>
  INCLUDE_DIR
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${CMAKE_PROJECT_NAME}>
  LIBRARY
  PRIVATE
  reference_line_info
  protobuf::libprotobuf
)

install(
  TARGETS ${PROJECT_NAME}
  PUBLIC_HEADER DESTINATION include
  LIBRARY DESTINATION lib
)

# export
ert_export(${CMAKE_PROJECT_NAME} ${PROJECT_NAME} ${PROJECT_NAMESPACE}:: ${CMAKE_PROJECT_VERSION})

# gtest 单元测试开关
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
    *.h*
    */tests/*
    */common/*
  )
  add_subdirectory(tests)
endif()

# message
message("")
message("┌──────── PNC world model Options ───────────────────────────────────────────")
message("│ Project:               ${CMAKE_PROJECT_NAME}")
message("│ Version:               ${CMAKE_PROJECT_VERSION}")
message("│ Time stamp:            ${PNC_SOLVER_VERSION_TIMESTAMP}")
message("│ Git commit-id:         ${PNC_SOLVER_VERSION_COMMIT_ID}")
message("│ Install prefix:        ${CMAKE_INSTALL_PREFIX}")
message("│ Compiler:              ${CMAKE_CXX_COMPILER}")
message("│ CMAKE_BUILD_TYPE:      ${CMAKE_BUILD_TYPE}")
message("└────────────────────────────────────────────────────────────────────────────")
message("")
