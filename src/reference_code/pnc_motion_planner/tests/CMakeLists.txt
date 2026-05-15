cmake_minimum_required(VERSION 3.14)

project(${CMAKE_PROJECT_NAME}_tests)

message(STATUS "Building GTest Projects for x86_64 platform")
find_package(GTest REQUIRED)
include(GoogleTest)

file(
  GLOB SUBFILES
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/*"
)
set(SUBDIRECTORIES "")
foreach(item IN LISTS SUBFILES)
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${item}")
    list(APPEND SUBDIRECTORIES "${item}")
  endif()
endforeach()
message(STATUS "${CMAKE_PROJECT_NAME}'s SUBDIRECTORIES: ${SUBDIRECTORIES}")

foreach(SUBDIR ${SUBDIRECTORIES})
  file(GLOB_RECURSE TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/*.cpp")
  if(TEST_SOURCES)
    set(NODE_NAME ${SUBDIR}_test)
    add_executable(${NODE_NAME} ${TEST_SOURCES})
    target_link_libraries(
      ${NODE_NAME}
      PUBLIC ${CMAKE_PROJECT_NAME}
             GTest::GTest
             GTest::Main
             gpal::world_model
             gpal::gpal-interface
             gpal::pnc_common_util
             gpal::pnc_common_interface
             gpal::pnc_solver
             eka::rt
             gcov
    )
    target_include_directories(
      ${NODE_NAME} 
      PRIVATE $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}> 
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../${SUBDIR}/include>
              # ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/${CMAKE_PROJECT_NAME}
    )
    target_compile_options(${NODE_NAME} PRIVATE -fprofile-arcs -ftest-coverage)
    target_compile_options(${SUBDIR} PRIVATE -fprofile-arcs -ftest-coverage)
    add_test(${NODE_NAME} ${NODE_NAME})
  endif()
endforeach()

install(TARGETS ${CMAKE_PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

