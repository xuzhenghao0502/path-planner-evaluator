project(adas_data)

# file(GLOB_RECURSE _HDRS "include/*.h")
# file(GLOB_RECURSE _SRCS "src/*.cpp")

pnc_add_library(
  ${PROJECT_NAME}
  OBJECT
  DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  INCLUDE_DIR
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  LIBRARY
  PUBLIC
  gpal::pnc_common_util
)

if(BUILD_TESTING)
  target_compile_options(${PROJECT_NAME} PRIVATE -fprofile-arcs -ftest-coverage)
endif()

install(DIRECTORY include/ DESTINATION include/${CMAKE_PROJECT_NAME})
