project(reference_line_info)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)
target_link_libraries(${PROJECT_NAME} INTERFACE reference_line path_data)

if(BUILD_TESTING)
  target_compile_options(${PROJECT_NAME} PRIVATE -fprofile-arcs -ftest-coverage)
endif()

install(DIRECTORY include/ DESTINATION include/${CMAKE_PROJECT_NAME})
