# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

add_custom_target(
        frontend
        # add final and temporary build directory for the frontend. That way npm does not need to know the CMAKE_BINARY_DIR which can variable
        COMMAND cmake -E make_directory "${CMAKE_BINARY_DIR}/frontend"
        COMMAND cmake -E make_directory "${CMAKE_SOURCE_DIR}/frontend/build"
        # Run frontend npm build process
        COMMAND npm install && npm run build
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/frontend")
# Copy the content of npm to the build directory
add_custom_command(
        TARGET frontend POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/frontend/build"
        "${CMAKE_BINARY_DIR}/frontend")
# delete npm temp build folder
add_custom_command (
        TARGET frontend
        POST_BUILD
        COMMAND cmake -E remove_directory "${CMAKE_SOURCE_DIR}/frontend/build"
)
set(PROJECT_VER "2.0 stable")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(db_esp32)