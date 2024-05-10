# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

add_custom_target(
  frontend
  COMMAND npm install && npm run build
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/frontend"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(db_esp32)
