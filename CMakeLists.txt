cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(HA_on_off_light)

# Inclui o diretório main como um subdiretório
add_subdirectory(main)

# Inclui o diretório managed_components como um subdiretório
add_subdirectory(managed_components)

set(CMAKE_C_COMPILER "path/to/your/esp32/compiler/gcc")
set(CMAKE_CXX_COMPILER "path/to/your/esp32/compiler/g++")
