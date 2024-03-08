cmake_minimum_required(VERSION 3.10)
set (CMAKE_CXX_STANDARD 17)

project(ignis)

find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GEOS CONFIG REQUIRED)
find_package(tinysplinecxx CONFIG REQUIRED)

add_executable(ignis_headless main_headless.cpp)
target_link_libraries(ignis_headless PRIVATE GEOS::geos)
target_link_libraries(ignis_headless PRIVATE tinysplinecxx::tinysplinecxx)

add_executable(ignis_imgui main_imgui.cpp)
target_link_libraries(ignis_imgui PRIVATE imgui::imgui implot::implot OpenGL::GL)
target_include_directories(ignis_imgui PRIVATE imgui::imgui)
target_link_libraries(ignis_imgui PRIVATE GEOS::geos)
target_link_libraries(ignis_imgui PRIVATE tinysplinecxx::tinysplinecxx)