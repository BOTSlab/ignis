cmake_minimum_required(VERSION 3.10)
set (CMAKE_CXX_STANDARD 17)

project(ignis)

find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

add_executable(ignis_headless main_headless.cpp)
target_include_directories(ignis_headless PRIVATE ${RAPIDCSV_INCLUDE_DIRS})

add_executable(ignis_imgui main_imgui.cpp)
target_link_libraries(ignis_imgui PRIVATE imgui::imgui implot::implot OpenGL::GL)
target_include_directories(ignis_imgui PRIVATE imgui::imgui)
target_include_directories(ignis_imgui PRIVATE ${RAPIDCSV_INCLUDE_DIRS})