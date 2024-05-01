cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")

# Quiets a warning about the minimum macOS version
set(CMAKE_CXX_FLAGS "-mmacosx-version-min=14.4")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

project(ignis)

# All of these packages are installed via vcpkg.
find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GEOS CONFIG REQUIRED)
find_package(Pagmo REQUIRED)

add_executable(ignis_headless ignis_headless.cpp)
target_link_libraries(ignis_headless PRIVATE GEOS::geos)

add_executable(ignis ignis.cpp)
target_link_libraries(ignis PRIVATE imgui::imgui implot::implot OpenGL::GL)
target_include_directories(ignis PRIVATE imgui::imgui)
target_link_libraries(ignis PRIVATE GEOS::geos)

add_executable(alife alife.cpp)
target_link_libraries(alife PRIVATE imgui::imgui implot::implot OpenGL::GL)
target_include_directories(alife PRIVATE imgui::imgui)

add_executable(vorlife vorlife.cpp)
target_link_libraries(vorlife PRIVATE imgui::imgui implot::implot OpenGL::GL)
target_include_directories(vorlife PRIVATE imgui::imgui)
target_link_libraries(vorlife PRIVATE GEOS::geos)

add_executable(vorlife_headless vorlife_headless.cpp)
target_link_libraries(vorlife_headless PRIVATE GEOS::geos)

add_executable(optimize optimize.cpp)
target_link_libraries(optimize PRIVATE Pagmo::pagmo)