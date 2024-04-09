cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")

# The following is neccessary for OpenMP (needed by libcmaes)
set(CMAKE_C_COMPILER "/opt/homebrew/opt/llvm/bin/clang")
set(CMAKE_CXX_COMPILER "/opt/homebrew/opt/llvm/bin/clang")

# Quiets a warning about the minimum macOS version
set(CMAKE_CXX_FLAGS "-mmacosx-version-min=14.4")

project(ignis)

# All of these packages are installed via vcpkg.
find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
#find_package(GEOS CONFIG REQUIRED)
find_package(Pagmo REQUIRED)

# libcmaes is installed externally.
find_package(PkgConfig REQUIRED)
pkg_check_modules(libcmaes REQUIRED IMPORTED_TARGET libcmaes)

#add_executable(ignis_headless ignis_headless.cpp)
#target_link_libraries(ignis_headless PRIVATE GEOS::geos)

#add_executable(ignis ignis.cpp)
#target_link_libraries(ignis PRIVATE imgui::imgui implot::implot OpenGL::GL)
#target_include_directories(ignis PRIVATE imgui::imgui)
#target_link_libraries(ignis PRIVATE GEOS::geos)

add_executable(alife alife.cpp)
target_link_libraries(alife PRIVATE imgui::imgui implot::implot OpenGL::GL)
target_include_directories(alife PRIVATE imgui::imgui)

add_executable(pagmo_optimize pagmo_optimize.cpp)
target_link_libraries(pagmo_optimize PRIVATE Pagmo::pagmo)

add_executable(optimize optimize.cpp)
target_link_libraries(optimize PRIVATE PkgConfig::libcmaes)
target_include_directories(optimize PRIVATE PkgConfig::libcmaes)
target_link_directories(optimize PRIVATE /opt/homebrew/opt/libomp/lib)
target_include_directories(optimize PRIVATE /opt/homebrew/opt/libomp/include)
