cmake_minimum_required(VERSION 3.28)
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
find_package(Pagmo REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

# OpenMP requires some further work...
set(OPENMP_LIBRARIES "/opt/homebrew/opt/libomp/lib")
set(OPENMP_INCLUDES "/opt/homebrew/opt/libomp/include")
#    set(CMAKE_C_COMPILER clang)
#set(CMAKE_CXX_COMPILER clang++)
#if(CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp")# -L${OPENMP_LIBRARIES} -I${OPENMP_INCLUDES}")# -lomp")
set(OpenMP_C_LIB_NAMES "omp")
set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")# -L${OPENMP_LIBRARIES} -I${OPENMP_INCLUDES}")# -lomp")
set(OpenMP_CXX_LIB_NAMES "omp")
set(OpenMP_omp_LIBRARY omp)
#endif()
include_directories("${OPENMP_INCLUDES}")
link_directories("${OPENMP_LIBRARIES}")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(OpenMP REQUIRED)

# Required for map_elites
find_package(TBB CONFIG REQUIRED)

add_executable(demo demo.cpp)
target_link_libraries(demo PRIVATE yaml-cpp::yaml-cpp)
target_link_libraries(demo PRIVATE imgui::imgui implot::implot OpenGL::GL)
target_include_directories(demo PRIVATE imgui::imgui)

add_executable(optimize optimize.cpp)
target_link_libraries(optimize PRIVATE yaml-cpp::yaml-cpp)
target_link_libraries(optimize PRIVATE Pagmo::pagmo)
target_link_libraries(optimize PRIVATE OpenMP::OpenMP_CXX)
#if(OpenMP_CXX_FOUND)
#endif()

add_executable(map_elites map_elites.cpp)
target_link_libraries(map_elites PRIVATE yaml-cpp::yaml-cpp)
target_link_libraries(map_elites PRIVATE TBB::tbb TBB::tbbmalloc)
target_link_libraries(map_elites PRIVATE OpenMP::OpenMP_CXX)

add_executable(compare_controllers compare_controllers.cpp)
target_link_libraries(compare_controllers PRIVATE yaml-cpp::yaml-cpp)
target_link_libraries(compare_controllers PRIVATE OpenMP::OpenMP_CXX)