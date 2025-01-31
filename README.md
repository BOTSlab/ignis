# ignis
A collection of experiments involving swarms of robots manipulating pucks.  The core simulator (Sim.hpp) models the physics of 2-d interactions between circular bodies.  

# Dependencies

[vcpkg](https://vcpkg.io/en/) is used as the package manager to install most (but not all) dependencies.  vcpkg itself is part of this repo as a git submodule.

While using vcpkg does much of the work required to install external packages there are some build tools needed on your machine.  The following (at least) must be installed:

  autoconf, cmake, libtool

The following is the procedure for adding packages using vcpkg.  Actually, first you should make sure the [package exists](https://vcpkg.io/en/packages).  This is how imgui was added:
```
    ./vcpkg/vcpkg add port imgui
    ./vcpkg/vcpkg install

    # Modify CMakeLists.txt as suggested
```
This last step is important and is not automated.  Some guesswork is required to figure out exactly what to incoroporate into CMakeLists.txt.

# Build instructions

The following two lines create the build subdirectory, building all executables
within it:
```
    cmake --preset=default
    cmake --build build
```

# Executables

```./build/forage``` GUI-based application to demonstrate swarm-based foraging approach described in paper submitted to ANTS 2024.  Loads parameters from ```last_parameters.dat```.

```./build/vorlife``` Combines the controller above with the idea of separating robots into Voronoi cells.  This is an idea with some merit, but the implementation here is crude and collisions between robots are still quite possible.

```./build/vorlife_headless``` Runs headless to generate experimental data for ```vorlife```

```./build/optimize``` Uses [Pagmo2's Generational Particle Swarm Optimization](https://esa.github.io/pagmo2/docs/cpp/algorithms/pso_gen.html) algorithm to find parameters for ```forage```.

```./build/ignis``` A somewhat failed experiment to employ a planning-based approach to planar construction.

# Private Notes

- This is how vcpkg itself was added as a submodule:

```
    git submodule add https://github.com/microsoft/vcpkg
    ./vcpkg/bootstrap-vcpkg.sh
```

- Using the following advice on switching between Debug/Release builds:

https://stackoverflow.com/questions/7724569/debug-vs-release-in-cmake/64719718#64719718

I succeeded (I think) in creating a debug build with the following sequence of commands

```
rm -fr build
cmake --preset=default
cmake -S . -B build/ -D CMAKE_BUILD_TYPE=Debug
cmake --build build
```
