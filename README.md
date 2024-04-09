# ignis
A light-based guidance system for a multi-robot system operating on a screen

# Private Notes

- Added vcpkg as a submodule

```
    git submodule add https://github.com/microsoft/vcpkg
    ./vcpkg/bootstrap-vcpkg.sh
```
- Procedure for adding packages using vcpkg.  This is how imgui was added:
```
    ./vcpkg/vcpkg add port imgui
    ./vcpkg/vcpkg install

    # Modify CMakeList.txt as suggested
```

- The following two lines build the application.
```
    cmake --preset=default
    cmake --build build
```

- While using vcpkg does much of the work required to install external packages
  there are some build tools needed on your machine.  The following (at least)
  must be installed:

  autoconf, cmake, libtool

- Using the following advice on switching between Debug/Release builds:

https://stackoverflow.com/questions/7724569/debug-vs-release-in-cmake/64719718#64719718

I succeeded (I think) in creating a debug build with the following sequence of commands

```
rm -fr build
cmake --preset=default
cmake -S . -B build/ -D CMAKE_BUILD_TYPE=Debug
cmake --build build
```

- libcmaes was not available to install with vcpkg, so I installed it on the
system.
