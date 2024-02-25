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