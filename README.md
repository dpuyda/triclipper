# TriClipper

TriClipper is a header-only library that implements the Vatti clipping
algorithm.

## Usage

To use TriClipper, simply add
[triclipper.h](include/triclipper/triclipper.h)
to your project.

## How It Works

If you are curious to know how TriClipper works in more detail, see
[How It Works](docs/how_it_works.md).

## Building Examples and Tests

TriClipper comes with examples and tests which can be built using the CMake
script [CMakeLists.txt](CMakeLists.txt).

For example:

1. Create a folder named "build" in the root of the git repository and navigate
to this folder.

    ```
    mkdir build && cd build
    ```

2. Run CMake from this folder using the BUILD_EXAMPLES and BUILD_TESTS options.

    ```
    cmake -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
    ```

### Debugging

To debug TriClipper, enable the debug output using the DEBUG_OUTPUT option.

For example:

```
cmake -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON -DDEBUG_OUTPUT=ON ..
```

## License

TriClipper is licensed under the [MIT License](LICENSE).
