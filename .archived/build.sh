# build everything
cmake -S all -B build
cmake --build build

# view changes
cmake --build build --target format

# format code
cmake --build build --target fix-format