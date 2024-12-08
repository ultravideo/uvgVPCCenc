message(DEBUG "./src/lib/utils/cmake")

set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_DEFAULT}")

# Define the library
add_library(utilsLibrary STATIC
    threadqueue.cpp
    preset.cpp
    parameters.cpp
)

add_library(logLibrary STATIC
    log.cpp
)

# # Optionally, add additional configurations, dependencies, or options
target_compile_options(utilsLibrary PRIVATE -fPIC)
target_compile_options(logLibrary PRIVATE -fPIC)
