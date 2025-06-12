message(DEBUG "./src/lib/mapGeneration/cmake")

set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_DEFAULT}")

# Define the library
add_library(mapGenerationLibrary STATIC
    mapGeneration.cpp
)

# Include directories for headers
# target_include_directories(mapGenerationLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Optionally, add additional configurations, dependencies, or options
target_compile_options(mapGenerationLibrary PRIVATE -fPIC)