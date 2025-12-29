message(DEBUG "./src/lib/patchGeneration/cmake")

add_library(kdLibrary STATIC
    kdTree.cpp
)

set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_DEFAULT}")

# Define the library
add_library(patchGenerationLibrary STATIC
    patchGeneration.cpp
    normalComputation.cpp
    normalOrientation.cpp
    ppiSegmenter.cpp
    patchSegmentation.cpp
    slicingComputation.cpp
)

# Include directories for headers
# target_include_directories(patchGenerationLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Optionally, add additional configurations, dependencies, or options
target_compile_options(kdLibrary PRIVATE -fPIC)
target_compile_options(patchGenerationLibrary PRIVATE -fPIC)