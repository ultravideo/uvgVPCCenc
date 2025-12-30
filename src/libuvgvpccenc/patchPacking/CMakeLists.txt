message(DEBUG "./src/lib/patchPacking/cmake")

set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_DEFAULT}")

# Define the library
add_library(patchPackingLibrary STATIC
    patchPacking.cpp
)

target_link_libraries(patchPackingLibrary
    PUBLIC
        uvgutils
)

# Include directories for headers
# target_include_directories(patchPackingLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Optionally, add additional configurations, dependencies, or options
target_compile_options(patchPackingLibrary PRIVATE -fPIC)
