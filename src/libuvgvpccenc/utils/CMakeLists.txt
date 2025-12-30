message(DEBUG "./src/lib/utils/cmake")

set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_DEFAULT}")

# Define the library
add_library(utilsLibrary STATIC
    preset.cpp
    parameters.cpp
    fileExport.cpp
    statsCollector.cpp
)

target_link_libraries(utilsLibrary
    PUBLIC
    uvgutils
)

target_compile_options(utilsLibrary PRIVATE -fPIC)
