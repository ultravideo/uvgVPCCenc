message(DEBUG "./src/app/cmake")

add_library(miniplyLibrary STATIC
    extras/miniply.cpp
)

add_library(getoptLibrary STATIC
    extras/getopt.c
)

set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_DEFAULT}")

# Define the executable
add_executable(uvgVPCCenc
    uvgVPCCencAppExample.cpp
    cli.cpp
)

target_compile_options(miniplyLibrary PRIVATE -fPIC)
target_compile_options(getoptLibrary PRIVATE -fPIC)

target_link_libraries(uvgVPCCenc uvgvpccenc)

# Define installation destinations
install(TARGETS uvgVPCCenc DESTINATION bin)