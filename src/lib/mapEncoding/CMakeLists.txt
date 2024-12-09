message(DEBUG "./src/lib/mapEncoding/cmake")

if(DEFINED ENV{UVGVPCC_KVAZAAR_REPO_URL} AND NOT "$ENV{UVGVPCC_KVAZAAR_REPO_URL}" STREQUAL "")
    message(WARNING "Kvazaar repo overwritten by environment variable UVGVPCC_KVAZAAR_REPO_URL")   
    set(KVAZAAR_REPO_URL $ENV{UVGVPCC_KVAZAAR_REPO_URL})
    if(DEFINED ENV{UVGVPCC_KVAZAAR_REPO_TAG} AND NOT "$ENV{UVGVPCC_KVAZAAR_REPO_TAG}" STREQUAL "")
        message(WARNING "Kvazaar tag overwritten by environment variable UVGVPCC_KVAZAAR_REPO_TAGS")
        set(KVAZAAR_REPO_TAGS $ENV{UVGVPCC_KVAZAAR_REPO_TAG})
    endif()
endif()

message(STATUS "Kvazaar git repository: ${KVAZAAR_REPO_URL}")
message(STATUS "Kvazaar tags: ${KVAZAAR_REPO_TAGS}")
message(STATUS "Kvazaar EP_UPDATE_DISCONNECTED: ${KVAZAAR_EP_UPDATE_DISCONNECTED}")

if(DEFINED ENV{UVGVPCC_UVG266_REPO_URL} AND NOT "$ENV{UVGVPCC_UVG266_REPO_URL}" STREQUAL "")
    message(WARNING "uvg266 repo overwritten by environment variable UVGVPCC_UVG266_REPO_URL")   
    set(UVG266_REPO_URL $ENV{UVGVPCC_UVG266_REPO_URL})
    if(DEFINED ENV{UVGVPCC_UVG266_REPO_TAG} AND NOT "$ENV{UVGVPCC_UVG266_REPO_TAG}" STREQUAL "")
        message(WARNING "uvg266 tag overwritten by environment variable UVGVPCC_UVG266_REPO_TAGS")
        set(UVG266_REPO_TAGS $ENV{UVGVPCC_UVG266_REPO_TAG})
    endif()
endif()

message(STATUS "uvg266 git repository: ${UVG266_REPO_URL}")
message(STATUS "uvg266 tags: ${UVG266_REPO_TAGS}")
message(STATUS "uvg266 EP_UPDATE_DISCONNECTED: ${UVG266_EP_UPDATE_DISCONNECTED}")

# The following code is used to generate a custom header file that will be used to define the prefix for the standard I/O functions
find_file(STDIO_INCLUDE_PATH stdio.h PATHS /usr/include /usr/local/include)

if(STDIO_INCLUDE_PATH)
    message(STATUS "Found stdio.h: ${STDIO_INCLUDE_PATH}")
else()
    message(FATAL_ERROR "Could not find stdio.h")
endif()

set(STDIO_PREFIX "kvazaar")
set(KVAZAAR_STDIO_LIB ${STDIO_PREFIX}_stdio)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/uvg_stdio.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/${STDIO_PREFIX}_stdio.h @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/uvg_stdio.c.in ${CMAKE_CURRENT_BINARY_DIR}/${STDIO_PREFIX}_stdio.c @ONLY)

add_library(${KVAZAAR_STDIO_LIB} STATIC
    ${STDIO_PREFIX}_stdio.c
)

# Add kvazaar as an external project fetched from Git
ExternalProject_Add(
    kvazaar
    PREFIX ${CMAKE_BINARY_DIR}/external
    GIT_REPOSITORY ${KVAZAAR_REPO_URL}
    GIT_TAG ${KVAZAAR_REPO_TAGS}
    BUILD_COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/include/${STDIO_PREFIX}_stdio.h ${CMAKE_BINARY_DIR}/external/src/kvazaar/src/stdio.h
                  COMMAND ${CMAKE_COMMAND} --build . --target kvazaar
    INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/include  # Create directory for headers
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/external/src/kvazaar/src/kvazaar.h ${CMAKE_CURRENT_BINARY_DIR}/include/  # Copy main header file
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/external/src/kvazaar-build/libkvazaar.a ${CMAKE_CURRENT_BINARY_DIR}/  # Copy static library file
    BUILD_BYPRODUCTS libkvazaar.a
    UPDATE_DISCONNECTED ${KVAZAAR_EP_UPDATE_DISCONNECTED}
    CMAKE_ARGS
        -DBUILD_SHARED_LIBS=OFF
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON  # Force kvazaar to be built with -fPIC
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    LOG_DOWNLOAD ON
    LOG_UPDATE ON
    LOG_PATCH ON
    LOG_CONFIGURE ON
    LOG_BUILD ON
    LOG_INSTALL ON
    LOG_TEST ON
    LOG_MERGED_STDOUTERR ON
    LOG_OUTPUT_ON_FAILURE ON
    VERBATIM 
)
add_dependencies(kvazaar ${KVAZAAR_STDIO_LIB})


# The following code is used to generate a custom header file that will be used to define the prefix for the standard I/O functions
set(STDIO_PREFIX "uvg266")
set(UVG266_STDIO_LIB ${STDIO_PREFIX}_stdio)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/uvg_stdio.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/${STDIO_PREFIX}_stdio.h @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/uvg_stdio.c.in ${CMAKE_CURRENT_BINARY_DIR}/${STDIO_PREFIX}_stdio.c @ONLY)

add_library(${UVG266_STDIO_LIB} STATIC
    ${STDIO_PREFIX}_stdio.c
)

# TODO(lf): Fix: uvg266 seems compile uvg-bin with ninja
# The workaround below should be solved at some point
if (CMAKE_GENERATOR STREQUAL "Ninja")
    set(UVG266_TARGET "libuvg266.a")
else()
    set(UVG266_TARGET "uvg266")
endif()

# Add uvg266 as an external project fetched from Git
ExternalProject_Add(
    uvg266
    PREFIX ${CMAKE_BINARY_DIR}/external
    GIT_REPOSITORY ${UVG266_REPO_URL}
    GIT_TAG ${UVG266_REPO_TAGS}
    BUILD_COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/include/${STDIO_PREFIX}_stdio.h ${CMAKE_BINARY_DIR}/external/src/uvg266/src/stdio.h
                  COMMAND ${CMAKE_COMMAND} --build . --target ${UVG266_TARGET}
    INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/include  # Create directory for headers
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/external/src/uvg266/src/uvg266.h ${CMAKE_CURRENT_BINARY_DIR}/include/  # Copy main header file
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/external/src/uvg266-build/libuvg266.a ${CMAKE_CURRENT_BINARY_DIR}/  # Copy static library file
    BUILD_BYPRODUCTS libuvg266.a
    UPDATE_DISCONNECTED ${UVG266_EP_UPDATE_DISCONNECTED}
    CMAKE_ARGS
        -DBUILD_SHARED_LIBS=OFF
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON  # Force uvg266 to be built with -fPIC
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    LOG_DOWNLOAD ON
    LOG_UPDATE ON
    LOG_PATCH ON
    LOG_CONFIGURE ON
    LOG_BUILD ON
    LOG_INSTALL ON
    LOG_TEST ON
    LOG_MERGED_STDOUTERR ON
    LOG_OUTPUT_ON_FAILURE ON
    VERBATIM
)

add_dependencies(uvg266 ${UVG266_STDIO_LIB})

# Define the library
add_library(catchLibLogLibrary STATIC
    catchLibLog.cpp
)

set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_DEFAULT}")

# Define the library
add_library(mapEncodingLibrary STATIC
    encoderKvazaar.cpp
    mapEncoding.cpp
    catchLibLog.cpp
)

# Include directories for headers
target_include_directories(mapEncodingLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/include)

# Optionally, add additional configurations, dependencies, or options
add_dependencies(mapEncodingLibrary uvg266 kvazaar)
target_compile_options(mapEncodingLibrary PRIVATE -fPIC)
target_link_libraries(mapEncodingLibrary PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/libkvazaar.a ${CMAKE_CURRENT_BINARY_DIR}/libuvg266.a ${KVAZAAR_STDIO_LIB} ${UVG266_STDIO_LIB})

# TODO(lf): following line might be useless (#gg)
target_compile_options(catchLibLogLibrary PRIVATE -fPIC)
