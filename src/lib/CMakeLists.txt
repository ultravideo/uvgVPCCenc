message(DEBUG "./src/lib/cmake")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

configure_file(version.cpp.in version.cpp)

add_subdirectory(utils)
add_subdirectory(bitstreamGeneration)
add_subdirectory(patchGeneration)
add_subdirectory(patchPacking)
add_subdirectory(mapEncoding)
add_subdirectory(mapGeneration)

set(LIB_SOURCES
    version.cpp
    uvgvpcc.cpp
)

if(BUILD_SHARED_LIBS)
  add_library(uvgvpcc SHARED ${LIB_SOURCES})
else()
  add_library(uvgvpcc STATIC ${LIB_SOURCES})
endif()

target_link_libraries(uvgvpcc
    bitstreamGeneration
    patchGenerationLibrary
    patchPackingLibrary
    mapEncodingLibrary
    catchLibLogLibrary
    mapGenerationLibrary
    utilsLibrary
    kdLibrary
    logLibrary
    miniplyLibrary
)

# Include directories for headers
target_include_directories(uvgvpcc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Optionally, add additional configurations, dependencies, or options
# target_compile_options(BitstreamGenerationLibrary PRIVATE -std=c++11)

# Define installation destinations
install(TARGETS uvgvpcc DESTINATION lib)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION include)