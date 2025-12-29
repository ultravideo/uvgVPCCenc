message(DEBUG "./src/lib/cmake")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(UVG_LOG_LEVEL_PRESET)
  add_compile_definitions(UVG_LOG_LEVEL=LogLevel::${UVG_LOG_LEVEL_PRESET})
endif()

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
  add_library(uvgvpccenc SHARED ${LIB_SOURCES})
else()
  add_library(uvgvpccenc STATIC ${LIB_SOURCES})
endif()

target_link_libraries(uvgvpccenc
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
target_include_directories(uvgvpccenc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)