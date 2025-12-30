message(DEBUG "./src/lib/cmake")

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(UVG_LOG_LEVEL_PRESET)
  add_compile_definitions(UVG_LOG_LEVEL=LogLevel::${UVG_LOG_LEVEL_PRESET})
endif()

configure_file(version.cpp.in version.cpp)

set(LIB_SOURCES
    version.cpp
    jobManagement.cpp
    log.cpp
    threadqueue.cpp
)

if(BUILD_SHARED_LIBS)
  add_library(uvgutils SHARED ${LIB_SOURCES})
else()
  add_library(uvgutils STATIC ${LIB_SOURCES})
endif()

target_link_libraries(uvgutils

)

# Include directories for headers
target_include_directories(uvgutils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)