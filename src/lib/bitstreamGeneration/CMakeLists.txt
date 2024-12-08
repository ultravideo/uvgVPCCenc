message(DEBUG "./src/lib/bitstreamGeneration/cmake")

# TODO to be removed when we our our lib
# if(MSVC)
# add_compile_options(/W3 /WX)
# else()
# set(CMAKE_C_COMPILER gcc)
# set(CMAKE_CXX_COMPILER g++)
# add_compile_options(-W -Wall)
# endif()

add_library(bitstreamGeneration STATIC
    bitstream_util.cpp
    vps.cpp
    video_sub_bitstream.cpp
    atlas_context.cpp
    gof.cpp
    bitstreamGeneration.cpp
)



# Include directories for headers
# target_include_directories(bitstreamGenerationLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# # Optionally, add additional configurations, dependencies, or options
# target_compile_options(bitstreamGenerationLibrary PRIVATE -fPIC)
target_compile_options(bitstreamGeneration PRIVATE -fPIC)