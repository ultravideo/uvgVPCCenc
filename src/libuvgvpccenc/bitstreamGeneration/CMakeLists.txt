message(DEBUG "./src/lib/bitstreamGeneration/cmake")

add_library(bitstreamGeneration STATIC
    bitstream_util.cpp
    vps.cpp
    video_sub_bitstream.cpp
    atlas_context.cpp
    gof.cpp
    bitstreamGeneration.cpp
)

target_link_libraries(bitstreamGeneration
    PUBLIC
        uvgutils
)

target_compile_options(bitstreamGeneration PRIVATE -fPIC)
