message(STATUS "Generate and include long tests from generate_long_tests.cmake")

# Define the URL and output paths
set(TEST_FILE_URL "https://ultravideo.fi/UVG-VPC/CI_VPCC.zip")
set(TEST_FILE_ARCHIVE "${CMAKE_SOURCE_DIR}/_sequences/archives/VPCC.zip")
set(TEST_SEQ_DIR "${CMAKE_SOURCE_DIR}/_sequences/VPCC")
set(TEST_OUTPUT_BITSTREAM_DIR "${CMAKE_BINARY_DIR}/bitstream")

# Download the file if it doesn't exist already
if(NOT EXISTS ${TEST_FILE_ARCHIVE})
    message(STATUS "Downloading test file...")
    file(DOWNLOAD ${TEST_FILE_URL} ${TEST_FILE_ARCHIVE})
endif()

# Extract the archive if it hasn't been extracted yet
if(NOT EXISTS ${TEST_SEQ_DIR})
    message(STATUS "Extracting test archive...")
    file(MAKE_DIRECTORY ${TEST_SEQ_DIR})
    file(ARCHIVE_EXTRACT INPUT ${TEST_FILE_ARCHIVE} DESTINATION ${TEST_SEQ_DIR})
endif()

file(MAKE_DIRECTORY ${TEST_OUTPUT_BITSTREAM_DIR})

message(STATUS "Defining tests in generate_long_tests.cmake")

# Define arrays for the different parameters
set(SEQUENCES ReadyForWinter FlowerWave Gymnast CasualSquat BlueBackpack)
set(MODES RA AI)
set(RATES "16-22-2" "32-42-4")
set(PRESETS fast slow)
set(DOUBLE_LAYER true false)

# Define per-sequence frame lists (this acts as your map)
set(FRAMES_ReadyForWinter 1 20)
set(FRAMES_FlowerWave 1 10)
set(FRAMES_Gymnast 1 18)
set(FRAMES_CasualSquat 2)
set(FRAMES_BlueBackpack 2)

set(VOXELS_ReadyForWinter vox9)
set(VOXELS_FlowerWave vox10)
set(VOXELS_Gymnast vox10)
set(VOXELS_CasualSquat vox11)
set(VOXELS_BlueBackpack vox11)

# Test configurations
set(TEST_CONFIGURATIONS default slicing efficientMapGen)

set(THREADS_LIST 4)
set(LOOPS_LIST 1)
set(TIMEOUT 200)
set(REF_MD5_FILE "${CMAKE_SOURCE_DIR}/tests/long_tests/ref_md5_long_tests.csv")

# Base path for sequences
set(TEST_SEQ_DIR "${CMAKE_SOURCE_DIR}/_sequences/VPCC")

# Loop over sequences, modes, voxels, presets, and rates
foreach(testConfig IN LISTS TEST_CONFIGURATIONS)
    foreach(seq IN LISTS SEQUENCES)
        set(voxelSize_list_var "VOXELS_${seq}")
        set(vox ${${voxelSize_list_var}})
        foreach(mode IN LISTS MODES)
            foreach(rate IN LISTS RATES)
                foreach(preset IN LISTS PRESETS)
                    foreach(doubleLayer IN LISTS DOUBLE_LAYER)
                        foreach(nbThread IN LISTS THREADS_LIST)
                            foreach(nbLoop IN LISTS LOOPS_LIST)
                                set(frame_list_var "FRAMES_${seq}")
                                set(nb_frame_list ${${frame_list_var}})
                                foreach(nbFrame IN LISTS nb_frame_list)
                                    # Filters to reduce the total number of tests
                                    if(doubleLayer STREQUAL "false" AND NOT seq STREQUAL "ReadyForWinter")
                                        continue() # Test single layer only with one sequence
                                    endif()

                                    if(preset STREQUAL "slow" AND NOT(seq STREQUAL "FlowerWave" OR seq STREQUAL "ReadyForWinter"))
                                        continue() # Test slow preset only with one sequence
                                    endif()

                                    add_test_wrapper(long ${testConfig} ${TEST_SEQ_DIR} ${seq} ${vox} ${mode} ${rate} ${preset} ${doubleLayer} ${nbThread} ${nbLoop} ${nbFrame} ${TIMEOUT} ${REF_MD5_FILE})
                                endforeach()
                            endforeach()
                        endforeach()
                    endforeach()
                endforeach()
            endforeach()
        endforeach()
    endforeach()
endforeach()
