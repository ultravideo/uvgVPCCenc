message(STATUS "Generate and include quick tests from generate_quick_tests.cmake")

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

message(STATUS "Defining tests in generate_quick_tests.cmake")

# Test configurations
set(TEST_CONFIGURATIONS default slicing efficientMapGen)

set(REF_MD5_FILE "${CMAKE_SOURCE_DIR}/tests/quick_tests/ref_md5_quick_tests.csv")
set(TEST_SEQ_DIR "${CMAKE_SOURCE_DIR}/_sequences/VPCC")
set(TIMEOUT 300)

# Loop over sequences, modes, voxels, presets, and rates
foreach(testConfig IN LISTS TEST_CONFIGURATIONS)
    add_test_wrapper(quick ${testConfig} ${TEST_SEQ_DIR} ReadyForWinter vox9 AI 32-42-4 fast true 20 1 18 ${TIMEOUT} ${REF_MD5_FILE})
    add_test_wrapper(quick ${testConfig} ${TEST_SEQ_DIR} ReadyForWinter vox9 RA 16-22-2 fast true 20 1 2 ${TIMEOUT} ${REF_MD5_FILE})
    add_test_wrapper(quick ${testConfig} ${TEST_SEQ_DIR} ReadyForWinter vox9 AI 32-42-4 slow false 20 1 2 ${TIMEOUT} ${REF_MD5_FILE})
    add_test_wrapper(quick ${testConfig} ${TEST_SEQ_DIR} FlowerWave vox10 AI 32-42-4 fast false 20 1 2 ${TIMEOUT} ${REF_MD5_FILE})
endforeach()
