message(STATUS "Generate and include v3crtp tests from generate_v3crtp_tests.cmake")

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

message(STATUS "Defining tests in generate_v3crtp_tests.cmake")

# Test configurations
set(TEST_CONFIGURATIONS default slicing efficientMapGen)

set(TEST_SEQ_DIR "${CMAKE_SOURCE_DIR}/_sequences/VPCC")
set(TIMEOUT 300)

set(IPS "127.0.0.1")
set(PORTS "8890" "8890,8891,8892,8893" "8890,8891,8892,8893,8894")
set(SDP_DIRS "-" ${TEST_OUTPUT_BITSTREAM_DIR})  

# Loop over sequences, modes, voxels, presets, and rates
foreach(IP IN LISTS IPS)
	foreach(PORT IN LISTS PORTS)
		foreach(SDP IN LISTS SDP_DIRS) 
			if(SDP STREQUAL "-" AND PORT STREQUAL "8890,8891,8892,8893")
				continue()
			endif()
			if(NOT SDP STREQUAL "-" AND PORT STREQUAL "8890,8891,8892,8893,8894")
				continue()
			endif()
			foreach(testConfig IN LISTS TEST_CONFIGURATIONS)
				add_v3crtp_test_wrapper(v3crtp ${testConfig} ${TEST_SEQ_DIR} ReadyForWinter vox9 AI 32-42-4 fast true 20 1 18 ${IP} ${PORT} ${SDP} ${TIMEOUT})
				add_v3crtp_test_wrapper(v3crtp ${testConfig} ${TEST_SEQ_DIR} ReadyForWinter vox9 RA 16-22-2 fast true 20 1 2 ${IP} ${PORT} ${SDP} ${TIMEOUT})
				add_v3crtp_test_wrapper(v3crtp ${testConfig} ${TEST_SEQ_DIR} ReadyForWinter vox9 AI 32-42-4 slow false 20 1 2 ${IP} ${PORT} ${SDP} ${TIMEOUT})
				add_v3crtp_test_wrapper(v3crtp ${testConfig} ${TEST_SEQ_DIR} FlowerWave vox10 AI 32-42-4 fast false 20 1 2 ${IP} ${PORT} ${SDP} ${TIMEOUT})
			endforeach()
		endforeach()
	endforeach()
endforeach()
