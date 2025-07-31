# Add the external utility
if(CHECK_PC_MD5)
    
ExternalProject_Add(
    tmc2
    PREFIX ${CMAKE_BINARY_DIR}/external/tmc2
    GIT_REPOSITORY https://github.com/MPEGGroup/mpeg-pcc-tmc2/
    GIT_TAG release-v24.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/external/tmc2/install
    UPDATE_DISCONNECTED ${TMC2_EP_UPDATE_DISCONNECTED}
    LOG_DOWNLOAD ON
    LOG_UPDATE ON
    LOG_PATCH ON
    LOG_CONFIGURE ON
    LOG_BUILD ON
    LOG_INSTALL ON
    LOG_TEST ON
    LOG_MERGED_STDOUTERR ON
    LOG_OUTPUT_ON_FAILURE ON
)

ExternalProject_Add(
    pccsum
    PREFIX ${CMAKE_BINARY_DIR}/external/pccsum
    GIT_REPOSITORY git@gitlab.tuni.fi:cs/ultravideo/pcc/utils/pccsum.git
    GIT_TAG main
    INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/external/pccsum/install
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/external/pccsum/src/pccsum-build/src/app/appPccsum ${CMAKE_BINARY_DIR}/external/pccsum/install
    UPDATE_DISCONNECTED ${PCCSUM_EP_UPDATE_DISCONNECTED}
    LOG_DOWNLOAD ON
    LOG_UPDATE ON
    LOG_PATCH ON
    LOG_CONFIGURE ON
    LOG_BUILD ON
    LOG_INSTALL ON
    LOG_TEST ON
    LOG_MERGED_STDOUTERR ON
    LOG_OUTPUT_ON_FAILURE ON
)
endif()

function(get_ref_md5 ref_file test_name md5_bitstream md5_pc)
    file(READ ${ref_file} csv_content)
    string(REPLACE "\n" ";" csv_lines ${csv_content}) # Split rows into list

    foreach(line IN LISTS csv_lines)
        string(REPLACE "," ";" fields ${line})
        list(GET fields 0 current_test_name)
        if(current_test_name STREQUAL ${test_name})

            list(GET fields 1 extracted_md5_bitstream)
            list(GET fields 2 extracted_md5_pc)

            set(${md5_bitstream} ${extracted_md5_bitstream} PARENT_SCOPE)
            set(${md5_pc} ${extracted_md5_pc} PARENT_SCOPE)
            return()
        endif()
    endforeach()
    set(${md5_bitstream} "" PARENT_SCOPE)
    set(${md5_pc} "" PARENT_SCOPE)
    message(WARNING "Test name '${test_name}' not found in ${ref_file}.")
endfunction()

macro(add_test_macro test_name test_input test_nbframe test_startFrame test_nbthread test_nbloop test_uvgvpccencParam test_timeout)
    get_ref_md5(${CMAKE_SOURCE_DIR}/tests/ref_md5.csv ${test_name} md5_bitstream md5_pc)

    add_test(NAME ${test_name}_encoding
    COMMAND uvgVPCCenc
    -i ${test_input}
    -n ${test_nbframe}
    -s ${test_startFrame}
    -o ${TEST_OUTPUT_BITSTREAM_DIR}/${test_name}.vpcc
    -t ${test_nbthread}
    -l ${test_nbloop}
    --uvgvpcc ${test_uvgvpccencParam}
    )
    set_tests_properties(${test_name}_encoding PROPERTIES TIMEOUT ${test_timeout})

    if(md5_bitstream STREQUAL "")
        message(FATAL_ERROR "Error: ${test_name} md5_bitstream is empty. Cannot proceed with the test.")
    endif()

    add_test(NAME ${test_name}_md5_bitstream
        COMMAND ${CMAKE_COMMAND} -E md5sum ${TEST_OUTPUT_BITSTREAM_DIR}/${test_name}.vpcc
    )  
    set_tests_properties(${test_name}_md5_bitstream
        PROPERTIES PASS_REGULAR_EXPRESSION "^${md5_bitstream}"
    )
    if(CHECK_PC_MD5)    
        if(md5_pc STREQUAL "")
            message(FATAL_ERROR "Error: ${test_name} md5_pc is empty. Cannot proceed with the test.")
        endif()
        add_test(NAME ${test_name}_decoding
            COMMAND echo decode
        )
        set_tests_properties(${test_name}_encoding PROPERTIES TIMEOUT ${test_timeout})

        add_test(NAME ${test_name}_md5_pc
            COMMAND echo pccsum
        )
        set_tests_properties(${test_name}_md5_pc
            PROPERTIES PASS_REGULAR_EXPRESSION "^${md5_pc}"
        )
    endif(CHECK_PC_MD5)

endmacro()



if(ENABLE_CI_TESTING)
    include(${CMAKE_SOURCE_DIR}/tests/ci_test.cmake)
endif()

if(ENABLE_MY_TESTING)
    if(DEFINED MY_TEST_PATH AND NOT "${MY_TEST_PATH}" STREQUAL "")
        if(NOT EXISTS ${MY_TEST_PATH})
            message(WARNING "Personal test file not found at ${MY_TEST_PATH}.")
        else()
            message(WARNING "Personal test file found at ${MY_TEST_PATH}.")
            include(${MY_TEST_PATH})
        endif()
    else()
        message(WARNING
        "MY_TEST_PATH path is either not defined or empty.\n"
        "To create your own test, copy, rename the template, and define the MY_TEST_PATH variable.\n"
        "Template can be found at ${CMAKE_SOURCE_DIR}/tests/template.cmake."
        )
    endif()
endif()