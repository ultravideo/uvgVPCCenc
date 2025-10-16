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
    # message(WARNING "AAA Here is the MD5 reference file name '${ref_file}'.")
    
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

macro(add_test_macro test_name test_input test_nbframe test_startFrame test_nbthread test_nbloop test_uvgvpccencParam test_timeout ref_md5_file)
    
    get_ref_md5(${ref_md5_file} ${test_name} md5_bitstream md5_pc)

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


    
    add_test(NAME ${test_name}_md5_bitstream
    COMMAND ${CMAKE_COMMAND} -E md5sum ${TEST_OUTPUT_BITSTREAM_DIR}/${test_name}.vpcc
    )



    # Make md5 test depend on encoding
    set_tests_properties(${test_name}_md5_bitstream PROPERTIES DEPENDS ${test_name}_encoding)
    if(md5_bitstream STREQUAL "")
        set_tests_properties(${test_name}_md5_bitstream
            PROPERTIES PASS_REGULAR_EXPRESSION "^The md5 is missing for this test, this string makes that md5 comparison test fail."
        )
    else()
        set_tests_properties(${test_name}_md5_bitstream
            PROPERTIES PASS_REGULAR_EXPRESSION "^${md5_bitstream}"
        )
    endif()

    if(CHECK_PC_MD5)
        if(md5_pc STREQUAL "")
            message(FATAL_ERROR "Error: ${test_name} md5_pc is empty. Cannot proceed with the test.")
        endif()

        add_test(NAME ${test_name}_decoding
            COMMAND echo decode
        )
        set_tests_properties(${test_name}_decoding PROPERTIES TIMEOUT ${test_timeout})
        set_tests_properties(${test_name}_md5_bitstream PROPERTIES DEPENDS ${test_name}_encoding)

        add_test(NAME ${test_name}_md5_pc
            COMMAND echo pccsum
        )
        # Make md5_pc depend on decoding
        set_tests_properties(${test_name}_md5_pc PROPERTIES
            DEPENDS ${test_name}_decoding
            PASS_REGULAR_EXPRESSION "^${md5_pc}"
        )
    endif(CHECK_PC_MD5)
endmacro()




if(NOT DEFINED ADD_TEST_WRAPPER_DEFINED)
    # Wrapper for add_test_macro(...) (avoid any inconsistency between the test parameter and the test name)
    function(add_test_wrapper testSet testConfig testSeqDir seq voxelSize mode rate preset doubleLayer nbThread nbLoop nbFrame timeout ref_md5)
        # build test name
        set(testName "${testSet}_${testConfig}_${seq}_${voxelSize}_${mode}_${rate}_${preset}_${doubleLayer}_${nbThread}t_${nbLoop}l_${nbFrame}n")
        set(startFrame 0)
        set(inputFile "${testSeqDir}/UVG-VPC/${seq}/ply_${voxelSize}/ply_xyz_rgb/${seq}_UVG_${voxelSize}_25_0_250_%04d.ply")
        set(configParam "")
        if(testConfig STREQUAL "slicing")
            set(configParam ",activateSlicing=true")
        endif()
        if(testConfig STREQUAL "efficientMapGen")
            set(configParam ",attributeBgFill=bbpe")
        endif()
        set(uvgVPCCencParam "presetName=${preset},mode=${mode},rate=${rate},doubleLayer=${doubleLayer},logLevel=DEBUG${configParam}")

        # call the add test macro
        add_test_macro(
            ${testName}
            ${inputFile}
            ${nbFrame}
            ${startFrame}
            ${nbThread}
            ${nbLoop}
            ${uvgVPCCencParam}
            ${timeout}
            ${ref_md5}
        )
    endfunction()

    set(ADD_TEST_WRAPPER_DEFINED TRUE)
endif()

if(ENABLE_CI_TESTING)
    add_subdirectory(quick_tests)
    add_subdirectory(long_tests)
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

# TODO(gg): ENABLE_CI_TESTING and ENABLE_MY_TESTING and ENABLE_TESTING are all usefull?

# Comments about the CMakePresets.json :
# TODO(gg): Is RelWithDebInfo preset really used?
# TODO(gg): is it usefull to have a 'CI' preset? Why not to run using 'Release' so that it is fast and simple. And if there is an issue, we can run the ci localy with the 'Debug' preset



