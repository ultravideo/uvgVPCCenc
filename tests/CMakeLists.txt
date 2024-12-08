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
# Get the number of tests
get_property(test_count GLOBAL PROPERTY TESTS)

# Check and print the number of tests
if(test_count)
    message(STATUS "Number of tests found: ${test_count}")
else()
    message(STATUS "No tests found.")
endif()