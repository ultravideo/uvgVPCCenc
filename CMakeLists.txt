

cmake_minimum_required(VERSION 3.22)

# Include CMake modules
include(ExternalProject)

set(uvgVPCCenc_VER 0.1.0)
set(uvgVPCCenc_DESCR "uvgVPCCenc is an Real-Time V-PCC encoder library written in C++.")
set(uvgVPCCenc_URL "https://github.com/ultravideo/uvgVPCCenc")

set(KVAZAAR_REPO_URL "https://github.com/ultravideo/kvazaar.git")
set(KVAZAAR_REPO_TAGS "bb836ff1422f247b490230e9d429a2d6658cd1a1")

set(KVAZAAR_EP_UPDATE_DISCONNECTED ON)

option(BUILD_SHARED_LIBS "Build using shared uvgVPCCenc library" ON)

project(uvgVPCCenc VERSION ${uvgVPCCenc_VER})

if(MSVC)
add_compile_options(/W3 /WX)
else()
add_compile_options(-W -Wall -Wno-unused-variable)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(${USE_CLANG_TIDY})
  set(CLANG_TIDY_DEFAULT "clang-tidy;-checks=*,-llvmlibc-*,-llvm-*,-modernize-*,-fuchsia-*,-altera-*,-hicpp-use-auto,-bugprone-easily-swappable-parameters,-performance-avoid-endl,-cppcoreguidelines-pro-bounds-constant-array-index,-cppcoreguidelines-avoid-magic-numbers,-readability-magic-numbers,-readability-identifier-length,-readability-math-missing-parentheses,-readability-function-cognitive-complexity,-boost-use-ranges,-misc-non-private-member-variables-in-classes,-cppcoreguidelines-avoid-non-const-global-variables,-abseil-string-find-str-contains,-cppcoreguidelines-avoid-do-while,-readability-use-std-min-max,-google-build-using-namespace")
endif()

# Useful for debugging
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=undefined -fsanitize=leak -fno-omit-frame-pointer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize-recover=address,undefined")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize-address-use-after-scope")
endif()

# Run subdirectory make files
add_subdirectory(src/lib)
add_subdirectory(src/app)
# add_subdirectory(examples)

# Custom target to apply formating to all source file
# todo : check if user has the package : sudo apt-get install clang-format
add_custom_target(format
  find
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    -type f
    (
    -name "*.cc"
    -o
    -name "*.cpp"
    -o
    -name "*.h"
    -o
    -name "*.hpp"
    )
    -print0
  | xargs -0 clang-format -i
  COMMAND
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Formatting source code..."
  VERBATIM
)

# Find Doxygen package
find_package(Doxygen)

# Check if Doxygen is found
if(DOXYGEN_FOUND)
    # Set input and output directories
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # Configure Doxygen file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # Add a target to generate the API documentation
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()
# lf : Needed step to setup doxygen
# sudo apt-get install doxygen
# sudo apt-get install graphviz, and 
# git clone https://github.com/jothepro/doxygen-awesome-css.git
# sudo mkdir -p /usr/local/share/doxygen-awesome-css/
# sudo cp doxygen-awesome-css/doxygen-awesome.css /usr/local/share/doxygen-awesome-css/
# sudo cp doxygen-awesome-css/doxygen-awesome-sidebar-only.css /usr/local/share/doxygen-awesome-css/

# Include test configuration script
if(ENABLE_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()