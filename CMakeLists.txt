cmake_minimum_required(VERSION 3.22)

# Include CMake modules
include(ExternalProject)

set(uvgVPCCenc_VER 1.0.0)
set(uvgVPCCenc_DESCR "uvgVPCCenc is a Real-Time V-PCC encoder library written in C++.")
set(uvgVPCCenc_URL "https://github.com/ultravideo/uvgVPCCenc")

set(KVAZAAR_REPO_URL "https://github.com/ultravideo/kvazaar.git")
set(KVAZAAR_REPO_TAGS "bb836ff1422f247b490230e9d429a2d6658cd1a1")

set(KVAZAAR_EP_UPDATE_DISCONNECTED ON)

option(BUILD_SHARED_LIBS "Build using shared uvgVPCCenc library" ON)

project(uvgVPCCenc
  VERSION ${uvgVPCCenc_VER}
  DESCRIPTION ${uvgVPCCenc_DESCR}
  HOMEPAGE_URL ${uvgVPCCenc_URL}
)

set(DEFERRED "@")

# if(MSVC)
# add_compile_options(/W3 /WX)
# else()
# add_compile_options(-W -Wall -Wno-unused-variable)
# endif()

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

# Doxygen configuration
if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY  ${CMAKE_SOURCE_DIR}/docs)
  set(DOXYGEN_DISABLE_INDEX NO)
  set(DOXYGEN_FULL_SIDEBAR NO)
  set(DOXYGEN_MARKDOWN_SUPPORT YES)
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "${CMAKE_SOURCE_DIR}/README.md")
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_SHOW_HEADERFILE YES)
  set(RECURSIVE YES)
  
  set(DOXYGEN_HAVE_DOT YES)
  set(DOXYGEN_CALL_GRAPH YES)
  set(DOXYGEN_CALLER_GRAPH YES)
  set(DOXYGEN_GRAPHICAL_HIERARCHY YES)
  set(DOXYGEN_UML_LOOK YES)
  set(DOXYGEN_CLASS_GRAPH YES)
  set(DOXYGEN_INTERACTIVE_SVG YES)
  set(DOXYGEN_DOT_IMAGE_FORMAT svg)

    doxygen_add_docs(
      docs
      README.md ${CMAKE_SOURCE_DIR}/src
      COMMENT "Generating API documentation with Doxygen"
    )
endif()

# INSTALL
include(GNUInstallDirs)

# Set correct pkgconfig libdir variable
if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
  set(UVGVPCCENC_PC_LIBDIR "${CMAKE_INSTALL_LIBDIR}")
else()
  set(UVGVPCCENC_PC_LIBDIR "\${exec_prefix}/${CMAKE_INSTALL_LIBDIR}")
endif()

# Set correct pkgconfig include variable
if(IS_ABSOLUTE "${CMAKE_INSTALL_INCLUDEDIR}")
  set(UVGVPCCENC_PC_INCDIR "${CMAKE_INSTALL_INCLUDEDIR}")
else()
  set(UVGVPCCENC_PC_INCDIR "\${exec_prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/lib/uvgvpccenc.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/src/lib/uvgvpccenc.pc.temp" @ONLY)
install(CODE "configure_file(\"${CMAKE_CURRENT_BINARY_DIR}/src/lib/uvgvpccenc.pc.temp\" \"${CMAKE_CURRENT_BINARY_DIR}/src/lib/uvgvpccenc.pc\" @ONLY)")

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/src/lib/uvgvpccenc.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
install(TARGETS uvgVPCCenc DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS uvgvpccenc
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/lib/include/uvgvpcc DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Include test configuration script
if(ENABLE_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()