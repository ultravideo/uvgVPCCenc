if(ENABLE_V3CRTP)
	# Check if uvgV3CRTP is already available
	find_package(uvgV3CRTP 0.11.1 QUIET) #TODO: update min version to a version with necessary bug fixes etc.

	# Try pkgConfig as well
	find_package(PkgConfig QUIET)
	if(PkgConfig_FOUND)
		if(NOT UVGV3CRTP_FOUND)
			pkg_search_module(UVGV3CRTP uvgv3crtp>=0.11.1 uvgV3CRTP>=0.11.1)
		endif()
	endif()

	# If we did not find uvgV3CRTP build it from source_group
	if(UVGV3CRTP_FOUND)
		message(STATUS "Found uvgV3CRTP")
	else()
		message(STATUS "uvgV3CRTP not found, building from source...")
		
		#include(FetchContent)
		find_package(Git REQUIRED)
		
		option(GIT_SUBMODULE "Check submodules during build" ON)
		if(GIT_SUBMODULE)
			message(STATUS "Update submodule")
			execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
							WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
							RESULT_VARIABLE GIT_SUBMOD_RESULT)
			if(NOT GIT_SUBMOD_RESULT EQUAL "0")
				message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodule")
			endif()
		endif()
		
		# Add subdir and specify relevant options
		set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

		set(UVGV3CRTP_DISABLE_TESTS ON CACHE BOOL "" FORCE)
		set(UVGV3CRTP_DISABLE_EXAMPLES  ON CACHE BOOL "" FORCE)
		set(UVGV3CRTP_DISABLE_INSTALL OFF CACHE BOOL "" FORCE)
		
		# Disable uvgRTP prints unless using a debug prints
		if(ENABLE_UVGRTP_PRINTS)
			set(UVGRTP_DISABLE_PRINTS OFF CACHE BOOL "" FORCE)
		else()
			set(UVGRTP_DISABLE_PRINTS ON CACHE BOOL "" FORCE)
		endif()
		
		add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/uvgV3CRTP)
		
		include_directories(${uvgv3crtp_SOURCE_DIR}/include)
		link_directories(${uvgv3crtp_SOURCE_DIR})
		
		unset(BUILD_SHARED_LIBS)

	endif()
endif()

# include and link directories for dependencies. 
# These are needed when the compilation happens a second time and the library 
# is found so no compilation happens. Not needed in every case, but a nice backup
include_directories(${CMAKE_BINARY_DIR}/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib)
