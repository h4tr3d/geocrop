cmake_minimum_required (VERSION 3.21)
project(GeoCrop)

set(APPLICATION_NAME geocrop)

# where to look first for cmake modules, before ${CMAKE_ROOT}/Modules/ is checked
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

# Compiler-specific C++20 activation.
set(CMAKE_CXX_STANDARD 20)
#set(CMAKE_CXX_STANDARD_REQUIRED yes)

# Warnings
set (GEOCROP_WARNING_OPTIONS
     $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
       -Wall -Wextra>
     $<$<CXX_COMPILER_ID:MSVC>:
       /W4>)

include_directories(.)

if(CMAKE_CROSSCOMPILING)
  set(QT_HEADERS_DIR "/usr/i486-mingw32/include")
  set(QT_LIBRARY_DIR "/usr/i486-mingw32/lib")

  set(CMAKE_EXE_LINKER_FLAGS 
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-subsystem,console -Wl,-enable-auto-import -Wl,-enable-runtime-pseudo-reloc")
endif()

if(WIN32)
  set(GDAL_DEV_PATH "C:\\GDAL" CACHE PATH "Path to directory containing GDAL include and lib, e.g. M:\\lib\\gdal")
  set(PROJ_DEV_PATH "C:\\PROJ" CACHE PATH "Path to directory containing PROJ.4 include and lib, e.g. M:\\lib\\PROJ")
endif(WIN32)

add_definitions(-D__STDC_CONSTANT_MACROS)

# Generate gitinfo
if(EXISTS ${CMAKE_SOURCE_DIR}/.git/)
    execute_process(COMMAND git describe --tags
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                    RESULT_VARIABLE result
                    OUTPUT_VARIABLE describe
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    if (${result} EQUAL 0)
        add_definitions(-DGEOCROP_VERSION_ADDON="-git+${describe}")
    else()
        add_definitions(-DGEOCROP_VERSION_ADDON="-git")
    endif()
endif()

#find_package(Threads)

find_package(GDAL REQUIRED)
find_package(PROJ4 REQUIRED)

add_executable(${APPLICATION_NAME} geocrop.cpp)
target_link_libraries(${APPLICATION_NAME} PRIVATE GDAL::gdal PROJ4::proj)
target_compile_options(${APPLICATION_NAME} PRIVATE ${GEOCROP_WARNING_OPTIONS})
