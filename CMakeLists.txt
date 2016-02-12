cmake_minimum_required (VERSION 2.8)
project(GeoCrop)

set(APPLICATION_NAME geocrop)

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

# Compiler-specific C++11 activation.
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if (NOT (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7))
        message(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.7 or greater.")
    endif ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
elseif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -stdlib=libc++")
else ()
    message(WARNING "I can't detect your C++11 compiler.")
endif ()

# where to look first for cmake modules, before ${CMAKE_ROOT}/Modules/ is checked
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

#find_package(Threads)
#find_package(PkgConfig)

find_package(GDAL REQUIRED)
find_package(PROJ REQUIRED)

include_directories(SYSTEM ${GDAL_INCLUDE_DIRS} ${PROJ_INCLUDE_DIRS})

add_executable(${APPLICATION_NAME} geocrop.cpp)
target_link_libraries(${APPLICATION_NAME} ${GDAL_LIBRARIES} ${PROJ_LIBRARIES})