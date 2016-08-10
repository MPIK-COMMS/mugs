cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(MugExamples)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

find_package(Eigen REQUIRED)
find_package(LibGp REQUIRED)
find_package(Boost COMPONENTS system thread timer filesystem REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${LIBGP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

link_directories(/usr/local/lib)

add_executable(train_model ${CMAKE_CURRENT_SOURCE_DIR}/mugs_evaluator.cpp)
target_link_libraries(mugs_evaluator
    /usr/local/lib/libmug.a
    ${LIBGP_LIBRARIES} 
    ${Boost_THREAD_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY} 
    ${Boost_SYSTEM_LIBRARY} 
    ${Boost_TIMER_LIBRARY} 
    )

