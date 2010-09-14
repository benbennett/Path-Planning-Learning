cmake_minimum_required (VERSION 2.6)
project (ADA_STAR)

# FOR WINDOWS YOU MUST DEFINE THE FOLLOWING ENV VARIABLES
#BOOST_ROOT=/e/programs/boost_1_35_0
#BOOST_LIBRARYDIR=$BOOST_ROOT/stage/lib
find_package( Boost 1.37 COMPONENTS  regex unit_test_framework ) #NOTE note used for future use. 
set(CMAKE_EXE_LINKER_FLAGS " -pg -ggdb")
set( CMAKE_BUILD_TYPE Debug)
add_executable(ada_star_bin ada_star.cpp stacktrace.cpp main.cpp )
message(STATUS "Boost: ${Boost_LIBRARIES}" )
target_link_libraries(ada_star_bin ${Boost_LIBRARIES} )
