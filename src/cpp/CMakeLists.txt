cmake_minimum_required (VERSION 2.6)
project (ADA_STAR)

# FOR WINDOWS YOU MUST DEFINE THE FOLLOWING ENV VARIABLES
#BOOST_ROOT=/e/programs/boost_1_35_0
#BOOST_LIBRARYDIR=$BOOST_ROOT/stage/lib
find_package( Boost 1.37 COMPONENTS  regex) #NOTE note used for future use. 
add_executable(ada_star_bin ada_star.cpp main.cpp )
message(STATUS "${Boost_LIBRARIES}" )
target_link_libraries(ada_star_bin ${Boost_LIBRARIES} )
