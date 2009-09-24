cmake_minimum_required (VERSION 2.6)
project (ADA_STAR)

add_executable(ada_star_bin main.cpp ada_star.cpp)
target_link_libraries(ada_star_bin)
