cmake_minimum_required(VERSION 3.2.1 FATAL_ERROR)

project(PC_OBJ_DETECT)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(helpers
            src/filter.cpp
            src/range_image.cpp)

add_executable(main src/main.cpp)
target_link_libraries(main 
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBS}
                      helpers)