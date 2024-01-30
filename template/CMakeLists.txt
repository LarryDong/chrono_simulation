
cmake_minimum_required(VERSION 3.10)

project(my_project)



LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS Irrlicht
             OPTIONAL_COMPONENTS PardisoMKL
             CONFIG)


#--------------------------------------------------------------
# Important! To ensure ABI compatibility, use the same C++ standard
# as the one used to build the Chrono libraries.
#--------------------------------------------------------------

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})


include_directories(${CHRONO_INCLUDE_DIRS})
add_executable(my_demo my_example.cpp)


target_compile_definitions(my_demo PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(my_demo PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(my_demo PUBLIC ${CHRONO_LINKER_FLAGS})

# #--------------------------------------------------------------
# # Link to Chrono libraries and dependency libraries
# #--------------------------------------------------------------

target_link_libraries(my_demo ${CHRONO_LIBRARIES})
