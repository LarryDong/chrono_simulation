
cmake_minimum_required(VERSION 3.10)

project(my_project)



LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")


set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")

list(APPEND LIBRARIES "ChronoEngine")
list(APPEND LIBRARIES "ChronoEngine_sensor")
list(APPEND LIBRARIES "ChronoEngine_irrlicht")
list(APPEND LIBRARIES "ChronoEngine_vehicle")
list(APPEND LIBRARIES "ChronoEngine_vehicle_irrlicht")
list(APPEND LIBRARIES "ChronoModels_vehicle")


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
add_executable(my_chrono 
    chrono_main.cpp
    my_sensors.cpp
)

#~ Do not know the meanings, but cannot be deleted.
set_target_properties(my_chrono PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
target_compile_definitions(my_chrono PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(my_chrono PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(my_chrono PUBLIC ${CHRONO_LINKER_FLAGS})
target_link_libraries(my_chrono ${CHRONO_LIBRARIES}  ${LIBRARIES})


# LWX paper 

add_executable(lwx lwx_main.cpp)
#~ Do not know the meanings, but cannot be deleted.
set_target_properties(lwx PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
target_compile_definitions(lwx PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(lwx PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(lwx PUBLIC ${CHRONO_LINKER_FLAGS})
target_link_libraries(lwx ${CHRONO_LIBRARIES}  ${LIBRARIES})


