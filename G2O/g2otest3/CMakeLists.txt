cmake_minimum_required( VERSION 2.8 ) 
cmake_policy(VERSION 2.8) 
project ( g2o_test ) 
set( CMAKE_CXX_COMPILER "g++" ) 
set( CMAKE_CXX_FLAGS "-std=c++11 -O3 " ) 
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules ) 
############### dependencies ###################### 
# Eigen 
include_directories( "/usr/include/eigen3" ) 
# G2O 
find_package( G2O REQUIRED ) 
find_package( Cholmod REQUIRED ) 
include_directories( ${G2O_INCLUDE_DIRS} ) 
find_package( CSparse REQUIRED ) 
include_directories( 
${G2O_INCLUDE_DIRS} 
${CSPARSE_INCLUDE_DIR} 
${Cholmod_INCLUDE_DIR} 
) 
message("include_dir = " ${G2O_CORE_LIBRARY_DEBUG}) 
message("include_dir = " ${Cholmod_INCLUDE_DIR}) 
set( THIRD_PARTY_LIBS 
${Cholmod_LIBRARIES} 
#  ${G2O_CORE_LIBRARY_DEBUG} 
#   ${G2O_STUFF_LIBRARY_DEBUG} 
#  ${G2O_TYPES_SLAM3D_DEBUG} 
#  ${G2O_TYPES_SBA_DEBUG} 
cholmod 
g2o_types_slam3d 
g2o_types_sba 
g2o_core 
g2o_stuff
g2o_solver_dense.so 
g2o_solver_cholmod.so 
) 
# Opencv
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS " version:${OpenCV_VERSION}")
message(STATUS " libraries:${OpenCV_LIBS}")
message(STATUS " include path:${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})


add_executable( Source Source.cpp ) 
target_link_libraries( Source 
${THIRD_PARTY_LIBS} 
) 
target_link_libraries( Source
${OpenCV_LIBS}
)




