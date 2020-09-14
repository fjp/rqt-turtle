cmake_minimum_required(VERSION 3.0.2)
project(rqt_turtle)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  actionlib
  turtle_actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5Widgets REQUIRED)
# https://docs.opencv.org/2.4/doc/tutorials/introduction/linux_gcc_cmake/linux_gcc_cmake.html
find_package(OpenCV REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_turtle
#  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp
#  DEPENDS system_lib
#)
# Call the macro without arguments because we don't want to export anything
# Note that this macro is required if catkin_install_python() is used!
# https://stackoverflow.com/questions/56801645/ros1-catkin-make-failed-catkin-install-python-called-without-required-destina
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rqt_turtle.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rqt_turtle_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


set(rqt_turtle_SRCS
  src/rqt_turtle/turtle_plugin.cpp
  src/rqt_turtle/service_caller.cpp
  src/rqt_turtle/draw.cpp
  src/rqt_turtle/turtle.cpp
  src/rqt_turtle/worker.cpp
)

set(rqt_turtle_HDRS
  include/rqt_turtle/turtle_plugin.h
  include/rqt_turtle/service_caller.h
  include/rqt_turtle/draw.h
  include/rqt_turtle/turtle.h
  include/rqt_turtle/worker.h
)

set(rqt_turtle_UIS
  resources/turtle_plugin.ui
  resources/ServiceCaller.ui
  resources/Draw.ui
  resources/Task.ui
)

set(rqt_turtle_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()


qt5_wrap_cpp(rqt_turtle_MOCS 
  ${rqt_turtle_HDRS}
)

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

qt5_wrap_ui(rqt_turtle_UIS_H 
  ${rqt_turtle_UIS}
)

message(${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")


include_directories(${rqt_turtle_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_turtle_SRCS} ${rqt_turtle_MOCS} ${rqt_turtle_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets ${OpenCV_LIBS})


# TODO use?
find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

#############
## Install ##
#############

# TODO????
# See http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin
# And https://github.com/ros-visualization/rqt_image_view/blob/master/CMakeLists.txt
# TODO?????
#Add the following lines to call the resource and plugin.xml # TODO required?
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
#install(DIRECTORY
#  resource
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#install(FILES
#  plugin.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# For our (optional) script to be installed to the right location, 
# if users install your package, this line is required
catkin_install_python(PROGRAMS scripts/rqt_turtle
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rqt_turtle.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
