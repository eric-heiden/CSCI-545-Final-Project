#############################################################################
#############################################################################
#
#  This is a CMakeList.txt file originally programmed for the CLMC/AMD labs
#  at the University of Southern California and the Max-Planck-Institute for
#  Intelligent Systems. We use a mixutre of explicit makefiles and cmake, but 
#  primarily we rely on cmake for all major compile dependencies. All our
#  software is provided under a slightly modified version of the LGPL license
#  to be found at http://www-clmc.usc.edu/software/license.
#
#  Copyright by Stefan Schaal, 2014
#
#############################################################################
#############################################################################
# which version are we using

cmake_minimum_required(VERSION 2.8)

#############################################################################
# include common cmake components

include($ENV{LAB_ROOT}/config/cmake/LAB.cmake)

#############################################################################
# user defined cmake components

# set global compile type
set(CMAKE_BUILD_TYPE RelWithDebInfo) # Optimization with debugging info
#set(CMAKE_BUILD_TYPE Release)       # Optimization 
#set(CMAKE_BUILD_TYPE Debug)         # Debug

# the robot name
set(NAME "nao")

# local defines
include_directories(BEFORE $ENV{LAB_ROOT}/${NAME}/include)
include_directories(BEFORE $ENV{LAB_ROOT}/${NAME}/math)
include_directories(BEFORE ../include)
include_directories(BEFORE ../src)


# ------------------------------------------------------------------------

set(SRCS_XTASK
	initUserTasks.c
	stepsequence.hpp
    	balance_task.cpp
	shift_task.cpp
	step_aerobic_task.cpp
	helper.hpp
	)

set(SRCS_XOPENGL
	initUserGraphics.c
	)

set(SRCS_XSIM
	initUserSimulation.c
	)

set(BINARIES
	$ENV{LAB_ROOT}/${NAME}/$ENV{MACHTYPE}/x${NAME}
	$ENV{LAB_ROOT}/${NAME}/$ENV{MACHTYPE}/xmotor
	$ENV{LAB_ROOT}/${NAME}/$ENV{MACHTYPE}/xpest
	)

set(RBINARIES
	$ENV{LAB_ROOT}/${NAME}/$ENV{MACHTYPE}/xr${NAME}
	$ENV{LAB_ROOT}/${NAME}/$ENV{MACHTYPE}/xrmotor
	)

# ------------------------------------------------------------------------

install(PROGRAMS ${BINARIES} DESTINATION ${CMAKE_BINARY_DIR})

add_executable(xtask ${SRCS_XTASK})
target_link_libraries(xtask SLtask SLcommon ${NAME}_task ${NAME} utility ${LAB_STD_LIBS})

add_executable(xopengl ${SRCS_XOPENGL})
target_link_libraries(xopengl SLopenGL SLcommon ${NAME}_openGL ${NAME} utility ${LAB_STD_LIBS})

add_executable(xsimulation ${SRCS_XSIM})
target_link_libraries(xsimulation  SLsimulation SLcommon ${NAME}_simulation ${NAME} utility ${LAB_STD_LIBS})



install(PROGRAMS ${RBINARIES} DESTINATION ${CMAKE_BINARY_DIR})

add_executable(xrtask ${SRCS_XTASK})
target_link_libraries(xrtask SLtask SLcommon r${NAME}_task ${NAME} utility ${LAB_STD_LIBS})

