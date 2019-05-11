set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BINARY_DIR}/01-kinematic_sim)

# create an executable
ADD_EXECUTABLE (01-kinematic_sim 01-kinematic_sim.cpp ${COMMON_SOURCE})
ADD_EXECUTABLE (test test.cpp ${COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (01-kinematic_sim ${COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (test ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_hand.urdf DESTINATION ${APP_RESOURCE_DIR})
