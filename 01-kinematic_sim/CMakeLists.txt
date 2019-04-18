# create an executable
ADD_EXECUTABLE (01-kinematic_sim main.cpp)
ADD_EXECUTABLE (test test.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (01-kinematic_sim ${COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (test ${COMMON_LIBRARIES})


# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY ${PROJECT_SOURCE_DIR}/Model/world.urdf ${PROJECT_SOURCE_DIR}/Model/panda_arm_hand.urdf DESTINATION ${RESOURCE_DIR})
