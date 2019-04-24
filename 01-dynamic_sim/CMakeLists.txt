# create an executable
ADD_EXECUTABLE (01-dynamic_sim 01-dynamic_sim.cpp)
ADD_EXECUTABLE (simviz simviz.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (01-dynamic_sim ${COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY ${PROJECT_SOURCE_DIR}/Model/world.urdf ${PROJECT_SOURCE_DIR}/Model/panda_arm_hand.urdf DESTINATION ${RESOURCE_DIR})