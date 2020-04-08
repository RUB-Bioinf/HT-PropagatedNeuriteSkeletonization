set(LIBRARY_NAME ${SKELETON_LIB})

include_directories(${CMAKE_SOURCE_DIR}/src/lib)
set(SOURCE_FILES    ReconstructionBranch.cpp
					model/Projective.cpp
					model/Perspective.cpp
					model/Orthographic.cpp)
# make the library
add_library(
    ${LIBRARY_NAME}
    ${INCLUDE_FILES}
    ${SOURCE_FILES}
	)

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES LINKER_LANGUAGE CXX)


