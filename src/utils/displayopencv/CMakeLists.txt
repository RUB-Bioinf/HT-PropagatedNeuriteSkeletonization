set(LIBRARY_NAME ${DISPLAYOCV_LIB})

set(SOURCE_FILES DisplayShapeOCV.cpp
                 DisplayBoundaryOCV.cpp
                 DisplaySkeletonOCV.cpp
				 DisplayBsplineOCV.cpp
				 DisplayPointsOCV.cpp)

# make the library
add_library(
    ${LIBRARY_NAME}
    ${INCLUDE_FILES}
    ${SOURCE_FILES}
	)

include_directories(${CMAKE_SOURCE_DIR}/src/lib/
					${OpenCV_INCLUDE_DIRS}
					${Boost_INCLUDE_DIR})

target_link_libraries(${LIBRARY_NAME} ${MATHTOOLS_LIB}
									  ${SHAPE_LIB}
									  ${BOUNDARY_LIB}
									  ${OPENCV_LIBS})


SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES LINKER_LANGUAGE CXX)
