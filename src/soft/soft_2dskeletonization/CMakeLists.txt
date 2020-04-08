include_directories(${CMAKE_SOURCE_DIR}/src/lib
					${CMAKE_SOURCE_DIR}/src/utils
					${Boost_INCLUDE_DIR}
					${SFML_INCLUDE_DIR})

set(source_files main.cpp)

#Déclaration de l'exécutable

set(EXEC_NAME soft_2dskeletonization)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/")

add_executable(${EXEC_NAME} ${source_files})

target_link_libraries(${EXEC_NAME} ${MATHTOOLS_LIB}
								   ${SKELETON_LIB}
								   ${ALGORITHM_LIB}
								   ${DISPLAYOCV_LIB}
								   ${Boost_PROGRAM_OPTIONS_LIBRARY}
								   ${OpenCV_LIBS})

