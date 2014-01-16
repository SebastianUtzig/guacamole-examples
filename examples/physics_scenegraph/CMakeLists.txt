# determine source and header files
file(GLOB_RECURSE EXAMPLE_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.c *.cpp)

GET_FILENAME_COMPONENT(_EXE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
SET(_EXE_NAME ${_EXE_NAME}.out)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

ADD_EXECUTABLE( ${_EXE_NAME}
    ${EXAMPLE_SRC}
)

TARGET_LINK_LIBRARIES( ${_EXE_NAME} debug ${LIBS} optimized ${LIBS})

# copy runtime libraries as a post-build process
IF (MSVC)
    gua_copy_runtime_dependencies ( ${_EXE_NAME} \"${GLOBAL_EXT_DIR}/bin/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")
    FOREACH (_LIB_DIR ${GUACAMOLE_LIBRARY_DIRS})
        gua_copy_runtime_dependencies ( ${_EXE_NAME} \"${_LIB_DIR}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")
    ENDFOREACH (_LIB_DIR ${GUACAMOLE_LIBRARY_DIRS})
ENDIF (MSVC)