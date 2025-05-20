file(GLOB_RECURSE
    aruco_src
        ${CMAKE_SOURCE_DIR}/src/aruco/*.cpp
)


add_library(aruco
    STATIC
        ${aruco_src}
)

target_link_libraries(aruco
    PUBLIC
        ${OpenCV_LIBS}
        ${OpenRM_LIBS}
        ${JSON_LIBS}
)
