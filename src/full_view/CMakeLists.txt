file(GLOB_RECURSE
    data_src 
        ${CMAKE_SOURCE_DIR}/src/full_view/*.cpp
)


add_library(full_view
    STATIC
        ${data_src}    
)

target_link_libraries(full_view
    PUBLIC
        ${OpenCV_LIBS}
        ${OpenRM_LIBS}
        ${JSON_LIBS}
        ${PCL_LIBRARIES}
)
