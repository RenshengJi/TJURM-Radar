file(GLOB_RECURSE
    data_src 
        ${CMAKE_SOURCE_DIR}/src/fuse/*.cpp
)


add_library(fuse
    STATIC
        ${data_src}    
)

target_link_libraries(fuse
    PUBLIC
        ${OpenCV_LIBS}
        ${OpenRM_LIBS}
        ${JSON_LIBS}
)
