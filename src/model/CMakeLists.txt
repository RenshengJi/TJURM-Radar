file(GLOB_RECURSE
    data_src 
        ${CMAKE_SOURCE_DIR}/src/model/*.cpp
)


add_library(model
    STATIC
        ${data_src}    
)

target_link_libraries(model
    PUBLIC
        ${OpenCV_LIBS}
        ${OpenRM_LIBS}
        ${JSON_LIBS}
)
