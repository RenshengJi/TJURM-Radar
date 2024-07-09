file(GLOB_RECURSE
    data_src 
        ${CMAKE_SOURCE_DIR}/src/serial/*.cpp
)


add_library(Serial
    STATIC
        ${data_src}    
)

target_link_libraries(Serial
    PUBLIC
        serial
        ${OpenCV_LIBS}
        ${OpenRM_LIBS}
)
