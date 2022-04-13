set(SRCS
    triangle.cc
    rasterizer.cc
    transform.cc
    utils.cc
    clip.cc
    framebuffer.cc
    display/linux.cc
)

add_library(xkiver SHARED ${SRCS})
target_compile_options(xkiver 
PUBLIC     
    -g 
    -Wall 
    -Wextra 
    -Werror 
    -Wconversion 
    -Wunused-parameter 
    -Wold-style-cast 
    -Woverloaded-virtual
    -Wpointer-arith
    -Wshadow
    -Wwrite-strings
    -std=c++17
)

target_link_libraries(xkiver 
PUBLIC    
${OpenCV_LIBRARIES} X11
)