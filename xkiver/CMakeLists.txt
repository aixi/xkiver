set(SRCS
    triangle.cc
    rasterizer.cc
    transform.cc
    utils.cc
)

add_library(xkiver SHARED ${SRCS})
target_link_libraries(xkiver ${OpenCV_LIBRARIES})