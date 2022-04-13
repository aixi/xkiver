add_executable(two_triangles two_triangles.cc)
target_link_libraries(two_triangles 
PUBLIC
    xkiver
)

add_executable(x11_gui x11_gui.cc)
target_link_libraries(x11_gui
PUBLIC
    xkiver
)