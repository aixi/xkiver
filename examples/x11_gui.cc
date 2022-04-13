#include <assert.h>
#include <chrono>
#include <thread> 
#include <boost/core/ignore_unused.hpp>
#include <xkiver/display/platform.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wunused-variable"


framebuffer_t *framebuffer_create(int width, int height) {
    int color_buffer_size = width * height * 4;
    int depth_buffer_size = sizeof(float) * width * height;
    
    framebuffer_t *framebuffer;

    assert(width > 0 && height > 0);

    framebuffer = (framebuffer_t*)malloc(sizeof(framebuffer_t));
    framebuffer->width = width;
    framebuffer->height = height;
    framebuffer->color_buffer = (unsigned char*)malloc(color_buffer_size);
    for (int i = 0; i < width * height - 3; i += 4)
    {
        framebuffer->color_buffer[i * 4] = 0;
        framebuffer->color_buffer[i * 4 + 1] = 0;
        framebuffer->color_buffer[i * 4 + 2] = 255;
        framebuffer->color_buffer[i * 4 + 3] = 0;
    }

    framebuffer->depth_buffer = (float*)malloc(depth_buffer_size);


    return framebuffer;
}

void framebuffer_release(framebuffer_t *framebuffer) {
    free(framebuffer->color_buffer);
    free(framebuffer->depth_buffer);
    free(framebuffer);
}

int main()
{
    platform_initialize();
    const char* WINDOW_TITLE = "Viewer";
    int WINDOW_WIDTH = 800;
    int WINDOW_HEIGHT = 600;
    window_t* window = window_create(WINDOW_TITLE, WINDOW_WIDTH, WINDOW_HEIGHT);
    assert(window);
    boost::ignore_unused(window);
    framebuffer_t* framebuffer = framebuffer_create(WINDOW_WIDTH, WINDOW_HEIGHT);
    while (!window_should_close(window)) {
         window_draw_buffer(window, framebuffer);
    }

    platform_terminate();
}

#pragma GCC diagnostic pop