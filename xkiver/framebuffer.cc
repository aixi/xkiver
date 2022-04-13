#include <xkiver/framebuffer.h>

namespace xkiver
{

FrameBuffer::FrameBuffer(int width, int height) : 
    width_(width),
    height_(height),
    color_buffer_(width_ * height_ * 4),
    depth_buffer_(width_ * height_)
{
}

} // namespace xkiver