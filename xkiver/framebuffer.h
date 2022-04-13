#ifndef XKIVER_FRAMEBUFFER_H
#define XKIVER_FRAMEBUFFER_H

#include <stdint.h>
#include <vector>

namespace xkiver
{

class FrameBuffer
{
public:
    FrameBuffer(int width, int height);

private:
    int width_;
    int height_;
    std::vector<uint8_t> color_buffer_;
    std::vector<float> depth_buffer_;
};

} // namespace xkiver 

#endif /* XKIVER_FRAMEBUFFER_H */
