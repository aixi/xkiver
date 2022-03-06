#ifndef XKIVER_RASTERIZER_H
#define XKIVER_RASTERIZER_H

#include <map>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <xkiver/triangle.h>
#include <xkiver/base/types.h>

namespace xkiver
{

enum class BufferType : unsigned char
{
    Color = 1,
    Depth = 2
};

BufferType operator|(BufferType a, BufferType b)
{
    return BufferType(static_cast<unsigned char>(a) | static_cast<unsigned char>(b));
}

BufferType operator&(BufferType a, BufferType b)
{
    return BufferType(static_cast<unsigned char>(a) & static_cast<unsigned char>(b));
}

enum class PrimitiveType
{
    Line,
    Triangle,
    TriangleLine
};

// For index type safe
struct PosBufId
{
    int id = 0;
};

struct IndexBufId
{
    int id = 0;
};

struct ColorBufId
{
    int id = 0;
};

// VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI -> FRAGSHADER

class Rasterizer
{
public:
    Rasterizer(int window_width, int window_height);
    
    DISALLOW_COPY_AND_ASSIGN(Rasterizer);

    PosBufId LoadPositions(const std::vector<Eigen::Vector3f>& positions);
    IndexBufId LoadIndices(const std::vector<Eigen::Vector3i>& indices);
    ColorBufId LoadColors(const std::vector<Eigen::Vector3f>& colors);

    void SetModelMatrix(const Eigen::Matrix4f& m);
    void SetViewMatrix(const Eigen::Matrix4f& v);
    void SetProjectionMatrix(const Eigen::Matrix4f& p);

    void SetPixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

    void Clear(BufferType which_buffers);

    void Draw(PosBufId pos_buf, IndexBufId idx_buf, ColorBufId color_buf, PrimitiveType type);

    const std::vector<Eigen::Vector3f>& GetFrameBuffer() const 
    {
        return frame_buf_;
    }

private:
    void RasterizeWireframe(const Triangle& triangle);

    void DrawLine(const Eigen::Vector3f& begin, const Eigen::Vector3f& end);

    void RasterizeTriangle(const Triangle& triangle);

private:
    int GetNextId() 
    {
        return next_id_++;
    }

    int GetIndexInBuffer(int x, int y);

private:
    int width_;
    int height_;
    int next_id_;
    
    std::vector<Eigen::Vector3f> frame_buf_;
    std::vector<float> depth_buf_;
    
    Eigen::Matrix4f model_;
    Eigen::Matrix4f view_;
    Eigen::Matrix4f projection_;

    std::map<int, std::vector<Eigen::Vector3f>> pos_buf_;
    std::map<int, std::vector<Eigen::Vector3i>> index_buf_;
    std::map<int, std::vector<Eigen::Vector3f>> color_buf_;

};

} // namespace xkiver

#endif /* XKIVER_RASTERIZER_H */
