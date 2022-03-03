#include <assert.h>
#include <math.h>
#include <limits>
#include <algorithm>
#include <iostream>
#include <boost/core/ignore_unused.hpp>
#include <xkiver/triangle.h>
#include <xkiver/rasterizer.h>
#include <xkiver/utils.h>

namespace xkiver
{

Rasterizer::Rasterizer(int window_width, int window_height) :  
    width_(window_width), 
    height_(window_height), 
    next_id_(0), 
    frame_buf_(width_ * height_),
    depth_buf_(width_ * height_, std::numeric_limits<float>::infinity())
{
}

void Rasterizer::SetModelMatrix(const Eigen::Matrix4f& m)
{
    model_ = m;
}

void Rasterizer::SetViewMatrix(const Eigen::Matrix4f& v)
{
    view_ = v;
}

void Rasterizer::SetProjectionMatrix(const Eigen::Matrix4f& p)
{
    projection_ = p;
}

PosBufId Rasterizer::LoadPositions(const std::vector<Eigen::Vector3f>& positions)
{
    int id = GetNextId();
    [[maybe_unused]] auto ret = pos_buf_.emplace(id, positions);
    assert(ret.second);
    return {id};
}

IndexBufId Rasterizer::LoadIndices(const std::vector<Eigen::Vector3i>& indices)
{
    int id = GetNextId();
    [[maybe_unused]] auto ret = index_buf_.emplace(id, indices);
    assert(ret.second);
    return {id};
}

ColorBufId Rasterizer::LoadColors(const std::vector<Eigen::Vector3f>& colors)
{
    int id = GetNextId();
    [[maybe_unused]] auto ret = color_buf_.emplace(id, colors);
    assert(ret.second);
    return {id};
}

void Rasterizer::SetPixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    frame_buf_[GetIndexInBuffer(static_cast<int>(point.x()), static_cast<int>(point.y()))] = color;
}

void Rasterizer::Clear(BufferType which_buffers)
{
    if ((which_buffers & BufferType::Color) == BufferType::Color)
    {
        std::fill(frame_buf_.begin(), frame_buf_.end(), Eigen::Vector3f(0, 0, 0));
    }
    if ((which_buffers & BufferType::Depth) == BufferType::Depth)
    {
        std::fill(depth_buf_.begin(), depth_buf_.end(), std::numeric_limits<float>::infinity());
    }
}

int Rasterizer::GetIndexInBuffer(int x, int y)
{
    return (height_ - 1 - y) * width_ + x;
}

void Rasterizer::RasterizeWireframe(const Triangle& triangle)
{
    const std::vector<Eigen::Vector3f>& vertexes = triangle.GetVertexes();
    DrawLine(vertexes[0], vertexes[1]);
    DrawLine(vertexes[1], vertexes[2]);
    DrawLine(vertexes[2], vertexes[0]);
}


// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void Rasterizer::DrawLine(const Eigen::Vector3f& begin, const Eigen::Vector3f& end)
{
    float x1 = begin.x();
    float y1 = begin.y();
    float x2 = end.x();
    float y2 = end.y();

    Eigen::Vector3f line_color(255.0f, 255.0f, 255.0f);

    int x;
    int y;
    int dx;
    int dy;
    int dx1;
    int dy1;
    int px;
    int py;
    int xe;
    int ye;
    int i;

    dx = static_cast<int>(x2 - x1);
    dy = static_cast<int>(y2 - y1);
    dx1 = static_cast<int>(fabs(dx));
    dy1 = static_cast<int>(fabs(dy));
    px = 2 * dy1 - dx1;
    py = 2 * dx1 - dy1;

    if(dy1 <= dx1)
    {
        if(dx >= 0)
        {
            x = static_cast<int>(x1);
            y = static_cast<int>(y1);
            xe = static_cast<int>(x2);
        }
        else
        {
            x = static_cast<int>(x2);
            y = static_cast<int>(y2);
            xe = static_cast<int>(x1);
        }
        Eigen::Vector3f point(static_cast<float>(x), static_cast<float>(y), 1.0f);
        SetPixel(point, line_color);
        for(i = 0; x < xe; ++i)
        {
            ++x;
            if(px < 0)
            {
                px = px + 2 * dy1;
            }
            else
            {
                if((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    ++y;
                }
                else
                {
                    --y;
                }
                px = px + 2 * (dy1 - dx1);
            }
            Eigen::Vector3f point2(static_cast<float>(x), static_cast<float>(y), 1.0f);
            SetPixel(point2, line_color);
        }
    }
    else
    {
        if(dy >= 0)
        {
            x = static_cast<int>(x1);
            y = static_cast<int>(y1);
            ye = static_cast<int>(y2);
        }
        else
        {
            x = static_cast<int>(x2);
            y = static_cast<int>(y2);
            ye = static_cast<int>(y1);
        }
        Eigen::Vector3f point3(static_cast<float>(x), static_cast<float>(y), 1.0f);
        SetPixel(point3, line_color);
        for(i = 0; y < ye; ++i)
        {
            ++y;
            if(py <= 0)
            {
                py = py + 2 * dx1;
            }
            else
            {
                if((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    ++x;
                }
                else
                {
                    --x;
                }
                py = py + 2 * (dx1 - dy1);
            }
            Eigen::Vector3f point4(static_cast<float>(x), static_cast<float>(y), 1.0f);
            SetPixel(point4,line_color);
        }
    }

}

void Rasterizer::RasterizeTriangle(const Triangle& triangle)
{
    Eigen::Vector4i bbox = triangle.GetBoundingBox2D();
    int min_x_pos = bbox[0];
    int min_y_pos = bbox[1];
    int max_x_pos = bbox[2];
    int max_y_pos = bbox[3];
    // for 2x2 MSAA
    std::vector<Eigen::Vector2f> small_pos {
        {0.25, 0.25}, 
        {0.25, 0.75},
        {0.72, 0.25}, 
        {0.75, 0.75}
    };
    for (int y = min_y_pos; y <= max_y_pos; ++y)
    {
        for (int x = min_x_pos; x <= max_x_pos; ++x)
        {
            float min_depth = std::numeric_limits<float>::max();
            int inside_count = 0;
            for (int i = 0; i < static_cast<int>(small_pos.size()); ++i)
            {
                float small_pos_x = static_cast<float>(x) + small_pos[i][0];
                float small_pos_y = static_cast<float>(y) + small_pos[i][1];
                if (IsinsideTriangle(small_pos_x, small_pos_y, triangle.GetVertexes()))
                {
                    std::vector<float> barycent_coord = ComputeBarycentric2D(small_pos_x, small_pos_y, triangle.GetVertexes());
                    // Perspective-Correct Interpolation
                    // ref: https://zhuanlan.zhihu.com/p/144331875
                    std::vector<Eigen::Vector4f> vertexes = triangle.ToVector4f();
                    float c0 = barycent_coord[0];
                    float c1 = barycent_coord[1];
                    float c2 = barycent_coord[2];
                    float w_reciprocal = 1.0f / (c0 / vertexes[0].w() + c1 / vertexes[1].w() + c2 / vertexes[2].w());
                    float z_interplated = c0 * vertexes[0].z() / vertexes[0].w() + 
                                        c1 * vertexes[1].z() / vertexes[1].w() + 
                                        c2 * vertexes[2].z() / vertexes[2].w();
                    z_interplated *= w_reciprocal;
                    min_depth = std::min(min_depth, z_interplated);
                    ++inside_count;
                }
            }
            if (inside_count > 0 && min_depth < depth_buf_[GetIndexInBuffer(x, y)])
            {
                Eigen::Vector3f color = triangle.GetColor() * inside_count / 4.0f;
                Eigen::Vector3f z_point(static_cast<float>(x), static_cast<float>(y), min_depth);
                depth_buf_[GetIndexInBuffer(x, y)] = min_depth;
                SetPixel(z_point, color);
            }
        }
    }
}

void Rasterizer::Draw(PosBufId pos_buf, IndexBufId idx_buf, ColorBufId color_buf, PrimitiveType type)
{
    // float f1 = (50.f - 0.1f) / 2.0f;
    // float f2 = (50.f + 0.1f) / 2.0f;
    const std::vector<Eigen::Vector3f>& vertice = pos_buf_[pos_buf.id];
    const std::vector<Eigen::Vector3i>& indexes = index_buf_[idx_buf.id];
    const std::vector<Eigen::Vector3f>& colors = color_buf_[color_buf.id];
    Eigen::Matrix4f mvp = projection_ * view_ * model_;
    for (const Eigen::Vector3i elements : indexes)
    {
        Triangle triangle;
        std::vector<Eigen::Vector4f> points;
        points.push_back(mvp * ToVec4(vertice[elements[0]], 1.0f));
        points.push_back(mvp * ToVec4(vertice[elements[1]], 1.0f));
        points.push_back(mvp * ToVec4(vertice[elements[2]], 1.0f));
        // Homogeneous division
        for (Eigen::Vector4f& point : points)
        {
            std::cout << point << std::endl;
            std::cout << "-------------------------" << std::endl;
            point /= point.w();
            std::cout << point << std::endl;
        }
        // viewport transformation
        for (Eigen::Vector4f& point : points)
        {
            point.x() = 0.5f * static_cast<float>(width_) * (point.x() + 1.0f);
            point.y() = 0.5f * static_cast<float>(height_) * (point.y() + 1.0f);
            // FIXME: why ?
            // point.z() = point.z() * f1 + f2;
        }
        if (type == PrimitiveType::Line)
        {
            std::cout << "PrimitiveType::Line not implemented" << std::endl;
        } 
        else
        { 
            for (int i = 0; i < 3; ++i)
            {
                triangle.SetVertex(i, points[i].head<3>());
                Eigen::Vector3f color = colors[elements[i]];
                triangle.SetColor(i, color.x(), color.y(), color.z());
            }
            if (type == PrimitiveType::Triangle)
            {
                RasterizeTriangle(triangle);
            }
            else if (type == PrimitiveType::TriangleLine) 
            {
                RasterizeWireframe(triangle);
            }
        }
    }
}

} // namespace xkiver