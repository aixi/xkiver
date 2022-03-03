#ifndef XKIVER_TRIANGLE_H
#define XKIVER_TRIANGLE_H

#include <vector>
#include <eigen3/Eigen/Eigen>
#include <xkiver/base/types.h>

namespace xkiver
{

class Triangle
{
public:
    Triangle();

    void SetVertex(int index, const Eigen::Vector3f& value);
    
    void SetNoraml(int index, const Eigen::Vector3f& normal);
    
    void SetColor(int index, float r, float g, float b);
    
    Eigen::Vector3f GetColor() const 
    {
        return color_[0] * 255;
    }

    const std::vector<Eigen::Vector3f>& GetVertexes() const
    {
        return vertexes_;
    }

    void SetTexCoord(int index, float s, float t);

    std::vector<Eigen::Vector4f> ToVector4f() const;

    Eigen::Vector4i GetBoundingBox2D() const;
    
private:
    std::vector<Eigen::Vector3f> vertexes_; // CCW
    std::vector<Eigen::Vector3f> color_;
    std::vector<Eigen::Vector2f> tex_coord_;
    std::vector<Eigen::Vector3f> normal_;
};

} // namespace xkiver

#endif /* XKIVER_TRIANGLE_H */
