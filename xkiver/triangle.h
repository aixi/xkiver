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
    
    void SetNormal(int index, const Eigen::Vector3f& normal);

    void SetNormals(const std::vector<Eigen::Vector3f>& normals);

    void SetColors(const std::vector<Eigen::Vector3f>& colors);
    
    void SetColor(int index, float r, float g, float b);

    void SetTexCoord(int index, float s, float t);
    
    Eigen::Vector3f GetColor(int index) const 
    {
        return colors_[index] * 255;
    }

    const std::vector<Eigen::Vector3f>& GetVertexes() const
    {
        return vertexes_;
    }

    std::vector<Eigen::Vector4f> ToVector4f() const;

    Eigen::Vector4i GetBoundingBox2D() const;
    
private:
    std::vector<Eigen::Vector3f> vertexes_; // CCW
    std::vector<Eigen::Vector3f> colors_;
    std::vector<Eigen::Vector2f> tex_coords_;
    std::vector<Eigen::Vector3f> normals_;
};

} // namespace xkiver

#endif /* XKIVER_TRIANGLE_H */
