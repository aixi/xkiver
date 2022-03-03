#include <assert.h>
#include <algorithm>
#include <xkiver/triangle.h>

namespace xkiver
{

Triangle::Triangle() : vertexes_(3), colors_(3), tex_coords_(3), normals_(3)
{
}

void Triangle::SetVertex(int index, const Eigen::Vector3f& value)
{
    vertexes_[index] = value;
}

void Triangle::SetNormal(int index, const Eigen::Vector3f& normal)
{
    normals_[index] = normal;
}

void Triangle::SetColor(int index, float r, float g, float b)
{
    assert(r >= 0 && r <= 255);
    assert(g >= 0 && g <= 255);
    assert(b >= 0 && b <= 255);
    colors_[index] << r / 255, g / 255, b / 255;
}

void Triangle::SetNormals(const std::vector<Eigen::Vector3f>& normals)
{
    normals_ = normals;
}

void Triangle::SetColors(const std::vector<Eigen::Vector3f>& colors)
{
    colors_ = colors;
}

void Triangle::SetTexCoord(int index, float s, float t)
{
    tex_coords_[index] << s, t;
}

std::vector<Eigen::Vector4f> Triangle::ToVector4f() const
{
    std::vector<Eigen::Vector4f> result(3);
    std::transform(vertexes_.begin(), vertexes_.end(), result.begin(), [](auto& v){return Eigen::Vector4f(v.x(), v.y(), v.z(), 1.0f);});
    return result;
}

Eigen::Vector4i Triangle::GetBoundingBox2D() const
{
    float min_x = std::min(std::min(vertexes_[0][0], vertexes_[1][0]), vertexes_[2][0]);
    float max_x = std::max(std::max(vertexes_[0][0], vertexes_[1][0]), vertexes_[2][0]);
    float min_y = std::min(std::min(vertexes_[0][1], vertexes_[1][1]), vertexes_[2][1]);
    float max_y = std::max(std::max(vertexes_[0][1], vertexes_[1][1]), vertexes_[2][1]);
    int min_x_pos = static_cast<int>(floor(min_x));
    int min_y_pos = static_cast<int>(floor(min_y));
    int max_x_pos = static_cast<int>(ceil(max_x));
    int max_y_pos = static_cast<int>(ceil(max_y));
    Eigen::Vector4i result(min_x_pos, min_y_pos, max_x_pos, max_y_pos);
    return result;
}   

} // namespace xkiver