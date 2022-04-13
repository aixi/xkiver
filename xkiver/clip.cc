#include <xkiver/clip.h>
#include <xkiver/utils.h>

namespace xkiver
{

Plane::Plane(const Eigen::Vector4f& out_normal, const Eigen::Vector4f& point) : 
    out_normal_(out_normal.normalized()), 
    point_(point) 
{
}

std::vector<Eigen::Vector4f> Plane::Clip(const std::vector<Eigen::Vector4f>& vertexes)
{
    std::vector<Eigen::Vector4f> result;
    size_t vertex_num = vertexes.size();
    for (size_t i = 0; i < vertex_num; ++i)
    {
        size_t curr_index = i;
        size_t prev_index = (i - 1 + vertex_num) % vertex_num;
        Eigen::Vector4f prev_vertex = vertexes[prev_index];
        Eigen::Vector4f curr_vertex = vertexes[curr_index];
        float d1 = Distance(prev_vertex);
        float d2 = Distance(curr_vertex);
        if (d1 * d2 < 0)
        {
            float t = d1 / (d1 - d2);
            Eigen::Vector4f intersect_point = prev_vertex + t * (curr_vertex - prev_vertex);
            result.push_back(intersect_point);
        }
        if (d2 < 0)
        {
            result.push_back(curr_vertex);
        }
    }
    return result;
}

float Plane::Distance(const Eigen::Vector4f& vertex)
{
    return out_normal_.dot(vertex - point_);
}

} // namespace xkiver