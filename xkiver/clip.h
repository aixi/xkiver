#ifndef XKIVER_CLIP_H
#define XKIVER_CLIP_H

#include <vector>
#include <eigen3/Eigen/Eigen>

namespace xkiver
{

class Plane
{
public:
    Plane(const Eigen::Vector4f& out_normal, const Eigen::Vector4f& point = Eigen::Vector4f(0, 0, 0, 0));

    std::vector<Eigen::Vector4f> Clip(const std::vector<Eigen::Vector4f>& vertexes);

private:
    float Distance(const Eigen::Vector4f& vertex);
    
private:
    Eigen::Vector4f out_normal_;
    Eigen::Vector4f point_;
};

} // namespace xkiver


#endif /* XKIVER_CLIP_H */
