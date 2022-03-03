#include <xkiver/utils.h>

namespace xkiver
{

float DegreeToRadian(float degree)
{
    return static_cast<float>(MY_PI) * degree / 180.0f;
}

float CrossProduct(float x1, float y1, float x2, float y2)
{
    return x1 * y2 - x2 * y1;
}

bool IsinsideTriangle(float x, float y, const std::vector<Eigen::Vector3f>& vertice)
{
    float x0 = vertice[0].x();
    float y0 = vertice[0].y();
    float x1 = vertice[1].x();
    float y1 = vertice[1].y();
    float x2 = vertice[2].x();
    float y2 = vertice[2].y();
    float p0 = CrossProduct(x - x0, y - y0, x1 - x0, y1 - y0);
    float p1 = CrossProduct(x - x1, y - y1, x2 - x1, y2 - y1);
    float p2 = CrossProduct(x - x2, y - y2, x0 - x2, y0 - y2);
    return (p0 > 0 && p1 >0 && p2 > 0) || (p0 < 0 && p1 < 0 && p2 < 0);
}

std::vector<float> ComputeBarycentric2D(float x, float y, const std::vector<Eigen::Vector3f>& vertice)
{
    float c1 = (x*(vertice[1].y() - vertice[2].y()) + (vertice[2].x() - vertice[1].x())*y + vertice[1].x()*vertice[2].y() - vertice[2].x()*vertice[1].y()) / (vertice[0].x()*(vertice[1].y() - vertice[2].y()) + (vertice[2].x() - vertice[1].x())*vertice[0].y() + vertice[1].x()*vertice[2].y() - vertice[2].x()*vertice[1].y());
    float c2 = (x*(vertice[2].y() - vertice[0].y()) + (vertice[0].x() - vertice[2].x())*y + vertice[2].x()*vertice[0].y() - vertice[0].x()*vertice[2].y()) / (vertice[1].x()*(vertice[2].y() - vertice[0].y()) + (vertice[0].x() - vertice[2].x())*vertice[1].y() + vertice[2].x()*vertice[0].y() - vertice[0].x()*vertice[2].y());
    float c3 = (x*(vertice[0].y() - vertice[1].y()) + (vertice[1].x() - vertice[0].x())*y + vertice[0].x()*vertice[1].y() - vertice[1].x()*vertice[0].y()) / (vertice[2].x()*(vertice[0].y() - vertice[1].y()) + (vertice[1].x() - vertice[0].x())*vertice[2].y() + vertice[0].x()*vertice[1].y() - vertice[1].x()*vertice[0].y());
    return {c1,c2,c3};
}

Eigen::Vector4f ToVec4(const Eigen::Vector3f& v3, float w)
{
    return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}

} // namespace xkiver