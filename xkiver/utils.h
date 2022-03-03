#ifndef XKIVER_UTILS_H
#define XKIVER_UTILS_H

#include <vector>
#include <eigen3/Eigen/Eigen>
#include <xkiver/constants.h>

namespace xkiver
{

float DegreeToRadian(float degree);

float CrossProduct(float x1, float y1, float x2, float y2);

bool IsinsideTriangle(float x, float y, const std::vector<Eigen::Vector3f>& vertice);

std::vector<float> ComputeBarycentric2D(float x, float y, const std::vector<Eigen::Vector3f>& vertice);

Eigen::Vector4f ToVec4(const Eigen::Vector3f& v3, float w = 1.0f);

} // namespace xkiver

#endif /* XKIVER_UTILS_H */
