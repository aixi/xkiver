#ifndef XKIVER_TRANSFORM_H
#define XKIVER_TRANSFORM_H

#include <eigen3/Eigen/Eigen>

namespace xkiver
{

Eigen::Matrix4f GetModelMatrix(float angle);

Eigen::Matrix4f GetViewMatrix(const Eigen::Vector3f& eye, const Eigen::Vector3f& look_at, const Eigen::Vector3f& up);

Eigen::Matrix4f GetPerspectiveMatrix(float fov_degree, float aspect_ratio, float near, float far);

} // namespace xkiver

#endif /* XKIVER_TRANSFORM_H */
