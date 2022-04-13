#include <boost/core/ignore_unused.hpp>
#include <xkiver/utils.h>
#include <xkiver/transform.h>

namespace xkiver
{

Eigen::Matrix4f GetModelMatrix(float angle)
{
    boost::ignore_unused(angle);
    return Eigen::Matrix4f::Identity();
}

Eigen::Matrix4f GetViewMatrix(const Eigen::Vector3f& eye, const Eigen::Vector3f& look_at, const Eigen::Vector3f& up)
{
    Eigen::Vector3f f((look_at - eye).normalized());
    Eigen::Vector3f s(f.cross(up).normalized());
    Eigen::Vector3f u(s.cross(f));
    Eigen::Matrix4f result(Eigen::Matrix4f::Identity());
    result(0, 0) = s[0];
    result(1, 0) = s[1];
    result(2, 0) = s[2];
    result(0, 1) = u[0];
    result(1, 1) = u[1];
    result(2, 1) = u[2];
    result(0, 2) = -f[0];
    result(1, 2) = -f[1];
    result(2, 2) = -f[2];
    result(3, 0) = -s.dot(eye);
    result(3, 1) = -u.dot(eye);
    result(3, 2) = f.dot(eye);
    return result;
    // boost::ignore_unused(look_at, up);
    // Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    // Eigen::Matrix4f translate;
    // translate << 1,0,0,-eye_pos[0],
    //              0,1,0,-eye_pos[1],
    //              0,0,1,-eye_pos[2],
    //              0,0,0,1;

    // view = translate*view;

}

Eigen::Matrix4f GetPerspectiveMatrix(float fov_degree, float aspect_ratio, float near, float far)
{
    float fov = DegreeToRadian(fov_degree);
    float n = near;
    float f = far;
    float t = static_cast<float>(tan(fov / 2.0f)) * fabsf(n);
    float b = -t;
    float r = aspect_ratio * t;
    float l = -r;
    Eigen::Matrix4f persp2ortho;
    persp2ortho << n, 0, 0, 0,
                  0, n, 0, 0,
                  0, 0, n + f, -n * f,
                  0, 0, 1, 0;
    Eigen::Matrix4f scale;
    scale << 2 / (r - l), 0, 0, 0,
             0, 2 / (t - b), 0, 0,
             0, 0, 2 / (n - f), 0,
             0, 0, 0, 1;
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -(r + l) / 2,
                 0, 1, 0, -(t + b) / 2,
                 0, 0, 1, -(n + f) / 2,
                 0, 0, 0, 1;
    return scale * translate * persp2ortho;
}

} // namespace xkiver