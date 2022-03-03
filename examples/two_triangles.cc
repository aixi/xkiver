#include <string>
#include <opencv2/opencv.hpp>
#include <xkiver/transform.h>
#include <xkiver/rasterizer.h>

using namespace xkiver;

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    Rasterizer r(700, 700);

    Eigen::Vector3f eye_pos(0, 0, 1);


    std::vector<Eigen::Vector3f> pos{
        {-10, 0, -2},
        {0, -20, -2},
        {-20, 0, -2},
        {10, 10, -5},
        {25, 15, -5},
        {10, 5, -5}
    };

    std::vector<Eigen::Vector3i> ind{
        {0, 1, 2},
        {3, 4, 5}
    };

    std::vector<Eigen::Vector3f> colors{
        {217.0, 238.0, 185.0},
        {217.0, 238.0, 185.0},
        {217.0, 238.0, 185.0},
        {185.0, 217.0, 238.0},
        {185.0, 217.0, 238.0},
        {185.0, 217.0, 238.0}
    };

    auto pos_id = r.LoadPositions(pos);
    auto ind_id = r.LoadIndices(ind);
    auto col_id = r.LoadColors(colors);

    int key = 0;
    int frame_count = 0;

    Eigen::Matrix4f model = GetModelMatrix(angle);
    Eigen::Matrix4f view = GetViewMatrix(eye_pos, Eigen::Vector3f(0, 0, -1), Eigen::Vector3f(0, 1, 0));
    Eigen::Matrix4f projection = GetPerspectiveMatrix(45.0f, 1.0f, 0.1f, 50.0f);
    if (command_line)
    {
        r.Clear(BufferType::Color | BufferType::Depth);

        r.SetModelMatrix(model);
        r.SetViewMatrix(view);
        r.SetProjectionMatrix(projection);

        r.Draw(pos_id, ind_id, col_id, PrimitiveType::Triangle);
        cv::Mat image(700, 700, CV_32FC3, const_cast<Eigen::Vector3f*>(r.GetFrameBuffer().data()));
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.Clear(BufferType::Color | BufferType::Depth);

        r.SetModelMatrix(model);
        r.SetViewMatrix(view);
        r.SetProjectionMatrix(projection);

        r.Draw(pos_id, ind_id, col_id, PrimitiveType::TriangleLine);

        cv::Mat image(700, 700, CV_32FC3, const_cast<Eigen::Vector3f*>(r.GetFrameBuffer().data()));
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
