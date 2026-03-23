#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    int suffix;

    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }
    else
    {
        try
        {
            suffix = std::stoi(argv[3]);
        }
        catch (const std::invalid_argument &e)
        {
            std::cerr << "Invalid argument: suffix must be an integer." << std::endl;
            return 2;
        }
        catch (const std::out_of_range &e)
        {
            std::cerr << "Out of range: suffix value is too large." << std::endl;
            return 2;
        }
    }

    // 错开不同巡视器的各种自增id
    ORB_SLAM3::Frame::nNextId = 1e9 * (suffix - 1);
    ORB_SLAM3::KeyFrame::nNextId = 1e9 * (suffix - 1);
    ORB_SLAM3::Map::nNextId = 1e9 * (suffix - 1);
    ORB_SLAM3::MapPoint::nNextId = 1e9 * (suffix - 1);

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool correct = true;
    // bool correct = false;
    auto pSLAM = std::make_shared<ORB_SLAM3::System>(argv[1], argv[2], ORB_SLAM3::System::RGBD, correct, true); //* 使用重载版本，倒数第二个correct
    auto node = std::make_shared<RgbdSlamNode>(pSLAM, suffix, correct);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    // auto pSLAM = std::make_shared<ORB_SLAM3::System>(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);  // 这里的true如果用变量代替，编译器会搞不清楚使用哪个重载
    // ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);

    return 0;
}
