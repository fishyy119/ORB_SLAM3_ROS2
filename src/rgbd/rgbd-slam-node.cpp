#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(std::shared_ptr<ORB_SLAM3::System> pSLAM, int suffix)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{

    // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/rgb_f/image_color");
    // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/depth_f/image");
    // FIX: https://github.com/zang09/ORB_SLAM3_ROS2/issues/24

    std::string rgb_topic = "/rgb_f/img_" + std::to_string(suffix);
    std::string depth_topic = "/depth_f/img_" + std::to_string(suffix);
    std::string pose_topic = "robot_pose_" + std::to_string(suffix);
    std::string sim_time_topic = "simulation_time_" + std::to_string(suffix);
    m_suffix = suffix;

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, rgb_topic);
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, depth_topic);

    pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        pose_topic, 10, [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
            m_pose = *msg;
        });

    sim_time_sub = this->create_subscription<std_msgs::msg::Float64>(
        sim_time_topic, 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
            m_simulation_time = msg->data;
        });

    pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        "robot_pose_1", 10, [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
            m_pose = *msg;
        });

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

}

RgbdSlamNode::~RgbdSlamNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down SLAM...");
    // Stop all threads
    m_SLAM->Shutdown();

    // 不使用配置文件设置保存路径，在这里手动调用
    m_SLAM->SaveAtlasWithPath(ORB_SLAM3::System::FileType::BINARY_FILE, "atlas_" + std::to_string(m_suffix));

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_" + std::to_string(m_suffix) + ".txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // RCLCPP_INFO(this->get_logger(), "Received RGB message, timestamp: %d", msgRGB->header.stamp.sec);
    // RCLCPP_INFO(this->get_logger(), "Received Depth message, timestamp: %d", msgD->header.stamp.sec);
    
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // cv::Mat resizedRGB, resizedD;
    // cv::resize(cv_ptrRGB->image, resizedRGB, cv::Size(640, 480));
    // cv::resize(cv_ptrD->image, resizedD, cv::Size(640, 480));

    // RCLCPP_INFO(this->get_logger(), "Depth image type: %s", cv_ptrD->image.type() == CV_32F ? "float" : "not float");

    bool correct = true;

    // Convert Pose message to Sophus::SE3f
    Eigen::Quaternionf q(m_pose.orientation.w, m_pose.orientation.x, m_pose.orientation.y, m_pose.orientation.z);
    Eigen::Vector3f t(m_pose.position.x, m_pose.position.y, m_pose.position.z);
    Sophus::SE3f poseSophus(q, t);
    // RCLCPP_INFO(this->get_logger(), "Pose received: position (%f, %f, %f), orientation (%f, %f, %f, %f)",
    //             m_pose.position.x, m_pose.position.y, m_pose.position.z,
    //             m_pose.orientation.x, m_pose.orientation.y, m_pose.orientation.z, m_pose.orientation.w);

    // RCLCPP_INFO(this->get_logger(), "Converted Sophus::SE3f pose: translation (%f, %f, %f), quaternion (%f, %f, %f, %f)",
    //             poseSophus.translation().x(), poseSophus.translation().y(), poseSophus.translation().z(),
    //             poseSophus.unit_quaternion().x(), poseSophus.unit_quaternion().y(), poseSophus.unit_quaternion().z(), poseSophus.unit_quaternion().w());

    m_SLAM->TrackRGBD1(cv_ptrRGB->image, cv_ptrD->image, m_simulation_time, correct, poseSophus);
    // m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, m_simulation_time);
    // m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
    
    // m_SLAM->TrackRGBD(resizedRGB, resizedD, Utility::StampToSec(msgRGB->header.stamp));

}