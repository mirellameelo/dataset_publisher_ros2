#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <publisher/kitti_util.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <popl.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto img_dir_path = op.add<popl::Value<std::string>>("i", "img-dir", "directory path which contains images");
    auto img_fps = op.add<popl::Value<unsigned int>>("", "fps", "FPS of images", 7);

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!img_dir_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // initialize this node
    auto node = std::make_shared<rclcpp::Node>("image_publisher");
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;

    auto publisher_left = image_transport::create_publisher(node.get(), "camera/left/image_raw", custom_qos);
    auto publisher_right = image_transport::create_publisher(node.get(), "camera/right/image_raw", custom_qos);

    sensor_msgs::msg::Image::SharedPtr msg_left, msg_right;
    rclcpp::WallRate pub_rate(img_fps->value());
    rclcpp::executors::SingleThreadedExecutor exec;

    exec.add_node(node);

    kitti_sequence sequence(img_dir_path->value());
    const auto frames = sequence.get_frames();

    for (unsigned int i = 0; i < frames.size(); ++i) {

        const auto& frame = frames.at(i);
        const auto img_left = cv::imread(frame.left_img_path_, cv::IMREAD_COLOR);
        const auto img_right = cv::imread(frame.right_img_path_, cv::IMREAD_COLOR);

        msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_left).toImageMsg();
        msg_left->header.stamp = node->now();
        msg_left->header.frame_id = "camera_link";

        msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_right).toImageMsg();
        msg_right->header.stamp = msg_left->header.stamp;
        msg_right->header.frame_id = "camera_link";

        publisher_left.publish(msg_left);
        publisher_right.publish(msg_right);

        exec.spin_some();
        pub_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
