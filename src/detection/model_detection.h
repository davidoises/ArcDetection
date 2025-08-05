#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace ARC::DETECT
{
    class DetectionModel : public rclcpp::Node
    {
        public:
            using MsgType = std_msgs::msg::Float32;

            DetectionModel();

        private:

            void process(const MsgType::UniquePtr msg);

            rclcpp::Subscription<MsgType>::SharedPtr subscription_;
    };
}