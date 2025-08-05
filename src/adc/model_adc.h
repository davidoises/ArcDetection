#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace ARC::ADC
{
    class ADCModel : public rclcpp::Node
    {
        public:
            using MsgType = std_msgs::msg::Float32;

            ADCModel();

        private:

            void process();

            // Output data
            MsgType message_;

            // Periodic execution of the adc process
            rclcpp::TimerBase::SharedPtr timer_;
            std::chrono::milliseconds execution_period_;

            rclcpp::Publisher<MsgType>::SharedPtr publisher_;
    };
}