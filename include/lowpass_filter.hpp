#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"


#include "imu_subscriber.hpp"
#include "message_filters/subscriber.h"
#include <chrono>


namespace lowpass_filter
{

    class lowpass_filter : public nav2_util::LifecycleNode{

        public:
            explicit lowpass_filter (const rclcpp::NodeOptions &options = rclcpp::NodeOptions());


            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::TimerBase::SharedPtr timer2_;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filtering_imu_pub;
            void timerCallback();
            void timerCallback2();
            void Imu_Received(const sensor_msgs::msg::Imu::SharedPtr msg);
            std::unique_ptr<piot_filter_utils::IMUSubscriber> imu_sub;
            sensor_msgs::msg::Imu current_imu;
            sensor_msgs::msg::Imu low_passfilter(const sensor_msgs::msg::Imu msg);
            sensor_msgs::msg::Imu old_imu;
            bool low_pass_start = false;
            float alpha = 0.8;
        private:


        protected:

        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

                    
    };





}
