#include "lowpass_filter.hpp"

using namespace std::chrono_literals;

namespace lowpass_filter{


    lowpass_filter::lowpass_filter(const rclcpp::NodeOptions & options)
    : nav2_util::LifecycleNode("lowpass_filter", "",options)
    {
        RCLCPP_INFO(get_logger(),"lowpass_filter executing");
        filtering_imu_pub = create_publisher<sensor_msgs::msg::Imu>("filtered_imu",1);
    
    }


    void lowpass_filter::timerCallback(){
    current_imu = imu_sub->getIMU();
    RCLCPP_INFO(get_logger(),"imu is %lf",current_imu.linear_acceleration.x);
    sensor_msgs::msg::Imu filterling_imu_data;
    filterling_imu_data = low_passfilter(current_imu);
    filtering_imu_pub->publish(filterling_imu_data);

    }



    void lowpass_filter::Imu_Received(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        current_imu = *msg;

    }



    sensor_msgs::msg::Imu lowpass_filter::low_passfilter(const sensor_msgs::msg::Imu msg)
    {
        sensor_msgs::msg::Imu out_imu_data;

        out_imu_data = msg;
        out_imu_data.linear_acceleration.x = alpha * old_imu.linear_acceleration.x + (1-alpha)* out_imu_data.linear_acceleration.x;
        out_imu_data.linear_acceleration.y = alpha * old_imu.linear_acceleration.y + (1-alpha)* out_imu_data.linear_acceleration.y;
        out_imu_data.linear_acceleration.z = alpha * old_imu.linear_acceleration.z + (1-alpha)* out_imu_data.linear_acceleration.z;

        // out_imu_data.header.stamp = rclcpp::Clock().now();
        old_imu = out_imu_data;
        
        return out_imu_data;

    }

    nav2_util::CallbackReturn
    lowpass_filter::on_activate(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Activating");
        createBond();
        timer_ = this->create_wall_timer(10ms,std::bind(&lowpass_filter::timerCallback,this));
        //timer2_ = this->create_wall_timer(100ms,std::bind(&lowpass_filter::timerCallback2,this));
        return nav2_util::CallbackReturn::SUCCESS;
    }


    nav2_util::CallbackReturn
    lowpass_filter::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating");
        destroyBond();
        return nav2_util::CallbackReturn::SUCCESS;
    }


    nav2_util::CallbackReturn
    lowpass_filter::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
    {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    // Cleanup the helper classes
    return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    lowpass_filter::on_shutdown(const rclcpp_lifecycle::State &)
    {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    lowpass_filter::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    auto node = shared_from_this();

    RCLCPP_INFO(get_logger(), "now");        

    imu_sub = std::make_unique<piot_filter_utils::IMUSubscriber>(shared_from_this());

    return nav2_util::CallbackReturn::SUCCESS;
    }
}
