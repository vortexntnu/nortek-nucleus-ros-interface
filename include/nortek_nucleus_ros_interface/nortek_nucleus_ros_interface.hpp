#ifndef NORTEK_NUCLEUS_ROS_INTERFACE__NORTEK_NUCLEUS_ROS_INTERFACE_HPP_
#define NORTEK_NUCLEUS_ROS_INTERFACE__NORTEK_NUCLEUS_ROS_INTERFACE_HPP_

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <nortek_nucleus_driver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <thread>

namespace nortek_nucleus::ros_interface {

class NortekNucleusRosInterface : public rclcpp::Node {
   public:
    explicit NortekNucleusRosInterface(const rclcpp::NodeOptions& options);

    ~NortekNucleusRosInterface() {
        nucleus_driver_->stop_nucleus();
        io_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

   private:
    void nucleus_callback(NortekNucleusFrame frame);

    void handle_imu(const ImuData& data);
    void handle_ahrs(const AhrsDataV2& data);
    void handle_bottom_track(const BottomTrackData& data);
    void handle_pressure(const FastPressureData& data);
    void handle_magnetometer(const MagnetoMeterData& data);

    void declare_ros_parameters();
    void create_publishers();
    void create_driver();
    void setup_connection();
    void configure_nucleus();
    void start_nucleus_stream();

    asio::io_context io_;
    std::thread io_thread_;

    std::unique_ptr<NortekNucleusDriver> nucleus_driver_;

    // Publishers — only created when enabled via parameters
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        dvl_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr
        magnetometer_pub_;

    // Enable flags
    bool enable_imu_{};
    bool enable_dvl_{};
    bool enable_pressure_{};
    bool enable_magnetometer_{};

    std::string frame_id_;
};

}  // namespace nortek_nucleus::ros_interface

#endif  // NORTEK_NUCLEUS_ROS_INTERFACE__NORTEK_NUCLEUS_ROS_INTERFACE_HPP_
