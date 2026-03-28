#include <nortek_nucleus_ros_interface/nortek_nucleus_ros_interface.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace nortek_nucleus::ros_interface {

NortekNucleusRosInterface::NortekNucleusRosInterface(
    const rclcpp::NodeOptions& options)
    : Node("nortek_nucleus_ros_interface", options) {
    declare_ros_parameters();
    create_publishers();
    create_driver();
    setup_connection();
    configure_instrument_settings();
    configure_nucleus();
    start_nucleus_stream();
}

void NortekNucleusRosInterface::declare_ros_parameters() {
    frame_id_ = declare_parameter<std::string>("frame_id");

    declare_parameter<std::string>("connection_params.remote_ip");
    declare_parameter<int>("connection_params.data_remote_port");
    declare_parameter<std::string>("connection_params.password");

    enable_imu_ = declare_parameter<bool>("enable_imu");
    enable_ins_odom_ = declare_parameter<bool>("enable_ins_odom");
    enable_dvl_ = declare_parameter<bool>("enable_dvl");
    enable_pressure_ = declare_parameter<bool>("enable_pressure");
    enable_magnetometer_ = declare_parameter<bool>("enable_magnetometer");
    enable_ins_twist_ = declare_parameter<bool>("enable_ins_twist");
    enable_ins_position_ = declare_parameter<bool>("enable_ins_position");
    enable_ins_pose_ = declare_parameter<bool>("enable_ins_pose");

    declare_parameter<std::string>("imu_data_raw_pub_topic");
    declare_parameter<std::string>("imu_data_pub_topic");
    declare_parameter<std::string>("ins_pub_topic");
    declare_parameter<std::string>("dvl_pub_topic");
    declare_parameter<std::string>("pressure_pub_topic");
    declare_parameter<std::string>("magnetometer_pub_topic");
    declare_parameter<std::string>("ins_twist_pub_topic");
    declare_parameter<std::string>("ins_position_pub_topic");
    declare_parameter<std::string>("ins_pose_pub_topic");

    declare_parameter<int>("imu_settings.freq");

    declare_parameter<int>("ahrs_settings.freq");
    declare_parameter<int>("ahrs_settings.mode");

    declare_parameter<int>("bottom_track_settings.mode");
    declare_parameter<int>("bottom_track_settings.velocity_range");
    declare_parameter<bool>("bottom_track_settings.enable_watertrack");

    declare_parameter<bool>("fast_pressure_settings.enable");
    declare_parameter<int>("fast_pressure_settings.sampling_rate");

    declare_parameter<int>("magnetometer_settings.freq");
    declare_parameter<int>("magnetometer_settings.mode");

    declare_parameter<double>("instrument_settings.rotxy", 0.0);
    declare_parameter<double>("instrument_settings.rotyz", 0.0);
    declare_parameter<double>("instrument_settings.rotxz", 0.0);
}

void NortekNucleusRosInterface::create_publishers() {
    auto qos = rclcpp::QoS(1).reliable();

    if (enable_imu_) {
        imu_data_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            get_parameter("imu_data_raw_pub_topic").as_string(), qos);
        imu_data_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            get_parameter("imu_data_pub_topic").as_string(), qos);
    }

    if (enable_ins_odom_) {
        ins_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            get_parameter("ins_pub_topic").as_string(), qos);
    }

    if (enable_dvl_) {
        dvl_pub_ =
            create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                get_parameter("dvl_pub_topic").as_string(), qos);
    }

    if (enable_pressure_) {
        pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
            get_parameter("pressure_pub_topic").as_string(), qos);
    }

    if (enable_magnetometer_) {
        magnetometer_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
            get_parameter("magnetometer_pub_topic").as_string(), qos);
    }

    if (enable_ins_twist_) {
        ins_twist_pub_ =
            create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                get_parameter("ins_twist_pub_topic").as_string(), qos);
    }

    if (enable_ins_position_) {
        ins_position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            get_parameter("ins_position_pub_topic").as_string(), qos);
    }

    if (enable_ins_pose_) {
        ins_pose_pub_ =
            create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                get_parameter("ins_pose_pub_topic").as_string(), qos);
    }
}

void NortekNucleusRosInterface::create_driver() {
    nucleus_driver_ = std::make_unique<NortekNucleusDriver>(
        io_, [this](NortekNucleusFrame frame) {
            this->nucleus_callback(std::move(frame));
        });
}

void NortekNucleusRosInterface::setup_connection() {
    NortekConnectionParams params;
    params.remote_ip = get_parameter("connection_params.remote_ip").as_string();
    params.data_remote_port = static_cast<uint16_t>(
        get_parameter("connection_params.data_remote_port").as_int());
    params.password = get_parameter("connection_params.password").as_string();

    std::error_code ec = nucleus_driver_->open_tcp_sockets(params);
    if (ec) {
        RCLCPP_ERROR(get_logger(), "Failed to open Nucleus connection: %s",
                     ec.message().c_str());
        return;
    }

    if (!params.password.empty()) {
        ec = nucleus_driver_->enter_password(params);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to authenticate: %s",
                         ec.message().c_str());
            return;
        }
    }

    RCLCPP_INFO(get_logger(), "Nucleus TCP connection opened.");
}

void NortekNucleusRosInterface::configure_instrument_settings() {
    InstrumentSettings inst;
    inst.rotxy = get_parameter("instrument_settings.rotxy").as_double();
    inst.rotyz = get_parameter("instrument_settings.rotyz").as_double();
    inst.rotxz = get_parameter("instrument_settings.rotxz").as_double();

    auto sc = nucleus_driver_->set_instrument_settings(inst);
    if (sc != NucleusStatusCode::Ok) {
        RCLCPP_ERROR(get_logger(),
                     "Failed to set instrument alignment settings (SETINST)");
    } else {
        RCLCPP_INFO(
            get_logger(),
            "Instrument alignment set: ROTXY=%.2f ROTYZ=%.2f ROTXZ=%.2f (deg)",
            inst.rotxy, inst.rotyz, inst.rotxz);
    }
}

void NortekNucleusRosInterface::configure_nucleus() {
    if (enable_imu_) {
        ImuSettings imu_settings;
        imu_settings.freq =
            static_cast<int>(get_parameter("imu_settings.freq").as_int());
        imu_settings.data_stream_settings = NucleusDataStreamSettings::On;
        imu_settings.data_format = DataSeriesId::ImuData;

        auto sc = nucleus_driver_->set_imu_settings(imu_settings);
        if (sc != NucleusStatusCode::Ok) {
            RCLCPP_ERROR(get_logger(), "Failed to configure IMU");
        }

        AhrsSettings ahrs_settings;
        ahrs_settings.freq =
            static_cast<int>(get_parameter("ahrs_settings.freq").as_int());
        ahrs_settings.mode =
            static_cast<AhrsMode>(get_parameter("ahrs_settings.mode").as_int());
        ahrs_settings.data_stream_settings = NucleusDataStreamSettings::On;
        ahrs_settings.data_format = DataSeriesId::AhrsData;

        sc = nucleus_driver_->set_ahrs_settings(ahrs_settings);
        if (sc != NucleusStatusCode::Ok) {
            RCLCPP_ERROR(get_logger(), "Failed to configure AHRS");
        }
    }

    if (enable_dvl_) {
        BottomTrackSettings bt_settings;
        bt_settings.mode = static_cast<BottomTrackMode>(
            get_parameter("bottom_track_settings.mode").as_int());
        bt_settings.velocity_range = static_cast<int>(
            get_parameter("bottom_track_settings.velocity_range").as_int());
        bt_settings.enable_watertrack =
            get_parameter("bottom_track_settings.enable_watertrack").as_bool();
        bt_settings.power_level_user_defined = false;
        bt_settings.power_level = 0;
        bt_settings.data_stream_settings = NucleusDataStreamSettings::On;
        bt_settings.data_format = NucleusDataFormats::BottomTrackBinaryFormat;

        auto sc = nucleus_driver_->set_bottom_track_settings(bt_settings);
        if (sc != NucleusStatusCode::Ok) {
            RCLCPP_ERROR(get_logger(), "Failed to configure bottom track");
        }
    }

    if (enable_pressure_) {
        FastPressureSettings fp_settings;
        fp_settings.enable_fast_pressure =
            get_parameter("fast_pressure_settings.enable").as_bool();
        fp_settings.sampling_rate = static_cast<int>(
            get_parameter("fast_pressure_settings.sampling_rate").as_int());
        fp_settings.data_stream_settings = NucleusDataStreamSettings::On;
        fp_settings.data_format = NucleusDataFormats::FastPressureFormat;

        auto sc = nucleus_driver_->set_fast_pressure_settings(fp_settings);
        if (sc != NucleusStatusCode::Ok) {
            RCLCPP_ERROR(get_logger(), "Failed to configure fast pressure");
        }
    }

    if (enable_magnetometer_) {
        MagnetometerSettings mag_settings;
        mag_settings.freq = static_cast<int>(
            get_parameter("magnetometer_settings.freq").as_int());
        mag_settings.mode = static_cast<MagnetometerMethod>(
            get_parameter("magnetometer_settings.mode").as_int());
        mag_settings.data_stream_settings = NucleusDataStreamSettings::On;
        mag_settings.data_format = DataSeriesId::MagnometerData;

        auto sc = nucleus_driver_->set_magnetometer_settings(mag_settings);
        if (sc != NucleusStatusCode::Ok) {
            RCLCPP_ERROR(get_logger(), "Failed to configure magnetometer");
        }
    }

    RCLCPP_INFO(get_logger(), "Nucleus configured.");
}

void NortekNucleusRosInterface::start_nucleus_stream() {
    nucleus_driver_->start_read();

    auto sc = nucleus_driver_->start_nucleus();
    if (sc != NucleusStatusCode::Ok) {
        RCLCPP_ERROR(get_logger(), "Failed to start Nucleus");
        return;
    }

    io_thread_ = std::thread([this]() { io_.run(); });
    RCLCPP_INFO(get_logger(), "Nucleus started.");
}

void NortekNucleusRosInterface::nucleus_callback(NortekNucleusFrame frame) {
    std::visit(
        [this](auto&& data) {
            using T = std::decay_t<decltype(data)>;
            if constexpr (std::is_same_v<T, ImuData>) {
                handle_imu(data);
            } else if constexpr (std::is_same_v<T, AhrsDataV2>) {
                handle_ahrs(data);
            } else if constexpr (std::is_same_v<T, InsDataV2>) {
                handle_ins(data);
            } else if constexpr (std::is_same_v<T, BottomTrackData>) {
                handle_bottom_track(data);
            } else if constexpr (std::is_same_v<T, FastPressureData>) {
                handle_pressure(data);
            } else if constexpr (std::is_same_v<T, MagnetoMeterData>) {
                handle_magnetometer(data);
            }
        },
        frame);
}

void NortekNucleusRosInterface::handle_imu(const ImuData& data) {
    if (!imu_data_raw_pub_)
        return;

    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = this->get_clock()->now();

    msg->angular_velocity.x = data.gyro_x;
    msg->angular_velocity.y = data.gyro_y;
    msg->angular_velocity.z = data.gyro_z;

    msg->linear_acceleration.x = data.accelerometer_x;
    msg->linear_acceleration.y = data.accelerometer_y;
    msg->linear_acceleration.z = data.accelerometer_z;

    // No orientation on data_raw per REP-145
    msg->orientation_covariance[0] = -1.0;

    imu_data_raw_pub_->publish(std::move(msg));
}

void NortekNucleusRosInterface::handle_ahrs(const AhrsDataV2& data) {
    // Cache orientation for use in INS odometry
    latest_ahrs_orientation_.w = data.data_quaternion_w;
    latest_ahrs_orientation_.x = data.data_quaternion_x;
    latest_ahrs_orientation_.y = data.data_quaternion_y;
    latest_ahrs_orientation_.z = data.data_quaternion_z;
    ahrs_received_ = true;

    if (!imu_data_pub_)
        return;

    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = this->get_clock()->now();

    msg->orientation = latest_ahrs_orientation_;

    imu_data_pub_->publish(std::move(msg));
}

void NortekNucleusRosInterface::handle_ins(const InsDataV2& data) {
    auto stamp = this->get_clock()->now();

    if (ins_odom_pub_) {
        auto msg = std::make_unique<nav_msgs::msg::Odometry>();
        msg->header.frame_id = frame_id_ + "_odom";
        msg->header.stamp = stamp;
        msg->child_frame_id = frame_id_;

        msg->pose.pose.position.x = data.position_ned_x;
        msg->pose.pose.position.y = data.position_ned_y;
        msg->pose.pose.position.z = data.position_ned_z;

        if (ahrs_received_) {
            msg->pose.pose.orientation = latest_ahrs_orientation_;
        } else {
            msg->pose.pose.orientation.w = 1.0;
        }

        msg->twist.twist.linear.x = data.velocity_body_x;
        msg->twist.twist.linear.y = data.velocity_body_y;
        msg->twist.twist.linear.z = data.velocity_body_z;

        msg->twist.twist.angular.x = data.turn_rate_x;
        msg->twist.twist.angular.y = data.turn_rate_y;
        msg->twist.twist.angular.z = data.turn_rate_z;

        ins_odom_pub_->publish(std::move(msg));
    }

    if (ins_twist_pub_) {
        auto twist_msg =
            std::make_unique<geometry_msgs::msg::TwistWithCovarianceStamped>();
        twist_msg->header.frame_id = frame_id_;
        twist_msg->header.stamp = stamp;

        twist_msg->twist.twist.linear.x = data.velocity_body_x;
        twist_msg->twist.twist.linear.y = data.velocity_body_y;
        twist_msg->twist.twist.linear.z = data.velocity_body_z;

        twist_msg->twist.twist.angular.x = data.turn_rate_x;
        twist_msg->twist.twist.angular.y = data.turn_rate_y;
        twist_msg->twist.twist.angular.z = data.turn_rate_z;

        twist_msg->twist.covariance[0] = -1.0;

        ins_twist_pub_->publish(std::move(twist_msg));
    }

    if (ins_position_pub_) {
        auto position_msg =
            std::make_unique<geometry_msgs::msg::PointStamped>();
        position_msg->header.frame_id = frame_id_ + "_odom";
        position_msg->header.stamp = stamp;

        position_msg->point.x = data.position_ned_x;
        position_msg->point.y = data.position_ned_y;
        position_msg->point.z = data.position_ned_z;

        ins_position_pub_->publish(std::move(position_msg));
    }

    if (ins_pose_pub_) {
        auto pose_msg =
            std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
        pose_msg->header.frame_id = frame_id_ + "_odom";
        pose_msg->header.stamp = stamp;

        pose_msg->pose.pose.position.x = data.position_ned_x;
        pose_msg->pose.pose.position.y = data.position_ned_y;
        pose_msg->pose.pose.position.z = data.position_ned_z;

        if (ahrs_received_) {
            pose_msg->pose.pose.orientation = latest_ahrs_orientation_;
        } else {
            pose_msg->pose.pose.orientation.w = 1.0;
        }

        ins_pose_pub_->publish(std::move(pose_msg));
    }
}

void NortekNucleusRosInterface::handle_bottom_track(
    const BottomTrackData& data) {
    if (!dvl_pub_)
        return;

    auto msg =
        std::make_unique<geometry_msgs::msg::TwistWithCovarianceStamped>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = this->get_clock()->now();

    msg->twist.twist.linear.x = data.velocity_x;
    msg->twist.twist.linear.y = data.velocity_y;
    msg->twist.twist.linear.z = data.velocity_z;

    // Fill diagonal of covariance with squared uncertainties
    msg->twist.covariance[0] =
        static_cast<double>(data.uncertainty_x * data.uncertainty_x);
    msg->twist.covariance[7] =
        static_cast<double>(data.uncertainty_y * data.uncertainty_y);
    msg->twist.covariance[14] =
        static_cast<double>(data.uncertainty_z * data.uncertainty_z);

    dvl_pub_->publish(std::move(msg));
}

void NortekNucleusRosInterface::handle_pressure(const FastPressureData& data) {
    if (!pressure_pub_)
        return;

    auto msg = std::make_unique<sensor_msgs::msg::FluidPressure>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = this->get_clock()->now();

    // Convert dBar to Pascal (1 dBar = 10000 Pa)
    msg->fluid_pressure = static_cast<double>(data.fast_pressure) * 10000.0;

    pressure_pub_->publish(std::move(msg));
}

void NortekNucleusRosInterface::handle_magnetometer(
    const MagnetoMeterData& data) {
    if (!magnetometer_pub_)
        return;

    auto msg = std::make_unique<sensor_msgs::msg::MagneticField>();
    msg->header.frame_id = frame_id_;
    msg->header.stamp = this->get_clock()->now();

    // Convert microtesla to tesla (sensor_msgs expects Tesla)
    msg->magnetic_field.x = static_cast<double>(data.magnetometer_x) * 1e-6;
    msg->magnetic_field.y = static_cast<double>(data.magnetometer_y) * 1e-6;
    msg->magnetic_field.z = static_cast<double>(data.magnetometer_z) * 1e-6;

    magnetometer_pub_->publish(std::move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(NortekNucleusRosInterface)

}  // namespace nortek_nucleus::ros_interface
