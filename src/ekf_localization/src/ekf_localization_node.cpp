#include <cmath>
#include <memory>

#include <Eigen/Dense>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"


class EkfLocalizationNode : public rclcpp::Node
{
public:
    EkfLocalizationNode()
    : Node("ekf_localization_node")
    {
        // -------------------------------------------------
        // Parameters
        // -------------------------------------------------

        process_noise_xy_ =
            this->declare_parameter<double>(
                "process_noise_xy",
                0.02
            );

        process_noise_yaw_ =
            this->declare_parameter<double>(
                "process_noise_yaw",
                0.02
            );

        icp_std_xy_ =
            this->declare_parameter<double>(
                "icp_std_xy",
                0.05
            );

        icp_std_yaw_ =
            this->declare_parameter<double>(
                "icp_std_yaw",
                0.03
            );

        initial_std_xy_ =
            this->declare_parameter<double>(
                "initial_std_xy",
                0.05
            );

        initial_std_yaw_ =
            this->declare_parameter<double>(
                "initial_std_yaw",
                0.03
            );


        // -------------------------------------------------
        // State
        //
        // x = [position_x, position_y, yaw]^T
        //
        // Current EKF:
        //
        // Prediction:
        //   /odom -> linear velocity
        //   /imu  -> yaw rate
        //
        // Measurement update:
        //   /icp_pose -> [x, y, yaw] in map frame
        // -------------------------------------------------

        state_.setZero();

        covariance_.setZero();

        covariance_(0, 0) =
            initial_std_xy_ * initial_std_xy_;

        covariance_(1, 1) =
            initial_std_xy_ * initial_std_xy_;

        covariance_(2, 2) =
            initial_std_yaw_ * initial_std_yaw_;


        // -------------------------------------------------
        // Subscribers
        // -------------------------------------------------

        odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                20,
                std::bind(
                    &EkfLocalizationNode::odomCallback,
                    this,
                    std::placeholders::_1
                )
            );

        imu_sub_ =
            this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu",
                50,
                std::bind(
                    &EkfLocalizationNode::imuCallback,
                    this,
                    std::placeholders::_1
                )
            );

        icp_pose_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseStamped
            >(
                "/icp_pose",
                10,
                std::bind(
                    &EkfLocalizationNode::icpPoseCallback,
                    this,
                    std::placeholders::_1
                )
            );


        // -------------------------------------------------
        // Publishers
        // -------------------------------------------------

        ekf_pose_pub_ =
            this->create_publisher<
                geometry_msgs::msg::PoseStamped
            >(
                "/ekf_pose",
                10
            );

        ekf_odom_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>(
                "/ekf_odom",
                10
            );


        RCLCPP_INFO(
            this->get_logger(),
            "EKF localization node started"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "State: [x, y, yaw] | Prediction: Odom + IMU | Update: /icp_pose"
        );
    }


private:
    // =====================================================
    // Utility
    // =====================================================

    double normalizeAngle(double angle) const
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }

        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }

        return angle;
    }


    double quaternionToYaw(
        double x,
        double y,
        double z,
        double w
    ) const
    {
        const double siny_cosp =
            2.0 * (
                w * z +
                x * y
            );

        const double cosy_cosp =
            1.0 -
            2.0 * (
                y * y +
                z * z
            );

        return std::atan2(
            siny_cosp,
            cosy_cosp
        );
    }


    void yawToQuaternion(
        double yaw,
        double & z,
        double & w
    ) const
    {
        z = std::sin(yaw * 0.5);
        w = std::cos(yaw * 0.5);
    }


    // =====================================================
    // IMU
    // =====================================================

    void imuCallback(
        const sensor_msgs::msg::Imu::SharedPtr msg
    )
    {
        latest_yaw_rate_ =
            msg->angular_velocity.z;

        imu_received_ = true;
    }


    // =====================================================
    // ICP Measurement
    //
    // /icp_pose:
    //   map-frame robot pose estimated by Custom ICP
    //
    // z = [x_icp, y_icp, yaw_icp]^T
    // =====================================================

    void icpPoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg
    )
    {
        Eigen::Vector3d measurement;

        measurement(0) =
            msg->pose.position.x;

        measurement(1) =
            msg->pose.position.y;

        measurement(2) =
            quaternionToYaw(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w
            );


        const rclcpp::Time measurement_time(
            msg->header.stamp
        );


        if (
            icp_time_initialized_ &&
            measurement_time <= last_icp_time_
        )
        {
            return;
        }


        last_icp_time_ =
            measurement_time;

        icp_time_initialized_ =
            true;


        // -------------------------------------------------
        // Initial state
        // -------------------------------------------------
        //
        // The first valid ICP pose anchors the EKF state
        // in the map frame.
        // -------------------------------------------------

        if (!state_initialized_)
        {
            state_ =
                measurement;

            state_(2) =
                normalizeAngle(
                    state_(2)
                );

            state_initialized_ =
                true;

            time_initialized_ =
                false;


            RCLCPP_INFO(
                this->get_logger(),
                "EKF initialized from ICP | x: %.4f | y: %.4f | yaw: %.4f",
                state_(0),
                state_(1),
                state_(2)
            );


            return;
        }


        updateWithIcp(
            measurement
        );
    }


    // =====================================================
    // Wheel Odometry
    //
    // /odom timestamps drive the EKF prediction cycle.
    // linear.x is used as translational velocity.
    // IMU angular_velocity.z is used as yaw rate.
    // =====================================================

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg
    )
    {
        if (
            !imu_received_ ||
            !state_initialized_
        )
        {
            return;
        }


        const rclcpp::Time current_time(
            msg->header.stamp
        );


        // The first Odom sample received after ICP
        // initialization sets the prediction time base.
        if (!time_initialized_)
        {
            last_prediction_time_ = current_time;
            time_initialized_ = true;

            publishEstimate(
                msg->header.stamp,
                msg->twist.twist.linear.x,
                latest_yaw_rate_
            );

            return;
        }


        const double dt =
            (current_time - last_prediction_time_).seconds();

        last_prediction_time_ = current_time;


        // Reject invalid or unusually large time gaps.
        if (dt <= 0.0 || dt > 0.5)
        {
            return;
        }


        const double linear_velocity =
            msg->twist.twist.linear.x;

        const double yaw_rate =
            latest_yaw_rate_;


        predict(
            linear_velocity,
            yaw_rate,
            dt
        );


        publishEstimate(
            msg->header.stamp,
            linear_velocity,
            yaw_rate
        );
    }


    // =====================================================
    // EKF Prediction
    //
    // Nonlinear motion model:
    //
    // x'   = x + v * cos(yaw) * dt
    // y'   = y + v * sin(yaw) * dt
    // yaw' = yaw + omega * dt
    //
    // Covariance:
    //
    // P' = F * P * F^T + Q
    //
    // F is the Jacobian of the nonlinear motion model.
    // =====================================================

    void predict(
        double linear_velocity,
        double yaw_rate,
        double dt
    )
    {
        const double yaw =
            state_(2);


        // -------------------------------------------------
        // State prediction
        // -------------------------------------------------

        state_(0) +=
            linear_velocity *
            std::cos(yaw) *
            dt;

        state_(1) +=
            linear_velocity *
            std::sin(yaw) *
            dt;

        state_(2) =
            normalizeAngle(
                state_(2) +
                yaw_rate * dt
            );


        // -------------------------------------------------
        // Motion-model Jacobian
        //
        // F =
        // [ 1  0  -v*sin(yaw)*dt ]
        // [ 0  1   v*cos(yaw)*dt ]
        // [ 0  0          1       ]
        // -------------------------------------------------

        Eigen::Matrix3d F =
            Eigen::Matrix3d::Identity();

        F(0, 2) =
            -linear_velocity *
            std::sin(yaw) *
            dt;

        F(1, 2) =
            linear_velocity *
            std::cos(yaw) *
            dt;


        // -------------------------------------------------
        // Process-noise covariance
        //
        // These values are initial tuning parameters.
        // They are not benchmark results.
        // -------------------------------------------------

        Eigen::Matrix3d Q =
            Eigen::Matrix3d::Zero();

        Q(0, 0) =
            process_noise_xy_ *
            process_noise_xy_ *
            dt;

        Q(1, 1) =
            process_noise_xy_ *
            process_noise_xy_ *
            dt;

        Q(2, 2) =
            process_noise_yaw_ *
            process_noise_yaw_ *
            dt;


        covariance_ =
            F *
            covariance_ *
            F.transpose() +
            Q;
    }


    // =====================================================
    // EKF Measurement Update
    //
    // Measurement:
    //   z = [x_icp, y_icp, yaw_icp]^T
    //
    // Measurement model:
    //   h(x) = x
    //
    // Therefore:
    //   H = I
    //
    // innovation = z - x
    // S          = P + R
    // K          = P * S^-1
    // x          = x + K * innovation
    // =====================================================

    void updateWithIcp(
        const Eigen::Vector3d & measurement
    )
    {
        Eigen::Vector3d innovation =
            measurement - state_;

        innovation(2) =
            normalizeAngle(
                innovation(2)
            );


        Eigen::Matrix3d R =
            Eigen::Matrix3d::Zero();

        R(0, 0) =
            icp_std_xy_ *
            icp_std_xy_;

        R(1, 1) =
            icp_std_xy_ *
            icp_std_xy_;

        R(2, 2) =
            icp_std_yaw_ *
            icp_std_yaw_;


        // H = I, therefore:
        // S = HPH^T + R = P + R
        const Eigen::Matrix3d S =
            covariance_ + R;


        const Eigen::Matrix3d K =
            covariance_ *
            S.inverse();


        state_ =
            state_ +
            K *
            innovation;

        state_(2) =
            normalizeAngle(
                state_(2)
            );


        // Joseph-form covariance update:
        // P = (I-KH)P(I-KH)^T + KRK^T
        // H = I
        const Eigen::Matrix3d I =
            Eigen::Matrix3d::Identity();

        const Eigen::Matrix3d I_K =
            I - K;


        covariance_ =
            I_K *
            covariance_ *
            I_K.transpose() +
            K *
            R *
            K.transpose();


        // RCLCPP_INFO_THROTTLE(
            // this->get_logger(),
            // *this->get_clock(),
            // 2000,
            // "EKF ICP update | innovation: x=%.4f m, y=%.4f m, yaw=%.4f rad",
            // innovation(0),
            // innovation(1),
            // innovation(2)
        // );
    }


    // =====================================================
    // Output
    // =====================================================

    void publishEstimate(
        const builtin_interfaces::msg::Time & stamp,
        double linear_velocity,
        double yaw_rate
    )
    {
        double qz = 0.0;
        double qw = 1.0;

        yawToQuaternion(
            state_(2),
            qz,
            qw
        );


        geometry_msgs::msg::PoseStamped pose_msg;

        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "map";

        pose_msg.pose.position.x = state_(0);
        pose_msg.pose.position.y = state_(1);
        pose_msg.pose.position.z = 0.0;

        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = qz;
        pose_msg.pose.orientation.w = qw;

        ekf_pose_pub_->publish(pose_msg);


        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header = pose_msg.header;
        odom_msg.child_frame_id = "base_footprint";

        odom_msg.pose.pose =
            pose_msg.pose;

        odom_msg.twist.twist.linear.x =
            linear_velocity;

        odom_msg.twist.twist.angular.z =
            yaw_rate;


        // nav_msgs/Odometry uses a 6x6 covariance array.
        // We expose the x, y and yaw diagonal terms.
        odom_msg.pose.covariance[0] =
            covariance_(0, 0);

        odom_msg.pose.covariance[7] =
            covariance_(1, 1);

        odom_msg.pose.covariance[35] =
            covariance_(2, 2);


        ekf_odom_pub_->publish(odom_msg);
    }


    // =====================================================
    // ROS Interfaces
    // =====================================================

    rclcpp::Subscription<
        nav_msgs::msg::Odometry
    >::SharedPtr odom_sub_;

    rclcpp::Subscription<
        sensor_msgs::msg::Imu
    >::SharedPtr imu_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr icp_pose_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr ekf_pose_pub_;

    rclcpp::Publisher<
        nav_msgs::msg::Odometry
    >::SharedPtr ekf_odom_pub_;


    // =====================================================
    // EKF State
    // =====================================================

    Eigen::Vector3d state_;
    Eigen::Matrix3d covariance_;


    // =====================================================
    // Parameters / Latest Sensor State
    // =====================================================

    double process_noise_xy_;
    double process_noise_yaw_;

    double icp_std_xy_;
    double icp_std_yaw_;

    double initial_std_xy_;
    double initial_std_yaw_;

    double latest_yaw_rate_ = 0.0;


    bool imu_received_ = false;
    bool state_initialized_ = false;
    bool time_initialized_ = false;
    bool icp_time_initialized_ = false;


    rclcpp::Time last_prediction_time_{
        0,
        0,
        RCL_ROS_TIME
    };

    rclcpp::Time last_icp_time_{
        0,
        0,
        RCL_ROS_TIME
    };
};


int main(
    int argc,
    char ** argv
)
{
    rclcpp::init(
        argc,
        argv
    );

    rclcpp::spin(
        std::make_shared<EkfLocalizationNode>()
    );

    rclcpp::shutdown();

    return 0;
}