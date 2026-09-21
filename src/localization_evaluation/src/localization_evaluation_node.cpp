#include <memory>
#include <cmath>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"


class LocalizationEvaluationNode : public rclcpp::Node
{
public:

    LocalizationEvaluationNode()
    : Node("localization_evaluation_node")
    {
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(
                this->get_clock()
            );

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_
            );

        ground_truth_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/ground_truth_pose",
                10,
                std::bind(
                    &LocalizationEvaluationNode::groundTruthCallback,
                    this,
                    std::placeholders::_1
                )
            );

        odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(
                    &LocalizationEvaluationNode::odomCallback,
                    this,
                    std::placeholders::_1
                )
            );

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &LocalizationEvaluationNode::evaluate,
                    this
                )
            );

        RCLCPP_INFO(
            this->get_logger(),
            "Localization Evaluation Node started"
        );
    }


private:

    struct Pose2D
    {
        double x;
        double y;
        double yaw;
    };


    double quaternionToYaw(
        const geometry_msgs::msg::Quaternion& q)
    {
        const double siny_cosp =
            2.0 * (
                q.w * q.z +
                q.x * q.y
            );

        const double cosy_cosp =
            1.0 -
            2.0 * (
                q.y * q.y +
                q.z * q.z
            );

        return std::atan2(
            siny_cosp,
            cosy_cosp
        );
    }


    double normalizeAngle(
        const double angle)
    {
        return std::atan2(
            std::sin(angle),
            std::cos(angle)
        );
    }


    void groundTruthCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        ground_truth_.x =
            msg->pose.position.x;

        ground_truth_.y =
            msg->pose.position.y;

        ground_truth_.yaw =
            quaternionToYaw(
                msg->pose.orientation
            );

        ground_truth_received_ =
            true;
    }


    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_.x =
            msg->pose.pose.position.x;

        odom_.y =
            msg->pose.pose.position.y;

        odom_.yaw =
            quaternionToYaw(
                msg->pose.pose.orientation
            );

        odom_received_ =
            true;
    }


    void evaluate()
    {
        if (!ground_truth_received_ ||
            !odom_received_)
        {
            return;
        }

        Pose2D icp_pose;

        try
        {
            // ICP 보정이 반영된 최종 로봇 pose
            // map -> base_footprint
            const auto transform =
                tf_buffer_->lookupTransform(
                    "map",
                    "base_footprint",
                    tf2::TimePointZero
                );

            icp_pose.x =
                transform.transform.translation.x;

            icp_pose.y =
                transform.transform.translation.y;

            icp_pose.yaw =
                quaternionToYaw(
                    transform.transform.rotation
                );
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for map -> base_footprint TF: %s",
                ex.what()
            );

            return;
        }


        // -------------------------------------------------
        // Wheel Odometry error
        // -------------------------------------------------

        const double odom_dx =
            odom_.x - ground_truth_.x;

        const double odom_dy =
            odom_.y - ground_truth_.y;

        const double odom_position_error =
            std::hypot(
                odom_dx,
                odom_dy
            );

        const double odom_yaw_error =
            std::abs(
                normalizeAngle(
                    odom_.yaw -
                    ground_truth_.yaw
                )
            );


        // -------------------------------------------------
        // ICP corrected pose error
        // -------------------------------------------------

        const double icp_dx =
            icp_pose.x - ground_truth_.x;

        const double icp_dy =
            icp_pose.y - ground_truth_.y;

        const double icp_position_error =
            std::hypot(
                icp_dx,
                icp_dy
            );

        const double icp_yaw_error =
            std::abs(
                normalizeAngle(
                    icp_pose.yaw -
                    ground_truth_.yaw
                )
            );


        // -------------------------------------------------
        // Accumulate squared error for RMSE
        // -------------------------------------------------

        ++sample_count_;

        odom_position_squared_sum_ +=
            odom_position_error *
            odom_position_error;

        icp_position_squared_sum_ +=
            icp_position_error *
            icp_position_error;

        odom_yaw_squared_sum_ +=
            odom_yaw_error *
            odom_yaw_error;

        icp_yaw_squared_sum_ +=
            icp_yaw_error *
            icp_yaw_error;


        const double odom_position_rmse =
            std::sqrt(
                odom_position_squared_sum_ /
                sample_count_
            );

        const double icp_position_rmse =
            std::sqrt(
                icp_position_squared_sum_ /
                sample_count_
            );

        const double odom_yaw_rmse =
            std::sqrt(
                odom_yaw_squared_sum_ /
                sample_count_
            );

        const double icp_yaw_rmse =
            std::sqrt(
                icp_yaw_squared_sum_ /
                sample_count_
            );


        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Error | Odom: %.4f m, %.4f rad | ICP: %.4f m, %.4f rad | RMSE | Odom: %.4f m, %.4f rad | ICP: %.4f m, %.4f rad | N: %zu",
            odom_position_error,
            odom_yaw_error,
            icp_position_error,
            icp_yaw_error,
            odom_position_rmse,
            odom_yaw_rmse,
            icp_position_rmse,
            icp_yaw_rmse,
            sample_count_
        );
    }


    std::unique_ptr<
        tf2_ros::Buffer
    > tf_buffer_;

    std::shared_ptr<
        tf2_ros::TransformListener
    > tf_listener_;


    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr ground_truth_sub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry
    >::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr timer_;


    Pose2D ground_truth_ {};
    Pose2D odom_ {};

    bool ground_truth_received_ =
        false;

    bool odom_received_ =
        false;


    std::size_t sample_count_ =
        0;

    double odom_position_squared_sum_ =
        0.0;

    double icp_position_squared_sum_ =
        0.0;

    double odom_yaw_squared_sum_ =
        0.0;

    double icp_yaw_squared_sum_ =
        0.0;
};


int main(
    int argc,
    char* argv[])
{
    rclcpp::init(
        argc,
        argv
    );

    rclcpp::spin(
        std::make_shared<LocalizationEvaluationNode>()
    );

    rclcpp::shutdown();

    return 0;
}
