#include <memory>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class GroundTruthNode : public rclcpp::Node
{
public:

    GroundTruthNode()
    : Node("ground_truth_node")
    {
        ground_truth_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/ground_truth_pose",
                10
            );

        auto qos =
            rclcpp::QoS(
                rclcpp::KeepLast(10)
            )
            .best_effort();

        ground_truth_sub_ =
            this->create_subscription<tf2_msgs::msg::TFMessage>(
                "/ground_truth/poses",
                qos,
                std::bind(
                    &GroundTruthNode::groundTruthCallback,
                    this,
                    std::placeholders::_1
                )
            );

        RCLCPP_INFO(
            this->get_logger(),
            "Ground Truth Node started"
        );
    }


private:

    double normalizeAngle(
        const double angle)
    {
        return std::atan2(
            std::sin(angle),
            std::cos(angle)
        );
    }


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


    void groundTruthCallback(
        const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform : msg->transforms)
        {
            if (transform.child_frame_id != "icp_ekf_amr")
            {
                continue;
            }

            const double world_x =
                transform.transform.translation.x;

            const double world_y =
                transform.transform.translation.y;

            const double world_yaw =
                quaternionToYaw(
                    transform.transform.rotation
                );

            if (!initial_pose_received_)
            {
                initial_x_ =
                    world_x;

                initial_y_ =
                    world_y;

                initial_yaw_ =
                    world_yaw;

                initial_pose_received_ =
                    true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Initial Ground Truth | x: %.5f m | y: %.5f m | yaw: %.5f rad",
                    initial_x_,
                    initial_y_,
                    initial_yaw_
                );
            }

            const double dx_world =
                world_x - initial_x_;

            const double dy_world =
                world_y - initial_y_;

            const double cos_initial =
                std::cos(initial_yaw_);

            const double sin_initial =
                std::sin(initial_yaw_);

            const double relative_x =
                cos_initial * dx_world +
                sin_initial * dy_world;

            const double relative_y =
                -sin_initial * dx_world +
                cos_initial * dy_world;

            const double relative_yaw =
                normalizeAngle(
                    world_yaw - initial_yaw_
                );

            geometry_msgs::msg::PoseStamped output;

            output.header.stamp =
                this->get_clock()->now();

            output.header.frame_id =
                "map";

            output.pose.position.x =
                relative_x;

            output.pose.position.y =
                relative_y;

            output.pose.position.z =
                0.0;

            const double half_yaw =
                relative_yaw * 0.5;

            output.pose.orientation.x =
                0.0;

            output.pose.orientation.y =
                0.0;

            output.pose.orientation.z =
                std::sin(
                    half_yaw
                );

            output.pose.orientation.w =
                std::cos(
                    half_yaw
                );

            ground_truth_pub_->publish(
                output
            );

            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     2000,
            //     "Ground Truth Pose | x: %.5f m | y: %.5f m | yaw: %.5f rad",
            //     relative_x,
            //     relative_y,
            //     relative_yaw
            // );

            break;
        }
    }


    rclcpp::Subscription<
        tf2_msgs::msg::TFMessage
    >::SharedPtr ground_truth_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr ground_truth_pub_;

    bool initial_pose_received_ =
        false;

    double initial_x_ =
        0.0;

    double initial_y_ =
        0.0;

    double initial_yaw_ =
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
        std::make_shared<GroundTruthNode>()
    );

    rclcpp::shutdown();

    return 0;
}
