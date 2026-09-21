#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


class LocalizationBenchmarkRunner : public rclcpp::Node
{
public:
    using NavigateToPose =
        nav2_msgs::action::NavigateToPose;

    using GoalHandleNavigateToPose =
        rclcpp_action::ClientGoalHandle<NavigateToPose>;


    LocalizationBenchmarkRunner()
    : Node("localization_benchmark_runner")
    {
        // -------------------------------------------------
        // Fixed benchmark route
        // -------------------------------------------------
        //
        // Start is the robot's initial pose.
        //
        // The following map-frame waypoints were selected
        // directly from RViz.
        // -------------------------------------------------

        waypoints_ = {
            {3.39557,  -4.12722, 0.0},
            {5.61326, 4.40286, 0.0},
            {7.68631, -1.58174, 0.0}
        };


        // -------------------------------------------------
        // TF
        // -------------------------------------------------

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(
                this->get_clock()
            );

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_
            );


        // -------------------------------------------------
        // Ground Truth / Wheel Odometry
        // -------------------------------------------------

        ground_truth_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseStamped
            >(
                "/ground_truth_pose",
                10,
                std::bind(
                    &LocalizationBenchmarkRunner::groundTruthCallback,
                    this,
                    std::placeholders::_1
                )
            );

        odom_sub_ =
            this->create_subscription<
                nav_msgs::msg::Odometry
            >(
                "/odom",
                20,
                std::bind(
                    &LocalizationBenchmarkRunner::odomCallback,
                    this,
                    std::placeholders::_1
                )
            );


        // -------------------------------------------------
        // Nav2 Action Client
        // -------------------------------------------------

        navigate_client_ =
            rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose"
            );

        bt_navigator_state_client_ =
            this->create_client<
                lifecycle_msgs::srv::GetState
            >(
                "/bt_navigator/get_state"
            );


        // -------------------------------------------------
        // Timers
        // -------------------------------------------------
        //
        // ready_timer_:
        //   Wait until Ground Truth, Odom and Nav2 are ready.
        //
        // evaluation_timer_:
        //   Record localization error continuously during
        //   the benchmark run.
        // -------------------------------------------------

        ready_timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(200),
                std::bind(
                    &LocalizationBenchmarkRunner::tryStartBenchmark,
                    this
                )
            );

        evaluation_timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &LocalizationBenchmarkRunner::evaluate,
                    this
                )
            );


        // Debug status logs are intentionally disabled.
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Localization Benchmark Runner started"
        // );

        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Route: START -> P1 -> P2 -> P3"
        // );
    }


private:
    // =====================================================
    // Data Structures
    // =====================================================

    struct Waypoint
    {
        double x;
        double y;
        double yaw;
    };


    struct Pose2D
    {
        double x;
        double y;
        double yaw;
    };


    struct EvaluationSample
    {
        Pose2D ground_truth;
        Pose2D odom;
        Pose2D icp;

        double odom_position_error;
        double odom_yaw_error;

        double icp_position_error;
        double icp_yaw_error;
    };


    // =====================================================
    // Angle Utility
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
            2.0 * (w * z + x * y);

        const double cosy_cosp =
            1.0 - 2.0 * (y * y + z * z);

        return std::atan2(
            siny_cosp,
            cosy_cosp
        );
    }


    // =====================================================
    // Input Callbacks
    // =====================================================

    void groundTruthCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg
    )
    {
        latest_ground_truth_ = *msg;
        ground_truth_received_ = true;
    }


    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg
    )
    {
        latest_odom_ = *msg;
        odom_received_ = true;
    }


    // =====================================================
    // Nav2 Lifecycle State
    // =====================================================

    void requestBtNavigatorState()
    {
        if (bt_navigator_active_ || bt_navigator_state_request_pending_)
        {
            return;
        }


        if (!bt_navigator_state_client_->service_is_ready())
        {
            return;
        }


        auto request =
            std::make_shared<
                lifecycle_msgs::srv::GetState::Request
            >();


        bt_navigator_state_request_pending_ = true;


        bt_navigator_state_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    lifecycle_msgs::srv::GetState
                >::SharedFuture future
            )
            {
                bt_navigator_state_request_pending_ = false;

                const auto response =
                    future.get();

                bt_navigator_active_ =
                    response->current_state.id ==
                    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
            }
        );
    }


    // =====================================================
    // Benchmark Start
    // =====================================================

    void tryStartBenchmark()
    {
        if (benchmark_started_)
        {
            return;
        }


        if (!ground_truth_received_ || !odom_received_)
        {
            // Debug status log disabled.
            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     2000,
            //     "Waiting for Ground Truth and Odometry..."
            // );

            return;
        }


        // The NavigateToPose action server can exist before
        // bt_navigator reaches the ACTIVE lifecycle state.
        // Do not start the benchmark until Nav2 is truly active.
        if (!bt_navigator_active_)
        {
            requestBtNavigatorState();

            // Debug status log disabled.
            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     2000,
            //     "Waiting for bt_navigator ACTIVE state..."
            // );

            return;
        }


        if (!navigate_client_->action_server_is_ready())
        {
            // Debug status log disabled.
            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     2000,
            //     "Waiting for Nav2 /navigate_to_pose action server..."
            // );

            return;
        }


        EvaluationSample start_sample;

        if (!getCurrentSample(start_sample))
        {
            // Debug status log disabled.
            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     2000,
            //     "Waiting for map -> base_footprint TF..."
            // );

            return;
        }


        openCsv();

        resetMetrics();

        benchmark_started_ = true;
        benchmark_active_ = true;

        writeCsvRow(
            "START",
            0,
            start_sample
        );

        logSnapshot(
            "START",
            start_sample
        );


        RCLCPP_INFO(
            this->get_logger(),
            "=================================================="
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Benchmark measurement started"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "=================================================="
        );


        sendWaypointGoal(0);
    }


    // =====================================================
    // Nav2 Waypoint Goal
    // =====================================================

    void sendWaypointGoal(std::size_t waypoint_index)
    {
        current_waypoint_number_ =
            waypoint_index + 1;

        if (waypoint_index >= waypoints_.size())
        {
            finishBenchmark();
            return;
        }


        const auto & waypoint =
            waypoints_[waypoint_index];


        NavigateToPose::Goal goal;

        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();

        goal.pose.pose.position.x = waypoint.x;
        goal.pose.pose.position.y = waypoint.y;
        goal.pose.pose.position.z = 0.0;

        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;
        goal.pose.pose.orientation.z =
            std::sin(waypoint.yaw / 2.0);

        goal.pose.pose.orientation.w =
            std::cos(waypoint.yaw / 2.0);


        RCLCPP_INFO(
            this->get_logger(),
            "Sending Point %zu | x: %.5f | y: %.5f | yaw: %.3f",
            waypoint_index + 1,
            waypoint.x,
            waypoint.y,
            waypoint.yaw
        );


        auto send_goal_options =
            rclcpp_action::Client<
                NavigateToPose
            >::SendGoalOptions();


        send_goal_options.goal_response_callback =
            [this, waypoint_index](
                GoalHandleNavigateToPose::SharedPtr goal_handle
            )
            {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Point %zu goal was rejected",
                        waypoint_index + 1
                    );

                    abortBenchmark();
                    return;
                }


                RCLCPP_INFO(
                    this->get_logger(),
                    "Point %zu goal accepted",
                    waypoint_index + 1
                );
            };


        send_goal_options.result_callback =
            [this, waypoint_index](
                const GoalHandleNavigateToPose::WrappedResult & result
            )
            {
                if (
                    result.code !=
                    rclcpp_action::ResultCode::SUCCEEDED
                )
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Point %zu navigation failed. Result code: %d",
                        waypoint_index + 1,
                        static_cast<int>(result.code)
                    );

                    abortBenchmark();
                    return;
                }


                EvaluationSample sample;

                if (!getCurrentSample(sample))
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Point %zu reached, but localization sample could not be captured",
                        waypoint_index + 1
                    );

                    abortBenchmark();
                    return;
                }


                const std::string label =
                    "P" +
                    std::to_string(
                        waypoint_index + 1
                    );


                writeCsvRow(
                    label,
                    waypoint_index + 1,
                    sample
                );

                logSnapshot(
                    label,
                    sample
                );


                if (
                    waypoint_index + 1 <
                    waypoints_.size()
                )
                {
                    sendWaypointGoal(
                        waypoint_index + 1
                    );
                }
                else
                {
                    finishBenchmark();
                }
            };


        navigate_client_->async_send_goal(
            goal,
            send_goal_options
        );
    }


    // =====================================================
    // Localization Evaluation
    // =====================================================

    bool getCurrentSample(
        EvaluationSample & sample
    )
    {
        if (!ground_truth_received_ || !odom_received_)
        {
            return false;
        }


        sample.ground_truth.x =
            latest_ground_truth_.pose.position.x;

        sample.ground_truth.y =
            latest_ground_truth_.pose.position.y;

        sample.ground_truth.yaw =
            quaternionToYaw(
                latest_ground_truth_.pose.orientation.x,
                latest_ground_truth_.pose.orientation.y,
                latest_ground_truth_.pose.orientation.z,
                latest_ground_truth_.pose.orientation.w
            );


        sample.odom.x =
            latest_odom_.pose.pose.position.x;

        sample.odom.y =
            latest_odom_.pose.pose.position.y;

        sample.odom.yaw =
            quaternionToYaw(
                latest_odom_.pose.pose.orientation.x,
                latest_odom_.pose.pose.orientation.y,
                latest_odom_.pose.pose.orientation.z,
                latest_odom_.pose.pose.orientation.w
            );


        try
        {
            const auto transform =
                tf_buffer_->lookupTransform(
                    "map",
                    "base_footprint",
                    tf2::TimePointZero
                );


            sample.icp.x =
                transform.transform.translation.x;

            sample.icp.y =
                transform.transform.translation.y;

            sample.icp.yaw =
                quaternionToYaw(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                );
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "TF unavailable: %s",
                ex.what()
            );

            return false;
        }


        sample.odom_position_error =
            std::hypot(
                sample.odom.x -
                    sample.ground_truth.x,
                sample.odom.y -
                    sample.ground_truth.y
            );

        sample.odom_yaw_error =
            std::abs(
                normalizeAngle(
                    sample.odom.yaw -
                    sample.ground_truth.yaw
                )
            );


        sample.icp_position_error =
            std::hypot(
                sample.icp.x -
                    sample.ground_truth.x,
                sample.icp.y -
                    sample.ground_truth.y
            );

        sample.icp_yaw_error =
            std::abs(
                normalizeAngle(
                    sample.icp.yaw -
                    sample.ground_truth.yaw
                )
            );


        return true;
    }


    void evaluate()
    {
        if (!benchmark_active_)
        {
            return;
        }


        EvaluationSample sample;

        if (!getCurrentSample(sample))
        {
            return;
        }


        odom_position_squared_sum_ +=
            sample.odom_position_error *
            sample.odom_position_error;

        odom_yaw_squared_sum_ +=
            sample.odom_yaw_error *
            sample.odom_yaw_error;

        icp_position_squared_sum_ +=
            sample.icp_position_error *
            sample.icp_position_error;

        icp_yaw_squared_sum_ +=
            sample.icp_yaw_error *
            sample.icp_yaw_error;


        if (
            sample.odom_position_error >
            odom_max_position_error_
        )
        {
            odom_max_position_error_ =
                sample.odom_position_error;
        }


        if (
            sample.icp_position_error >
            icp_max_position_error_
        )
        {
            icp_max_position_error_ =
                sample.icp_position_error;
        }


        ++sample_count_;


        writeCsvRow(
            "SAMPLE",
            current_waypoint_number_,
            sample
        );
    }


    // =====================================================
    // Waypoint Snapshot
    // =====================================================

    void logSnapshot(
        const std::string & label,
        const EvaluationSample & sample
    )
    {
        RCLCPP_INFO(
            this->get_logger(),
            "--------------------------------------------------"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "%s reached",
            label.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Ground Truth | x: %.4f | y: %.4f | yaw: %.4f",
            sample.ground_truth.x,
            sample.ground_truth.y,
            sample.ground_truth.yaw
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Odom         | x: %.4f | y: %.4f | yaw: %.4f | error: %.4f m / %.4f rad",
            sample.odom.x,
            sample.odom.y,
            sample.odom.yaw,
            sample.odom_position_error,
            sample.odom_yaw_error
        );

        RCLCPP_INFO(
            this->get_logger(),
            "ICP          | x: %.4f | y: %.4f | yaw: %.4f | error: %.4f m / %.4f rad",
            sample.icp.x,
            sample.icp.y,
            sample.icp.yaw,
            sample.icp_position_error,
            sample.icp_yaw_error
        );

        RCLCPP_INFO(
            this->get_logger(),
            "--------------------------------------------------"
        );
    }


    // =====================================================
    // Final Result
    // =====================================================

    void finishBenchmark()
    {
        benchmark_active_ = false;


        if (sample_count_ == 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Benchmark finished without evaluation samples"
            );

            closeCsv();
            return;
        }


        const double count =
            static_cast<double>(
                sample_count_
            );


        const double odom_position_rmse =
            std::sqrt(
                odom_position_squared_sum_ /
                count
            );

        const double odom_yaw_rmse =
            std::sqrt(
                odom_yaw_squared_sum_ /
                count
            );

        const double icp_position_rmse =
            std::sqrt(
                icp_position_squared_sum_ /
                count
            );

        const double icp_yaw_rmse =
            std::sqrt(
                icp_yaw_squared_sum_ /
                count
            );


        RCLCPP_INFO(
            this->get_logger(),
            "=================================================="
        );

        RCLCPP_INFO(
            this->get_logger(),
            "BENCHMARK COMPLETE"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Samples: %zu",
            sample_count_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Odom | Position RMSE: %.4f m | Yaw RMSE: %.4f rad | Max Position Error: %.4f m",
            odom_position_rmse,
            odom_yaw_rmse,
            odom_max_position_error_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "ICP  | Position RMSE: %.4f m | Yaw RMSE: %.4f rad | Max Position Error: %.4f m",
            icp_position_rmse,
            icp_yaw_rmse,
            icp_max_position_error_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "CSV: %s",
            csv_path_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "=================================================="
        );


        closeCsv();
    }


    void abortBenchmark()
    {
        benchmark_active_ = false;

        RCLCPP_ERROR(
            this->get_logger(),
            "Benchmark aborted"
        );

        closeCsv();
    }


    // =====================================================
    // CSV
    // =====================================================

    void openCsv()
    {
        const char * home =
            std::getenv("HOME");


        std::filesystem::path result_directory;

        if (home != nullptr)
        {
            result_directory =
                std::filesystem::path(home) /
                "ros2_icp_ekf_localization" /
                "results";
        }
        else
        {
            result_directory =
                std::filesystem::path("/tmp") /
                "ros2_icp_ekf_localization_results";
        }


        std::filesystem::create_directories(
            result_directory
        );


        const auto timestamp =
            std::chrono::system_clock::now()
                .time_since_epoch()
                .count();


        csv_path_ =
            (
                result_directory /
                (
                    "localization_benchmark_" +
                    std::to_string(timestamp) +
                    ".csv"
                )
            ).string();


        csv_file_.open(
            csv_path_,
            std::ios::out
        );


        csv_file_
            << "sim_time,event,waypoint,"
            << "gt_x,gt_y,gt_yaw,"
            << "odom_x,odom_y,odom_yaw,"
            << "icp_x,icp_y,icp_yaw,"
            << "odom_position_error,odom_yaw_error,"
            << "icp_position_error,icp_yaw_error\n";


        RCLCPP_INFO(
            this->get_logger(),
            "CSV recording: %s",
            csv_path_.c_str()
        );
    }


    void writeCsvRow(
        const std::string & event,
        std::size_t waypoint,
        const EvaluationSample & sample
    )
    {
        if (!csv_file_.is_open())
        {
            return;
        }


        csv_file_
            << std::fixed
            << std::setprecision(6)
            << this->now().seconds() << ","
            << event << ","
            << waypoint << ","

            << sample.ground_truth.x << ","
            << sample.ground_truth.y << ","
            << sample.ground_truth.yaw << ","

            << sample.odom.x << ","
            << sample.odom.y << ","
            << sample.odom.yaw << ","

            << sample.icp.x << ","
            << sample.icp.y << ","
            << sample.icp.yaw << ","

            << sample.odom_position_error << ","
            << sample.odom_yaw_error << ","

            << sample.icp_position_error << ","
            << sample.icp_yaw_error
            << "\n";


        csv_file_.flush();
    }


    void closeCsv()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close();
        }
    }


    // =====================================================
    // Metrics
    // =====================================================

    void resetMetrics()
    {
        sample_count_ = 0;

        odom_position_squared_sum_ = 0.0;
        odom_yaw_squared_sum_ = 0.0;

        icp_position_squared_sum_ = 0.0;
        icp_yaw_squared_sum_ = 0.0;

        odom_max_position_error_ = 0.0;
        icp_max_position_error_ = 0.0;

        current_waypoint_number_ = 1;
    }


    // =====================================================
    // Members
    // =====================================================

    std::vector<Waypoint> waypoints_;


    geometry_msgs::msg::PoseStamped
        latest_ground_truth_;

    nav_msgs::msg::Odometry
        latest_odom_;


    bool ground_truth_received_ = false;
    bool odom_received_ = false;

    bool benchmark_started_ = false;
    bool benchmark_active_ = false;


    std::size_t current_waypoint_number_ = 1;

    std::size_t sample_count_ = 0;


    double odom_position_squared_sum_ = 0.0;
    double odom_yaw_squared_sum_ = 0.0;

    double icp_position_squared_sum_ = 0.0;
    double icp_yaw_squared_sum_ = 0.0;

    double odom_max_position_error_ = 0.0;
    double icp_max_position_error_ = 0.0;


    std::ofstream csv_file_;
    std::string csv_path_;


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


    rclcpp_action::Client<
        NavigateToPose
    >::SharedPtr navigate_client_;

    rclcpp::Client<
        lifecycle_msgs::srv::GetState
    >::SharedPtr bt_navigator_state_client_;


    bool bt_navigator_active_ = false;
    bool bt_navigator_state_request_pending_ = false;


    rclcpp::TimerBase::SharedPtr
        ready_timer_;

    rclcpp::TimerBase::SharedPtr
        evaluation_timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(
        argc,
        argv
    );

    rclcpp::spin(
        std::make_shared<
            LocalizationBenchmarkRunner
        >()
    );

    rclcpp::shutdown();

    return 0;
}
