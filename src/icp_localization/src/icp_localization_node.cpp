#include <memory>
#include <cmath>
#include <vector>
#include <functional>
#include <cstddef>
#include <limits>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Simple 2D point structure
struct Point2D
{
    double x;
    double y;
};

struct Pose2D
{
    double x;
    double y;
    double yaw;
};

struct Correspondence
{
    Point2D source;             // 현재 LiDAR scan에서 나온 point (map_scan_points)
    Point2D target;             // 저장된 OccupancyGrid에서 추출한 reference map point
    double squared_distance;    // source와 target 사이 거리의 제곱
};

// ICP Localization Node
class IcpLocalizationNode : public rclcpp::Node
{
public:

    IcpLocalizationNode()
    : Node("icp_localization_node")
    {
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(
                this->get_clock()
            );

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_
            );

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(
                *this
            );

        // LaserScan Subscriber
        scan_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(
                    &IcpLocalizationNode::scanCallback,
                    this,
                    std::placeholders::_1
                )
            );

        // =================================================
        // Shared QoS for map-related data
        // =================================================
        //
        // 아래 3개의 map 관련 통신에 동일한 QoS를 사용한다.
        //
        // 1) /map subscriber
        //    - Map Server가 발행한 저장 지도를 수신
        //
        // 2) /icp_map_points publisher
        //    - 저장된 map에서 추출한 ICP reference points
        //
        // 3) /icp_scan_points_map publisher
        //    - 현재 LiDAR scan을 map 좌표계로 변환한 points
        //
        // KeepLast(1):
        //   가장 최근 데이터 1개만 보관
        //
        // Reliable:
        //   데이터 전달을 신뢰성 있게 처리
        //
        // Transient Local:
        //   RViz가 늦게 연결되어도 가장 최근 데이터를 받을 수 있음
        // =================================================
        auto map_related_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(1)
            )
            .reliable()
            .transient_local();

        // Map Subscriber
        map_sub_ =
            this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map",
                map_related_qos,
                std::bind(
                    &IcpLocalizationNode::mapCallback,
                    this,
                    std::placeholders::_1
                )
        );

        // Scan PointCloud Publisher
        scan_points_pub_ = 
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/icp_scan_points",
                10
            );

        scan_points_odom_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/icp_scan_points_odom",
                10
            );

        scan_points_map_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/icp_scan_points_map",
                map_related_qos
            );

        map_points_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/icp_map_points",
                map_related_qos
            );

        icp_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/icp_pose",
                10
            );

        RCLCPP_INFO(
            this->get_logger(),
            "ICP Localization Node started"
        );
    }


private:

    void broadcastMapToOdom(
        const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::TransformStamped transform_msg;

        // ICP가 계산한 pose는
        // map 좌표계에서 odom 좌표계의 위치 / 방향을 의미한다.
        transform_msg.header.stamp =
            stamp;

        transform_msg.header.frame_id =
            "map";

        transform_msg.child_frame_id =
            "odom";


        // -------------------------------------------------
        // Translation
        // -------------------------------------------------

        transform_msg.transform.translation.x =
            map_to_odom_.x;

        transform_msg.transform.translation.y =
            map_to_odom_.y;

        transform_msg.transform.translation.z =
            0.0;


        // -------------------------------------------------
        // 2D yaw -> quaternion
        // -------------------------------------------------
        //
        // Roll = 0
        // Pitch = 0
        // Yaw = map_to_odom_.yaw
        //
        // 따라서 quaternion은 z / w 성분만 사용한다.
        // -------------------------------------------------

        const double half_yaw =
            map_to_odom_.yaw * 0.5;

        transform_msg.transform.rotation.x =
            0.0;

        transform_msg.transform.rotation.y =
            0.0;

        transform_msg.transform.rotation.z =
            std::sin(
                half_yaw
            );

        transform_msg.transform.rotation.w =
            std::cos(
                half_yaw
            );


        tf_broadcaster_->sendTransform(
            transform_msg
        );


        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(),
        //     *this->get_clock(),
        //     2000,
        //     "Broadcast map -> odom TF | x: %.5f m | y: %.5f m | yaw: %.5f rad",
        //     map_to_odom_.x,
        //     map_to_odom_.y,
        //     map_to_odom_.yaw
        // );
    }


    bool publishIcpPose(
        const rclcpp::Time& stamp)
    {
        try
        {
            // -------------------------------------------------
            // odom -> base_footprint
            // -------------------------------------------------
            //
            // ICP는 map -> odom을 추정한다.
            // 최종 ICP robot pose는:
            //
            // T_map_base =
            //     T_map_odom * T_odom_base
            //
            // 로 계산한다.
            // -------------------------------------------------

            const auto odom_to_base =
                tf_buffer_->lookupTransform(
                    "odom",
                    "base_footprint",
                    stamp,
                    rclcpp::Duration::from_seconds(0.1)
                );


            const double odom_x =
                odom_to_base.transform.translation.x;

            const double odom_y =
                odom_to_base.transform.translation.y;

            const double odom_yaw =
                tf2::getYaw(
                    odom_to_base.transform.rotation
                );


            const double cos_map_yaw =
                std::cos(
                    map_to_odom_.yaw
                );

            const double sin_map_yaw =
                std::sin(
                    map_to_odom_.yaw
                );


            // -------------------------------------------------
            // T_map_odom * T_odom_base
            // -------------------------------------------------

            const double map_base_x =
                cos_map_yaw * odom_x
                - sin_map_yaw * odom_y
                + map_to_odom_.x;

            const double map_base_y =
                sin_map_yaw * odom_x
                + cos_map_yaw * odom_y
                + map_to_odom_.y;


            const double map_base_yaw_raw =
                map_to_odom_.yaw +
                odom_yaw;

            const double map_base_yaw =
                std::atan2(
                    std::sin(map_base_yaw_raw),
                    std::cos(map_base_yaw_raw)
                );


            geometry_msgs::msg::PoseStamped pose_msg;

            pose_msg.header.stamp =
                stamp;

            pose_msg.header.frame_id =
                "map";


            pose_msg.pose.position.x =
                map_base_x;

            pose_msg.pose.position.y =
                map_base_y;

            pose_msg.pose.position.z =
                0.0;


            const double half_yaw =
                map_base_yaw * 0.5;

            pose_msg.pose.orientation.x =
                0.0;

            pose_msg.pose.orientation.y =
                0.0;

            pose_msg.pose.orientation.z =
                std::sin(
                    half_yaw
                );

            pose_msg.pose.orientation.w =
                std::cos(
                    half_yaw
                );


            icp_pose_pub_->publish(
                pose_msg
            );


            return true;
        }
        catch (
            const tf2::TransformException& ex
        )
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Could not publish /icp_pose: %s",
                ex.what()
            );


            return false;
        }
    }


    void mapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Map received: width=%u, height=%u, resolution=%.3f",
        //     msg->info.width,
        //     msg->info.height,
        //     msg->info.resolution
        // );

        // width = 0이면 index / width 계산이 불가능하므로
        // 비정상 map으로 판단한다.
        if (
            msg->info.width == 0 ||
            msg->info.height == 0
        )
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Received empty OccupancyGrid"
            );

            return;
        }
        
        // [수정] map_points는 더 이상 mapCallback() 안에서만 존재하는 지역 변수가 아니다.
        // /map에서 만든 기준점을 이후 scanCallback()의 ICP 계산에서도
        // 사용할 수 있도록 class member인 map_points_에 저장한다.
        map_points_.clear();

        map_points_.reserve(
            msg->data.size()
        );

        const std::size_t width =
            msg->info.width;

        const double resolution =
            msg->info.resolution;

        const double origin_x =
            msg->info.origin.position.x;

        const double origin_y =
            msg->info.origin.position.y;

        // OccupancyGrid data traversal
        for (
            std::size_t index = 0;
            index < msg->data.size();
            ++index
        )
        {
            const int occupancy =
                msg->data[index];


            // Use occupied cells only
            if (occupancy >= 65)
            {
                const std::size_t column =
                    index % width;

                const std::size_t row =
                    index / width;


                const double x =
                    origin_x +
                    (static_cast<double>(column) + 0.5) *
                    resolution;

                const double y =
                    origin_y +
                    (static_cast<double>(row) + 0.5) *
                    resolution;


                Point2D point;

                point.x = x;
                point.y = y;

                map_points_.push_back(point);
            }
        }


        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Converted OccupancyGrid to %zu reference points",
        //     map_points_.size()
        // );


        // -------------------------------------------------
        // Point2D vector -> PointCloud2
        // -------------------------------------------------
        sensor_msgs::msg::PointCloud2 cloud_msg;

        cloud_msg.header =
            msg->header;


        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1,"xyz");  

        modifier.resize(map_points_.size());
        cloud_msg.is_dense = true;

        sensor_msgs::PointCloud2Iterator<float>
            iter_x(cloud_msg, "x");

        sensor_msgs::PointCloud2Iterator<float>
            iter_y(cloud_msg, "y");

        sensor_msgs::PointCloud2Iterator<float>
            iter_z(cloud_msg, "z");


        for (const auto& point : map_points_)
        {
            *iter_x =
                static_cast<float>(point.x);

            *iter_y =
                static_cast<float>(point.y);

            *iter_z =
                0.0f;


            ++iter_x;
            ++iter_y;
            ++iter_z;
        }


        map_points_pub_->publish(
            cloud_msg
        );
    }

    // LaserScan Callback
    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(),
        //     *this->get_clock(),
        //     2000,
        //     "Reference map points available in scanCallback: %zu",
        //     map_points_.size()
        // );
        
        std::vector<Point2D> points;

        points.reserve(msg->ranges.size());

        // LaserScan -> 2D Point Conversion
        for(std::size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const float range = msg->ranges[i];

            if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max)
            {
                // double angle = msg->angle_min + i * msg->angle_increment;
                double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
                double x = range * std::cos(angle);
                double y = range * std::sin(angle);

                Point2D point;
                point.x = x;
                point.y = y;

                points.push_back(point);
            }
        }

        try
        {
            // -------------------------------------------------
            // laser_link -> odom TF 조회
            // -------------------------------------------------

            // -------------------------------------------------
            // LaserScan timestamp와 동일한 시점의 TF 조회
            // -------------------------------------------------
            //
            // 기존 tf2::TimePointZero는 "가장 최신 TF"를 가져온다.
            //
            // 하지만 로봇이 움직이는 동안에는:
            //
            //   LaserScan 측정 시각 != 가장 최신 TF 시각
            //
            // 이 될 수 있기 때문에 scan point를 잘못된 robot pose로
            // 변환하는 시간 오차가 발생할 수 있다.
            //
            // 따라서 현재 LaserScan의 header.stamp를 사용해서
            // 해당 scan이 실제 측정된 시각의 laser_link -> odom
            // transform을 조회한다.
            // -------------------------------------------------
            const rclcpp::Time scan_time(
                msg->header.stamp
            );


            auto transform =
                tf_buffer_->lookupTransform(
                    "odom",
                    msg->header.frame_id,
                    scan_time,
                    rclcpp::Duration::from_seconds(0.1)
                );


            // -------------------------------------------------
            // Timestamp synchronization validation
            // -------------------------------------------------
            //
            // lookupTransform()이 성공했다는 것은 tf_buffer_ 안에
            // scan_time에 대응되는 TF history가 존재한다는 의미다.
            // -------------------------------------------------
            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     2000,
            //     "Timestamp-synced TF | scan: %.3f sec | frame: %s -> odom",
            //     scan_time.seconds(),
            //     msg->header.frame_id.c_str()
            // );


            // -------------------------------------------------
            // TF Translation
            // -------------------------------------------------

            const double tx =
                transform.transform.translation.x;

            const double ty =
                transform.transform.translation.y;


            // -------------------------------------------------
            // TF Quaternion -> Yaw
            // -------------------------------------------------

            const double yaw =
                tf2::getYaw(
                    transform.transform.rotation
                );


            // -------------------------------------------------
            // Laser Frame Points -> Odom Frame Points
            // -------------------------------------------------

            std::vector<Point2D> odom_points;

            odom_points.reserve(
                points.size()
            );


            // -------------------------------------------------
            // Apply rotation + translation
            // -------------------------------------------------

            for (const auto& point : points)
            {
                Point2D transformed_point;


                transformed_point.x =
                    std::cos(yaw) * point.x
                    - std::sin(yaw) * point.y
                    + tx;


                transformed_point.y =
                    std::sin(yaw) * point.x
                    + std::cos(yaw) * point.y
                    + ty;


                odom_points.push_back(
                    transformed_point
                );
            }


            // =================================================
            // Odom Frame Points -> Map Frame Points
            // =================================================
            //
            // 현재 odom_points는 "odom" 좌표계 기준이다.
            //
            // ICP에서 저장된 지도 기준점(map_points_)과 비교하려면
            // 현재 LiDAR scan도 같은 "map" 좌표계로 표현되어야 한다.
            //
            // map_to_odom_은 현재 추정 중인 map -> odom 변환값이다.
            // 이 변환 T_map_odom을 odom 기준 point에 적용하면
            // 해당 point를 map 좌표계에서 표현할 수 있다.
            //
            // 현재 benchmark에서는 초기 검증 결과
            // map과 odom이 거의 동일하게 정렬되어 있으므로
            // map_to_odom_의 초기값을 (0, 0, 0)으로 사용한다.
            //
            // 이후 ICP가 구현되면 이 값은 고정값이 아니라
            // ICP가 계산한 보정 결과에 따라 갱신될 예정이다.
            // =================================================

            std::vector<Point2D> map_scan_points;

            map_scan_points.reserve(
                odom_points.size()
            );


            // -------------------------------------------------
            // map -> odom 추정값의 yaw에 대한 sin / cos
            //
            // 모든 point마다 sin(), cos()를 반복 계산하지 않고
            // scan 하나당 한 번만 계산해서 재사용한다.
            // -------------------------------------------------

            const double cos_map_yaw =
                std::cos(
                    map_to_odom_.yaw
                );

            const double sin_map_yaw =
                std::sin(
                    map_to_odom_.yaw
                );


            // -------------------------------------------------
            // 2D Rigid Transform
            //
            // odom 기준 point (x, y)
            //          ↓
            // 회전 + 이동
            //          ↓
            // map 기준 point (x', y')
            //
            // x' = cos(yaw)x - sin(yaw)y + tx
            // y' = sin(yaw)x + cos(yaw)y + ty
            // -------------------------------------------------

            for (const auto& point : odom_points)
            {
                Point2D map_point;


                map_point.x =
                    cos_map_yaw * point.x
                    - sin_map_yaw * point.y
                    + map_to_odom_.x;


                map_point.y =
                    sin_map_yaw * point.x
                    + cos_map_yaw * point.y
                    + map_to_odom_.y;


                map_scan_points.push_back(
                    map_point
                );
            }

            // =================================================
            // ICP Iteration + Convergence
            // =================================================
            const double max_correspondence_distance =
                0.15;

            // 한 개의 LaserScan 프레임에 대해 최대 10회 반복
            const int max_icp_iterations =
                10;

            const double translation_convergence_threshold =
                0.001;

            const double rotation_convergence_threshold =
                0.001;


            if (map_points_.empty())
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Reference map points are not available yet"
                );
            }
            else
            {
                Pose2D iteration_pose =
                    map_to_odom_;

                bool converged =
                    false;

                int performed_iterations =
                    0;

                std::size_t final_raw_count =
                    0;

                std::size_t final_valid_count =
                    0;

                std::size_t final_rejected_count =
                    0;

                double final_raw_mean_distance =
                    0.0;

                double final_valid_mean_distance =
                    0.0;

                double final_raw_max_distance =
                    0.0;

                double final_corrected_mean_distance =
                    0.0;

                double final_delta_x =
                    0.0;

                double final_delta_y =
                    0.0;

                double final_delta_yaw =
                    0.0;


                // 한 개의 LaserScan에 대해 correspondence와 correction을
                // 반복해서 계산하고, correction이 충분히 작아지면 종료한다.
                for (
                    int iteration = 0;
                    iteration < max_icp_iterations;
                    ++iteration
                )
                {
                    // ---------------------------------------------
                    // Current Pose -> Map Frame Scan Points
                    // ---------------------------------------------
                    map_scan_points.clear();

                    map_scan_points.reserve(
                        odom_points.size()
                    );


                    const double iteration_cos_yaw =
                        std::cos(
                            iteration_pose.yaw
                        );

                    const double iteration_sin_yaw =
                        std::sin(
                            iteration_pose.yaw
                        );


                    for (const auto& point : odom_points)
                    {
                        Point2D map_point;


                        map_point.x =
                            iteration_cos_yaw * point.x
                            - iteration_sin_yaw * point.y
                            + iteration_pose.x;


                        map_point.y =
                            iteration_sin_yaw * point.x
                            + iteration_cos_yaw * point.y
                            + iteration_pose.y;


                        map_scan_points.push_back(
                            map_point
                        );
                    }


                    // ---------------------------------------------
                    // Nearest Neighbor + Outlier Rejection
                    // ---------------------------------------------
                    std::vector<Correspondence> correspondences;

                    correspondences.reserve(
                        map_scan_points.size()
                    );


                    std::size_t rejected_correspondence_count =
                        0;

                    double raw_distance_sum =
                        0.0;

                    double valid_distance_sum =
                        0.0;

                    double maximum_raw_distance =
                        0.0;


                    for (const auto& scan_point : map_scan_points)
                    {
                        double minimum_squared_distance =
                            std::numeric_limits<double>::max();


                        Point2D nearest_map_point;


                        for (const auto& map_point : map_points_)
                        {
                            const double dx =
                                map_point.x - scan_point.x;

                            const double dy =
                                map_point.y - scan_point.y;


                            const double squared_distance =
                                dx * dx + dy * dy;


                            if (
                                squared_distance <
                                minimum_squared_distance
                            )
                            {
                                minimum_squared_distance =
                                    squared_distance;

                                nearest_map_point =
                                    map_point;
                            }
                        }


                        const double nearest_distance =
                            std::sqrt(
                                minimum_squared_distance
                            );


                        raw_distance_sum +=
                            nearest_distance;

                        if (
                            nearest_distance >
                            maximum_raw_distance
                        )
                        {
                            maximum_raw_distance =
                                nearest_distance;
                        }


                        if (
                            nearest_distance >
                            max_correspondence_distance
                        )
                        {
                            ++rejected_correspondence_count;
                            continue;
                        }


                        Correspondence correspondence;

                        correspondence.source =
                            scan_point;

                        correspondence.target =
                            nearest_map_point;

                        correspondence.squared_distance =
                            minimum_squared_distance;


                        correspondences.push_back(
                            correspondence
                        );


                        valid_distance_sum +=
                            nearest_distance;
                    }


                    const std::size_t raw_correspondence_count =
                        map_scan_points.size();


                    const double raw_mean_distance =
                        raw_correspondence_count == 0
                        ? 0.0
                        : raw_distance_sum /
                          static_cast<double>(
                              raw_correspondence_count
                          );


                    const double valid_mean_distance =
                        correspondences.empty()
                        ? 0.0
                        : valid_distance_sum /
                          static_cast<double>(
                              correspondences.size()
                          );


                    if (correspondences.size() < 3)
                    {
                        // RCLCPP_WARN_THROTTLE(
                        //     this->get_logger(),
                        //     *this->get_clock(),
                        //     2000,
                        //     "Not enough valid correspondences for ICP: %zu",
                        //     correspondences.size()
                        // );

                        break;
                    }


                    // ---------------------------------------------
                    // Centroid
                    // ---------------------------------------------
                    Point2D source_centroid {
                        0.0,
                        0.0
                    };

                    Point2D target_centroid {
                        0.0,
                        0.0
                    };


                    for (const auto& correspondence : correspondences)
                    {
                        source_centroid.x +=
                            correspondence.source.x;

                        source_centroid.y +=
                            correspondence.source.y;


                        target_centroid.x +=
                            correspondence.target.x;

                        target_centroid.y +=
                            correspondence.target.y;
                    }


                    const double correspondence_count =
                        static_cast<double>(
                            correspondences.size()
                        );


                    source_centroid.x /=
                        correspondence_count;

                    source_centroid.y /=
                        correspondence_count;


                    target_centroid.x /=
                        correspondence_count;

                    target_centroid.y /=
                        correspondence_count;


                    // ---------------------------------------------
                    // Delta Yaw
                    // ---------------------------------------------
                    double rotation_dot_sum =
                        0.0;

                    double rotation_cross_sum =
                        0.0;


                    for (const auto& correspondence : correspondences)
                    {
                        const double source_x =
                            correspondence.source.x -
                            source_centroid.x;

                        const double source_y =
                            correspondence.source.y -
                            source_centroid.y;


                        const double target_x =
                            correspondence.target.x -
                            target_centroid.x;

                        const double target_y =
                            correspondence.target.y -
                            target_centroid.y;


                        rotation_dot_sum +=
                            source_x * target_x +
                            source_y * target_y;


                        rotation_cross_sum +=
                            source_x * target_y -
                            source_y * target_x;
                    }


                    const double delta_yaw =
                        std::atan2(
                            rotation_cross_sum,
                            rotation_dot_sum
                        );


                    const double cos_delta_yaw =
                        std::cos(
                            delta_yaw
                        );

                    const double sin_delta_yaw =
                        std::sin(
                            delta_yaw
                        );


                    // ---------------------------------------------
                    // Delta X / Delta Y
                    // ---------------------------------------------
                    const double rotated_source_centroid_x =
                        cos_delta_yaw * source_centroid.x -
                        sin_delta_yaw * source_centroid.y;

                    const double rotated_source_centroid_y =
                        sin_delta_yaw * source_centroid.x +
                        cos_delta_yaw * source_centroid.y;


                    const double delta_x =
                        target_centroid.x -
                        rotated_source_centroid_x;

                    const double delta_y =
                        target_centroid.y -
                        rotated_source_centroid_y;


                    // ---------------------------------------------
                    // Correction Validation
                    // ---------------------------------------------
                    double corrected_distance_sum =
                        0.0;


                    for (const auto& correspondence : correspondences)
                    {
                        const double corrected_source_x =
                            cos_delta_yaw * correspondence.source.x
                            - sin_delta_yaw * correspondence.source.y
                            + delta_x;

                        const double corrected_source_y =
                            sin_delta_yaw * correspondence.source.x
                            + cos_delta_yaw * correspondence.source.y
                            + delta_y;


                        const double corrected_dx =
                            correspondence.target.x -
                            corrected_source_x;

                        const double corrected_dy =
                            correspondence.target.y -
                            corrected_source_y;


                        corrected_distance_sum +=
                            std::sqrt(
                                corrected_dx * corrected_dx +
                                corrected_dy * corrected_dy
                            );
                    }


                    const double corrected_mean_distance =
                        corrected_distance_sum /
                        correspondence_count;


                    // ---------------------------------------------
                    // Delta_T * Current_T
                    // ---------------------------------------------
                    const double old_x =
                        iteration_pose.x;

                    const double old_y =
                        iteration_pose.y;

                    const double old_yaw =
                        iteration_pose.yaw;


                    iteration_pose.x =
                        cos_delta_yaw * old_x -
                        sin_delta_yaw * old_y +
                        delta_x;

                    iteration_pose.y =
                        sin_delta_yaw * old_x +
                        cos_delta_yaw * old_y +
                        delta_y;


                    const double updated_yaw =
                        old_yaw +
                        delta_yaw;


                    iteration_pose.yaw =
                        std::atan2(
                            std::sin(updated_yaw),
                            std::cos(updated_yaw)
                        );


                    performed_iterations =
                        iteration + 1;


                    final_raw_count =
                        raw_correspondence_count;

                    final_valid_count =
                        correspondences.size();

                    final_rejected_count =
                        rejected_correspondence_count;

                    final_raw_mean_distance =
                        raw_mean_distance;

                    final_valid_mean_distance =
                        valid_mean_distance;

                    final_raw_max_distance =
                        maximum_raw_distance;

                    final_corrected_mean_distance =
                        corrected_mean_distance;

                    final_delta_x =
                        delta_x;

                    final_delta_y =
                        delta_y;

                    final_delta_yaw =
                        delta_yaw;


                    const double translation_correction =
                        std::hypot(
                            delta_x,
                            delta_y
                        );


                    if (
                        translation_correction <
                        translation_convergence_threshold &&
                        std::abs(delta_yaw) <
                        rotation_convergence_threshold
                    )
                    {
                        converged =
                            true;

                        break;
                    }
                }


                // 한 Scan에 대한 ICP 반복이 끝난 뒤에만
                // 최종 pose를 map_to_odom_에 반영한다.
                map_to_odom_ =
                    iteration_pose;


                // -------------------------------------------------
                // Dynamic map -> odom TF broadcast
                // -------------------------------------------------
                //
                // 현재 LaserScan의 timestamp를 그대로 사용해서
                // 이번 ICP 결과에 대응되는 map -> odom TF를 발행한다.
                //
                // 이제 localization_icp.launch.py의 임시 static
                // map -> odom publisher는 제거해야 한다.
                // -------------------------------------------------
                broadcastMapToOdom(
                    scan_time
                );


                // -------------------------------------------------
                // Publish ICP robot pose
                // -------------------------------------------------
                //
                // /icp_pose is the map-frame robot pose used as
                // the EKF measurement.
                // -------------------------------------------------
                publishIcpPose(
                    scan_time
                );


                // RViz publish용 map_scan_points도 최종 pose로 갱신한다.
                map_scan_points.clear();

                map_scan_points.reserve(
                    odom_points.size()
                );


                const double final_cos_yaw =
                    std::cos(
                        map_to_odom_.yaw
                    );

                const double final_sin_yaw =
                    std::sin(
                        map_to_odom_.yaw
                    );


                for (const auto& point : odom_points)
                {
                    Point2D map_point;


                    map_point.x =
                        final_cos_yaw * point.x
                        - final_sin_yaw * point.y
                        + map_to_odom_.x;


                    map_point.y =
                        final_sin_yaw * point.x
                        + final_cos_yaw * point.y
                        + map_to_odom_.y;


                    map_scan_points.push_back(
                        map_point
                    );
                }


                // RCLCPP_INFO_THROTTLE(
                //     this->get_logger(),
                //     *this->get_clock(),
                //     2000,
                //     "ICP Iteration | iter: %d/%d | converged: %s | "
                //     "Raw: %zu | Valid: %zu | Rejected: %zu | "
                //     "Raw mean: %.5f m | mean: %.5f -> %.5f m | Raw max: %.5f m",
                //     performed_iterations,
                //     max_icp_iterations,
                //     converged ? "true" : "false",
                //     final_raw_count,
                //     final_valid_count,
                //     final_rejected_count,
                //     final_raw_mean_distance,
                //     final_valid_mean_distance,
                //     final_corrected_mean_distance,
                //     final_raw_max_distance
                // );


                // RCLCPP_INFO_THROTTLE(
                //     this->get_logger(),
                //     *this->get_clock(),
                //     2000,
                //     "Final Correction | "
                //     "dx: %.6f m | dy: %.6f m | dyaw: %.6f rad | "
                //     "map_to_odom: (%.5f, %.5f, %.5f)",
                //     final_delta_x,
                //     final_delta_y,
                //     final_delta_yaw,
                //     map_to_odom_.x,
                //     map_to_odom_.y,
                //     map_to_odom_.yaw
                // );
            }


            // =================================================
            // Map Frame PointCloud2
            // =================================================
            //
            // map_scan_points를 RViz에서 확인하기 위해
            // sensor_msgs/msg/PointCloud2 형태로 변환한다.
            //
            // 좌표값 자체가 이미 map 기준으로 계산되어 있으므로
            // frame_id도 반드시 "map"으로 지정한다.
            // =================================================

            sensor_msgs::msg::PointCloud2 map_scan_cloud_msg;


            // 원래 LaserScan과 동일한 측정 시각을 유지한다.
            map_scan_cloud_msg.header.stamp =
                msg->header.stamp;


            // 좌표 자체가 map 기준이므로 frame_id = "map"
            map_scan_cloud_msg.header.frame_id =
                "map";


            sensor_msgs::PointCloud2Modifier map_scan_modifier(
                map_scan_cloud_msg
            );


            map_scan_modifier.setPointCloud2FieldsByString(
                1,
                "xyz"
            );


            map_scan_modifier.resize(
                map_scan_points.size()
            );


            map_scan_cloud_msg.is_dense =
                true;


            // -------------------------------------------------
            // PointCloud2 Iterators
            // -------------------------------------------------

            sensor_msgs::PointCloud2Iterator<float>
                map_scan_iter_x(
                    map_scan_cloud_msg,
                    "x"
                );

            sensor_msgs::PointCloud2Iterator<float>
                map_scan_iter_y(
                    map_scan_cloud_msg,
                    "y"
                );

            sensor_msgs::PointCloud2Iterator<float>
                map_scan_iter_z(
                    map_scan_cloud_msg,
                    "z"
                );


            // -------------------------------------------------
            // map_scan_points -> PointCloud2
            // -------------------------------------------------

            for (const auto& point : map_scan_points)
            {
                *map_scan_iter_x =
                    static_cast<float>(
                        point.x
                    );

                *map_scan_iter_y =
                    static_cast<float>(
                        point.y
                    );

                // 2D LiDAR이므로 z = 0
                *map_scan_iter_z =
                    0.0f;


                ++map_scan_iter_x;
                ++map_scan_iter_y;
                ++map_scan_iter_z;
            }


            // -------------------------------------------------
            // Publish map-frame scan points
            // -------------------------------------------------

            scan_points_map_pub_->publish(
                map_scan_cloud_msg
            );


            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     2000,
            //     "Transformed %zu scan points from odom to map",
            //     map_scan_points.size()
            // );


            // -------------------------------------------------
            // Odom PointCloud2
            // -------------------------------------------------

            sensor_msgs::msg::PointCloud2 odom_cloud_msg;


            odom_cloud_msg.header.stamp =
                msg->header.stamp;

            odom_cloud_msg.header.frame_id =
                "odom";


            sensor_msgs::PointCloud2Modifier odom_modifier(
                odom_cloud_msg
            );


            odom_modifier.setPointCloud2FieldsByString(
                1,
                "xyz"
            );

            odom_modifier.resize(
                odom_points.size()
            );

            odom_cloud_msg.is_dense =
                true;

            // -------------------------------------------------
            // PointCloud2 Iterators
            // -------------------------------------------------

            sensor_msgs::PointCloud2Iterator<float>
                odom_iter_x(
                    odom_cloud_msg,
                    "x"
                );

            sensor_msgs::PointCloud2Iterator<float>
                odom_iter_y(
                    odom_cloud_msg,
                    "y"
                );

            sensor_msgs::PointCloud2Iterator<float>
                odom_iter_z(
                    odom_cloud_msg,
                    "z"
                );


            // -------------------------------------------------
            // Copy Odom Points
            // -------------------------------------------------

            for (const auto& point : odom_points)
            {
                *odom_iter_x =
                    static_cast<float>(
                        point.x
                    );

                *odom_iter_y =
                    static_cast<float>(
                        point.y
                    );

                *odom_iter_z =
                    0.0f;


                ++odom_iter_x;
                ++odom_iter_y;
                ++odom_iter_z;
            }


            // -------------------------------------------------
            // Publish
            // -------------------------------------------------

            scan_points_odom_pub_->publish(
                odom_cloud_msg
            );


            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     2000,
            //     "Transformed %zu scan points from %s to odom",
            //     odom_points.size(),
            //     msg->header.frame_id.c_str()
            // );
        }
        catch (
            const tf2::TransformException& ex
        )
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Could not transform %s to odom: %s",
                msg->header.frame_id.c_str(),
                ex.what()
            );


            return;
        }


        // -------------------------------------------------
        // Point2D -> PointCloud2
        // -------------------------------------------------

        sensor_msgs::msg::PointCloud2 cloud_msg;


        // Use same timestamp and coordinate frame as LaserScan
        cloud_msg.header = msg->header;


        // -------------------------------------------------
        // Define PointCloud fields
        // -------------------------------------------------

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        // Allocate enough memory for all points
        modifier.resize(points.size());

        // This point cloud contains valid points only
        cloud_msg.is_dense = true;


        // -------------------------------------------------
        // PointCloud2 Iterators
        // -------------------------------------------------
        sensor_msgs::PointCloud2Iterator<float>
            iter_x(
                cloud_msg,
                "x"
            );

        sensor_msgs::PointCloud2Iterator<float>
            iter_y(
                cloud_msg,
                "y"
            );

        sensor_msgs::PointCloud2Iterator<float>
            iter_z(
                cloud_msg,
                "z"
            );


        // -------------------------------------------------
        // Copy Point2D data into PointCloud2
        // -------------------------------------------------

        for (const auto& point : points)
        {
            *iter_x =
                static_cast<float>(
                    point.x
                );

            *iter_y =
                static_cast<float>(
                    point.y
                );

            *iter_z =
                0.0f;


            ++iter_x;
            ++iter_y;
            ++iter_z;
        }


        // -------------------------------------------------
        // Publish PointCloud2
        // -------------------------------------------------

        scan_points_pub_->publish(
            cloud_msg
        );

        // // Debug output
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Converted LaserScan to %zu 2D points",
        //     points.size()
        // );
    }

    // =====================================================
    // ICP Reference Map Points
    // =====================================================

    std::vector<Point2D> map_points_;


    // =====================================================
    // Initial map -> odom Pose Estimate
    // =====================================================
    //
    // 현재 benchmark에서는 map과 odom이 초기 상태에서
    // 거의 동일하게 정렬되는 것을 RViz에서 검증했기 때문에
    // 초기 추정값을 x=0, y=0, yaw=0 으로 둔다.
    //
    // 현재 단계:
    //   고정된 Initial Guess
    //
    // 이후 ICP 구현 후:
    //   ICP가 계산한 correction을 이용해 이 값을 갱신
    // =====================================================

    Pose2D map_to_odom_ {
        0.0,
        0.0,
        0.0
    };


    std::unique_ptr<
        tf2_ros::Buffer
    > tf_buffer_;

    std::shared_ptr<
        tf2_ros::TransformListener
    > tf_listener_;

    std::unique_ptr<
        tf2_ros::TransformBroadcaster
    > tf_broadcaster_;

    // ROS2 Subscriber
    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan
    >::SharedPtr scan_sub_;

    rclcpp::Subscription<
        nav_msgs::msg::OccupancyGrid
    >::SharedPtr map_sub_;

    // ROS2 Publisher
    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2
    >::SharedPtr scan_points_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2
    >::SharedPtr map_points_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2
    >::SharedPtr scan_points_odom_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2
    >::SharedPtr scan_points_map_pub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr icp_pose_pub_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<IcpLocalizationNode>()
    );

    rclcpp::shutdown();

    return 0;
}