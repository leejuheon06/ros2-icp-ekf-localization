#include <memory>
#include <cmath>
#include <vector>
#include <functional>
#include <cstddef>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
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

        RCLCPP_INFO(
            this->get_logger(),
            "ICP Localization Node started"
        );
    }


private:

    void mapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Map received: width=%u, height=%u, resolution=%.3f",
            msg->info.width,
            msg->info.height,
            msg->info.resolution
        );

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


        RCLCPP_INFO(
            this->get_logger(),
            "Converted OccupancyGrid to %zu reference points",
            map_points_.size()
        );


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
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Reference map points available in scanCallback: %zu",
            map_points_.size()
        );
        
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

            auto transform =
                tf_buffer_->lookupTransform(
                    "odom",
                    msg->header.frame_id,
                    tf2::TimePointZero
                );


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


            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Transformed %zu scan points from odom to map",
                map_scan_points.size()
            );


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


            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Transformed %zu scan points from %s to odom",
                odom_points.size(),
                msg->header.frame_id.c_str()
            );
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

        // Debug output
        RCLCPP_INFO(
            this->get_logger(),
            "Converted LaserScan to %zu 2D points",
            points.size()
        );
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