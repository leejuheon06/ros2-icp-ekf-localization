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

// Simple 2D point structure
struct Point2D
{
    double x;
    double y;
};

// ICP Localization Node
class IcpLocalizationNode : public rclcpp::Node
{
public:

    IcpLocalizationNode()
    : Node("icp_localization_node")
    {
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

        // Map QoS
        auto map_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(1)
            )
            .reliable()
            .transient_local();

        // Map Subscriber
        map_sub_ =
            this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map",
                map_qos,
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

        auto map_points_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(1)
            )
            .reliable()
            .transient_local();

        map_points_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/icp_map_points",
                map_points_qos
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
        
        std::vector<Point2D> map_points;

        map_points.reserve(
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

                map_points.push_back(point);
            }
        }


        RCLCPP_INFO(
            this->get_logger(),
            "Converted OccupancyGrid to %zu reference points",
            map_points.size()
        );


        // -------------------------------------------------
        // Point2D vector -> PointCloud2
        // -------------------------------------------------

        sensor_msgs::msg::PointCloud2 cloud_msg;

        cloud_msg.header =
            msg->header;


        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1,"xyz");  

        modifier.resize(map_points.size());
        cloud_msg.is_dense = true;

        sensor_msgs::PointCloud2Iterator<float>
            iter_x(cloud_msg, "x");

        sensor_msgs::PointCloud2Iterator<float>
            iter_y(cloud_msg, "y");

        sensor_msgs::PointCloud2Iterator<float>
            iter_z(cloud_msg, "z");


        for (const auto& point : map_points)
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

        sensor_msgs::msg::PointCloud2 cloud_msg;


        // Use same timestamp and coordinate frame as LaserScan
        cloud_msg.header = msg->header;


        // -------------------------------------------------
        // Define PointCloud fields
        //
        // Each point contains:
        // x
        // y
        // z
        //
        // Since this is 2D LiDAR:
        // z = 0
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