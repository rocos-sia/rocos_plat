#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class LaserScanPublisher : public rclcpp::Node
{
public:
  LaserScanPublisher() : Node("laser_scan_publisher")
  {
    // Create a publisher for the LaserScan message on the /scan topic
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // Set up a timer to periodically publish the message
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 1 Hz (1 message per second)
        std::bind(&LaserScanPublisher::publishLaserScan, this)
    );
  }

private:
  void publishLaserScan()
  {
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    // Fill in the LaserScan message with sample data
    scan_msg->header.stamp = this->now();
    scan_msg->header.frame_id = "map";
    scan_msg->angle_min = -1.57;      // Minimum angle (radians)
    scan_msg->angle_max = 1.57;       // Maximum angle (radians)
    scan_msg->angle_increment = 0.01; // Angle increment between measurements (radians)
    scan_msg->time_increment = 0.0;   // Time increment between measurements (seconds)
    scan_msg->scan_time = 0.1;        // Time it takes to perform one scan (seconds)
    scan_msg->range_min = 0.1;        // Minimum range value (meters)
    scan_msg->range_max = 10.0;       // Maximum range value (meters)

    // Simulated range data (replace this with actual sensor data)
    for (double angle = scan_msg->angle_min; angle <= scan_msg->angle_max; angle += scan_msg->angle_increment)
    {
      double range = 5.0 + 2.5 * std::sin(angle); // Example range data
      scan_msg->ranges.push_back(range);
    }

    publisher_->publish(std::move(scan_msg));
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanPublisher>());
  rclcpp::shutdown();
  return 0;
}
