// 本代码的主要目的在于统一消息类型格式，因为cartographer激光雷达slam建图的话题为/scan
// 消息类型格式：sensor_msgs/msg/LaserScan
// 而原本代码使用的格式为sensor_msgs/msg/point_cloud2.hpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <iostream>
#include <cstring>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iomanip>
#include <random>

// 定义服务器的IP地址和端口
const std::string SERVER_IP = "192.168.31.111";
const int SERVER_PORT = 5555;
// 定义查询参数
std::vector<uint8_t> hexQuery = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00};

// Convert big-endian bytes to double
double bigEndianToDouble(const uint8_t *bytes)
{
  uint64_t value = 0;
  for (size_t i = 0; i < sizeof(double); ++i)
  {
    value = (value << 8) | bytes[i];
  }
  return *reinterpret_cast<double *>(&value);
}

void parseTCPData(const std::vector<uint8_t> &data, std::vector<double> &output)
{
  if (data.size() % sizeof(double) != 0)
  {
    std::cout << "Invalid data size" << std::endl;
    return;
  }
  else
  {

    std::cout << "Effective data size  " << data.size() << std::endl;
  }

  size_t numDoubles = data.size() / sizeof(double);
  output.resize(numDoubles);

  for (size_t i = 0; i < numDoubles; ++i)
  {
    double value = bigEndianToDouble(&data[i * sizeof(double)]);
    output[i] = value;
  }
}

//

// 定义节点类
class LaserScanPublisher : public rclcpp::Node
{
public:
  LaserScanPublisher() : Node("publisher_node")
  {
    // Create a publisher for the LaserScan message on the /scan topic
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // Set up a timer to periodically publish the message
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 1 Hz (1 message per second)
        std::bind(&LaserScanPublisher::publishLaserScan, this));
  }
  // 参数
  std::vector<uint8_t> receivedData;
  std::vector<double> parsedData;
  std::vector<uint8_t> anaylazeData;
  // 大端转小端
  double bigEndianToDouble(const uint8_t *bytes)
  {
    uint64_t value = 0;
    for (size_t i = 0; i < sizeof(double); ++i)
    {
      value = (value << 8) | bytes[i];
    }
    return *reinterpret_cast<double *>(&value);
  }
  // 转换数据成浮点数
  void parseTCPData(const std::vector<uint8_t> &data, std::vector<double> &output)
  {
    if (data.size() % sizeof(double) != 0)
    {
      std::cout << "Invalid data size" << std::endl;
      return;
    }
    else
    {
      std::cout << "youxiao " << data.size() << std::endl;
    }

    size_t numDoubles = data.size() / sizeof(double);
    output.resize(numDoubles);

    for (size_t i = 0; i < numDoubles; ++i)
    {
      double value = bigEndianToDouble(&data[i * sizeof(double)]);
      output[i] = value;
    }
  }
  // 从服务器接收数据
  std::vector<uint8_t> receiveDataFromServer(const std::string &serverIP, int serverPort, const std::vector<uint8_t> &hexQuery)
  {
    std::vector<uint8_t> receivedData;

    // 创建套接字
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1)
    {
      std::cerr << "Failed to create socket." << std::endl;
      return receivedData;
    }

    // 设置服务器地址
    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);
    if (inet_pton(AF_INET, serverIP.c_str(), &(serverAddr.sin_addr)) <= 0)
    {
      std::cerr << "Invalid address/Address not supported." << std::endl;
      close(sockfd);
      return receivedData;
    }

    // 连接到服务器
    if (connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1)
    {
      std::cerr << "Failed to connect to server." << std::endl;
      close(sockfd);
      return receivedData;
    }
    // 发送查询参数
    if (!hexQuery.empty())
    {
      std::cout << "Sending query to server..." << std::endl;
      ssize_t bytesSent = send(sockfd, hexQuery.data(), hexQuery.size(), 0);
      if (bytesSent == -1)
      {
        std::cerr << "Failed to send query to server." << std::endl;
        close(sockfd);
        return receivedData;
      }
      std::cout << "Query sent." << std::endl;
    }

    // 从服务器接收数据
    // 打印反馈内容

    unsigned char buffer[5770];
    int bytesRead;
    int totalBytesReceived = 0;
    while ((bytesRead = recv(sockfd, buffer, sizeof(buffer), 0)) > 0)
    {
      if (bytesRead <= 0)
      {
        // 处理接收错误或连接关闭的情况
        break;
      }

      receivedData.insert(receivedData.end(), buffer, buffer + bytesRead);
      totalBytesReceived += bytesRead;
      std::cout << "Received " << bytesRead << " bytes from server. Total received: " << totalBytesReceived << std::endl;
      if (totalBytesReceived == 5770)
      {
        break;
      }
    }

    if (!receivedData.empty())
    {
      std::cout << "Received response: ";
      for (const auto &byte : receivedData)
      {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
      }
      std::cout << std::endl;
    }

    close(sockfd);

    return receivedData;
  }

private:
  void publishLaserScan()
  {
    // 使用 sensor_msgs::msg::LaserScan 替换 sensor_msgs::msg::PointCloud2
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    // Fill in the LaserScan message with sample data
    scan_msg->header.stamp = this->now();
    scan_msg->header.frame_id = "laser";
    scan_msg->angle_min = -1.57 * 2;  // Minimum angle (radians)
    scan_msg->angle_max = 1.57 * 2;   // Maximum angle (radians)
    scan_msg->angle_increment = 0.01; // Angle increment between measurements (radians)
    scan_msg->time_increment = 0.0;   // Time increment between measurements (seconds)
    scan_msg->scan_time = 0.1;        // Time it takes to perform one scan (seconds)
    scan_msg->range_min = 0.1;        // Minimum range value (meters)
    scan_msg->range_max = 10.0;       // Maximum range value (meters)

    // tcp接收数据
    // receivedData = receiveDataFromServer(SERVER_IP, SERVER_PORT, hexQuery);
    // anaylazeData = {receivedData.begin() + 10, receivedData.end()};
    // parseTCPData(anaylazeData, parsedData);

    // 设置激光扫描数据
    for (double angle = scan_msg->angle_min; angle <= scan_msg->angle_max; angle += scan_msg->angle_increment)
    {
      double x = angle;                                // Example x coordinate
      double y = std::sin(angle);                      // Example y coordinate
                                                       // 设置随机数生成器
      std::random_device rd;                           // 用于获得真随机数种子
      std::mt19937 gen(rd());                          // 使用 Mersenne Twister 作为随机数引擎
      std::uniform_real_distribution<> dis(-0.5, 0.5); // 均匀分布随机数范围 [-0.1, 0.1]
      // 添加随机数
      double random_value = dis(gen);
      y+=random_value;
      // Push the x, y coordinates as a pair into the ranges array
      scan_msg->ranges.push_back(std::sqrt(x * x + y * y));
    }

    /*
    // Set the number of ranges based on the number of parsed data points
    scan_msg->ranges.resize(parsedData.size() / 2);

    // Set point cloud data in the LaserScan message
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
      float x = parsedData[i * 2];     // x
      float y = parsedData[i * 2 + 1]; // y
      float range = std::sqrt(x * x + y * y);
      scan_msg->ranges[i] = range;
    }
    */
    publisher_->publish(std::move(scan_msg));
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);
  // 示例数据

  auto node = std::make_shared<LaserScanPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
