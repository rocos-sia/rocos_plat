#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("publisher_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_topic", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PublisherNode::publishPointCloud, this));
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
  void publishPointCloud()
  {
    auto pointCloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pointCloudMsg->header.stamp = this->now();
    pointCloudMsg->header.frame_id = "base_link";
    pointCloudMsg->height = 1;
    pointCloudMsg->width = 180*2;
    pointCloudMsg->is_dense = true;

    // Set point cloud fields
    pointCloudMsg->fields.resize(3);
    pointCloudMsg->fields[0].name = "x";
    pointCloudMsg->fields[0].offset = 0;
    pointCloudMsg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointCloudMsg->fields[0].count = 1;
    pointCloudMsg->fields[1].name = "y";
    pointCloudMsg->fields[1].offset = 4;
    pointCloudMsg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointCloudMsg->fields[1].count = 1;
    pointCloudMsg->fields[2].name = "z";
    pointCloudMsg->fields[2].offset = 8;
    pointCloudMsg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointCloudMsg->fields[2].count = 1;

    // Resize the point cloud data
    pointCloudMsg->point_step = 12;
    pointCloudMsg->row_step = pointCloudMsg->point_step * pointCloudMsg->width;
    pointCloudMsg->data.resize(pointCloudMsg->row_step);

    // tcp接收数据
    receivedData = receiveDataFromServer(SERVER_IP, SERVER_PORT, hexQuery);
    anaylazeData = {receivedData.begin() + 10, receivedData.end()};
    parseTCPData(anaylazeData, parsedData);

    // Set point cloud data
    float *data = reinterpret_cast<float *>(pointCloudMsg->data.data());
    for (size_t i = 0; i < pointCloudMsg->width; ++i)
    {
      data[i * 3] = parsedData[i*2];     // x
      data[i * 3 + 1] = parsedData[i*2+1]; // y
      data[i * 3 + 2] = 0;                     // z
    }

    publisher_->publish(*pointCloudMsg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);
  // 示例数据

  auto node = std::make_shared<PublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
