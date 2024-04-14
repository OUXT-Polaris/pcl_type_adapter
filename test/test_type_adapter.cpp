// Copyright 2024 OUXT Polaris. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <functional>
#include <pcl_type_adapter/pcl_type_adapter.hpp>

namespace pcl_type_adapter
{
template <typename PCL_POINTCLOUD_TYPE>
class PubNode : public rclcpp::Node
{
public:
  using AdaptedType = rclcpp::TypeAdapter<PCL_POINTCLOUD_TYPE, sensor_msgs::msg::PointCloud2>;
  explicit PubNode(const rclcpp::NodeOptions & options) : Node("test", options)
  {
    publisher_ = create_publisher<AdaptedType>("pointcloud", 1);
  }
  void publish(const PCL_POINTCLOUD_TYPE & point_cloud) { publisher_->publish(point_cloud); }

private:
  std::shared_ptr<rclcpp::Publisher<AdaptedType>> publisher_;
};

template <typename PCL_POINTCLOUD_TYPE>
class SubNode : public rclcpp::Node
{
public:
  using AdaptedType = rclcpp::TypeAdapter<PCL_POINTCLOUD_TYPE, sensor_msgs::msg::PointCloud2>;
  explicit SubNode(
    const rclcpp::NodeOptions & options,
    const std::function<void(const PCL_POINTCLOUD_TYPE &)> function)
  : Node("test", options)
  {
    subscriber_ = create_subscription<AdaptedType>("pointcloud", 1, function);
  }

private:
  std::shared_ptr<rclcpp::Subscription<AdaptedType>> subscriber_;
};

TEST(TypeAdaptaer, PointXYZ)
{
  bool point_cloud_recieved = false;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  const auto point_cloud = pcl::PointCloud<pcl::PointXYZ>();
  auto sub_node = std::make_shared<SubNode<pcl::PointCloud<pcl::PointXYZ>>>(
    options, [&](const pcl::PointCloud<pcl::PointXYZ> & point_cloud) {
      EXPECT_TRUE(&point_cloud == &point_cloud);
      point_cloud_recieved = true;
    });
  auto pub_node = std::make_shared<PubNode<pcl::PointCloud<pcl::PointXYZ>>>(options);
  pub_node->publish(point_cloud);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(sub_node);
  exec.add_node(pub_node);
  exec.spin_some();
  rclcpp::shutdown();
  EXPECT_TRUE(point_cloud_recieved);
}

#define DEFINE_PUB_SUB_TEST(POINT_TYPE)                                                   \
  TEST(TypeAdaptaer, POINT_TYPE)                                                          \
  {                                                                                       \
    bool point_cloud_recieved = false;                                                    \
    rclcpp::init(0, nullptr);                                                             \
    rclcpp::NodeOptions options;                                                          \
    options.use_intra_process_comms(true);                                                \
    const auto point_cloud = pcl::PointCloud<pcl::POINT_TYPE>();                          \
    auto sub_node = std::make_shared<SubNode<pcl::PointCloud<pcl::POINT_TYPE>>>(          \
      options, [&](const pcl::PointCloud<pcl::POINT_TYPE> & point_cloud) {                \
        EXPECT_TRUE(&point_cloud == &point_cloud);                                        \
        point_cloud_recieved = true;                                                      \
      });                                                                                 \
    auto pub_node = std::make_shared<PubNode<pcl::PointCloud<pcl::POINT_TYPE>>>(options); \
    pub_node->publish(point_cloud);                                                       \
    rclcpp::executors::SingleThreadedExecutor exec;                                       \
    exec.add_node(sub_node);                                                              \
    exec.add_node(pub_node);                                                              \
    exec.spin_some();                                                                     \
    rclcpp::shutdown();                                                                   \
    EXPECT_TRUE(point_cloud_recieved);                                                    \
  }

DEFINE_PUB_SUB_TEST(PointXYZI)

#undef DEFINE_PUB_SUB_TEST
}  // namespace pcl_type_adapter

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
