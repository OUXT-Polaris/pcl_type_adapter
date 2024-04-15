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

#ifndef PCL_TYPE_ADAPTER__PCL_TYPE_ADAPTER_HPP_
#define PCL_TYPE_ADAPTER__PCL_TYPE_ADAPTER_HPP_

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

template <typename PCL_POINTCLOUD_TYPE>
struct rclcpp::TypeAdapter<std::shared_ptr<PCL_POINTCLOUD_TYPE>, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = std::shared_ptr<PCL_POINTCLOUD_TYPE>;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    pcl::toROSMsg(*source, destination);
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    pcl::fromROSMsg(source, *destination);
  }
};

#endif  // PCL_TYPE_ADAPTER__PCL_TYPE_ADAPTER_HPP_
