# pcl_type_adapter

ROS 2 package of type adapter for transferring point clouds via zero-copy communication.

## How to use.

### Publish pointcloud

```cpp
class PubNode : public rclcpp::Node
{
public:
  using AdaptedType = rclcpp::TypeAdapter<pcl::PointCloud::PointXYZ::Ptr, sensor_msgs::msg::PointCloud2>;
  explicit PubNode(const rclcpp::NodeOptions & options) : Node("test", options)
  {
    publisher_ = create_publisher<AdaptedType>("pointcloud", 1);
  }
  void publish(const pcl::PointCloud::PointXYZ::Ptr & point_cloud) { publisher_->publish(point_cloud); }

private:
  std::shared_ptr<rclcpp::Publisher<AdaptedType>> publisher_;
};
```

### Subscribe pointcloud

```cpp
class SubNode : public rclcpp::Node
{
public:
  using AdaptedType = rclcpp::TypeAdapter<pcl::PointCloud::PointXYZ::Ptr, sensor_msgs::msg::PointCloud2>;
  explicit SubNode(
    const rclcpp::NodeOptions & options)
  : Node("test", options)
  {
    subscriber_ = create_subscription<AdaptedType>("pointcloud", 1, [](const auto & pointcloud){});
  }

private:
  std::shared_ptr<rclcpp::Subscription<AdaptedType>> subscriber_;
};
```
