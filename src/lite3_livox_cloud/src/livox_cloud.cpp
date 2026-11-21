#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <utility>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using Scan = livox_ros_driver2::msg::CustomMsg;
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;

Field field(const char *name, uint32_t offset, uint8_t type) {
  Field f;
  f.name = name;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  return f;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lite3_livox_cloud");
  auto output = node->create_publisher<Cloud>("/lite3/livox_points", 5);
  auto input = node->create_subscription<Scan>(
      "/lite3/livox_lidar", rclcpp::SensorDataQoS().keep_last(2),
      [output, logger = node->get_logger()](Scan::ConstSharedPtr scan) {
        // Match the deployed driver's packed 22-byte layout, including invalid returns.
        constexpr uint32_t stride = 22;
        if (scan->point_num != scan->points.size() ||
            scan->point_num > std::numeric_limits<uint32_t>::max() / stride) {
          RCLCPP_WARN(logger, "Ignoring a Livox scan with inconsistent point count");
          return;
        }
        auto cloud = std::make_unique<Cloud>();
        cloud->header = scan->header;
        cloud->height = 1;
        cloud->width = scan->point_num;
        cloud->fields = {field("x", 0, Field::FLOAT32), field("y", 4, Field::FLOAT32),
                         field("z", 8, Field::FLOAT32), field("t", 12, Field::UINT32),
                         field("intensity", 16, Field::FLOAT32), field("tag", 20, Field::UINT8),
                         field("line", 21, Field::UINT8)};
        const uint16_t endian_probe = 1;
        cloud->is_bigendian = *reinterpret_cast<const uint8_t*>(&endian_probe) == 0;
        cloud->is_dense = true;
        cloud->point_step = stride;
        cloud->row_step = stride * cloud->width;
        cloud->data.resize(cloud->row_step);
        auto dest = cloud->data.data();
        for (const auto &p : scan->points) {
          const float intensity = p.reflectivity;
          std::memcpy(dest, &p.x, 4);
          std::memcpy(dest + 4, &p.y, 4);
          std::memcpy(dest + 8, &p.z, 4);
          std::memcpy(dest + 12, &p.offset_time, 4);
          std::memcpy(dest + 16, &intensity, 4);
          dest[20] = p.tag;
          dest[21] = p.line;
          dest += stride;
        }
        output->publish(std::move(cloud));
      });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
