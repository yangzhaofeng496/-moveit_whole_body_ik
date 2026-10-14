#include <ros/ros.h>

#include <algorithm>
#include <cstdint>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace moveit_whole_body_ik {

class PcdOctomapPlanningSceneNode {
 public:
  PcdOctomapPlanningSceneNode()
      : nh_(),
        pnh_("~"),
        cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
        display_cloud_(new pcl::PointCloud<pcl::PointXYZ>()) {
    pnh_.param("pcd_file", pcd_file_, std::string());
    pnh_.param("frame_id", frame_id_, std::string("world"));
    pnh_.param("cloud_topic", cloud_topic_, std::string("/whole_body_ik/pcd_cloud"));
    pnh_.param("resolution", resolution_, 0.10);
    pnh_.param("translation_x", translation_x_, 0.0);
    pnh_.param("translation_y", translation_y_, 0.0);
    pnh_.param("translation_z", translation_z_, 0.0);
    pnh_.param("yaw", yaw_, 0.0);
    pnh_.param("filter_below_base_link", filter_below_base_link_, false);
    pnh_.param("base_link_z", base_link_z_, 0.0);
    pnh_.param("crop_to_workspace", crop_to_workspace_, true);
    pnh_.param("workspace_min_x", workspace_min_x_, -5.0);
    pnh_.param("workspace_max_x", workspace_max_x_, 5.0);
    pnh_.param("workspace_min_y", workspace_min_y_, -5.0);
    pnh_.param("workspace_max_y", workspace_max_y_, 5.0);
    pnh_.param("workspace_min_z", workspace_min_z_, -1.0);
    pnh_.param("workspace_max_z", workspace_max_z_, 5.0);
    pnh_.param("display_full_cloud", display_full_cloud_, true);
    pnh_.param("display_resolution", display_resolution_, 1.0);

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 1, true);
    planning_scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(
        "/planning_scene", 1, true);
    apply_client_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>(
        "/apply_planning_scene");

    if (pcd_file_.empty()) {
      ROS_FATAL("pcd_file is empty");
      ros::shutdown();
      return;
    }
    if (!loadPcd()) {
      ros::shutdown();
      return;
    }
    publishCloud();
    retry_timer_ = nh_.createTimer(ros::Duration(1.0),
                                   &PcdOctomapPlanningSceneNode::applyScene,
                                   this);
  }

 private:
  bool loadPcd() {
    octomap_.reset(new octomap::OcTree(resolution_));
    display_cloud_->clear();
    display_voxels_.clear();
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    pcl::PointCloud<pcl::PointXYZ> transformed;
    std::size_t raw_points = 0;

    // The archive's scans.pcd is a 1.3 GB binary PCD with 41 million points
    // and extra normal/intensity fields.  PCL's generic reader materializes
    // every point before cropping, which makes startup take several minutes.
    // Read only the x/y/z fields record-by-record and discard points outside
    // the configured planning workspace while streaming from disk.
    std::ifstream input(pcd_file_, std::ios::binary);
    std::string line;
    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<int> counts;
    std::size_t point_count = 0;
    std::streampos data_offset = std::streampos(-1);
    std::string data_mode;
    while (input && std::getline(input, line)) {
      std::istringstream stream(line);
      std::string key;
      stream >> key;
      if (key == "FIELDS") {
        std::string field;
        while (stream >> field) fields.push_back(field);
      } else if (key == "SIZE") {
        int value;
        while (stream >> value) sizes.push_back(value);
      } else if (key == "COUNT") {
        int value;
        while (stream >> value) counts.push_back(value);
      } else if (key == "POINTS") {
        stream >> point_count;
      } else if (key == "DATA") {
        stream >> data_mode;
        data_offset = input.tellg();
        break;
      }
    }

    const auto find_field = [&fields](const std::string& name) {
      for (std::size_t i = 0; i < fields.size(); ++i) {
        if (fields[i] == name) return i;
      }
      return fields.size();
    };
    const std::size_t x_field = find_field("x");
    const std::size_t y_field = find_field("y");
    const std::size_t z_field = find_field("z");
    const bool has_binary_xyz =
        input && data_mode == "binary" && data_offset != std::streampos(-1) &&
        x_field < fields.size() && y_field < fields.size() &&
        z_field < fields.size() && sizes.size() == fields.size();

    if (has_binary_xyz) {
      if (counts.size() != fields.size()) counts.assign(fields.size(), 1);
      std::vector<std::size_t> offsets(fields.size(), 0);
      std::size_t point_step = 0;
      for (std::size_t i = 0; i < fields.size(); ++i) {
        offsets[i] = point_step;
        point_step += static_cast<std::size_t>(sizes[i]) * counts[i];
      }
      if (point_step == 0 || sizes[x_field] != 4 || sizes[y_field] != 4 ||
          sizes[z_field] != 4 || counts[x_field] != 1 ||
          counts[y_field] != 1 || counts[z_field] != 1) {
        data_offset = std::streampos(-1);
      } else {
        input.clear();
        input.seekg(data_offset);
        std::vector<char> record(point_step);
        transformed.points.reserve(std::min<std::size_t>(point_count, 1000000));
        for (std::size_t i = 0; i < point_count && input.read(record.data(), record.size()); ++i) {
          float source_x, source_y, source_z;
          std::memcpy(&source_x, record.data() + offsets[x_field], sizeof(float));
          std::memcpy(&source_y, record.data() + offsets[y_field], sizeof(float));
          std::memcpy(&source_z, record.data() + offsets[z_field], sizeof(float));
          ++raw_points;
          if (!std::isfinite(source_x) || !std::isfinite(source_y) ||
              !std::isfinite(source_z)) continue;
          const double x = c * source_x - s * source_y + translation_x_;
          const double y = s * source_x + c * source_y + translation_y_;
          const double z = source_z + translation_z_;
          if (isBelowBaseLinkPlane(z)) continue;
          if (crop_to_workspace_ &&
              (x < workspace_min_x_ || x > workspace_max_x_ ||
               y < workspace_min_y_ || y > workspace_max_y_ ||
               z < workspace_min_z_ || z > workspace_max_z_)) continue;
          addDisplayPoint(x, y, z);
          transformed.emplace_back(static_cast<float>(x), static_cast<float>(y),
                                   static_cast<float>(z));
        }
      }
    }

    if (!has_binary_xyz || data_offset == std::streampos(-1)) {
      cloud_->clear();
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *cloud_) != 0) {
        ROS_ERROR("Unable to load PCD file: %s", pcd_file_.c_str());
        return false;
      }
      raw_points = cloud_->size();
      transformed.points.reserve(cloud_->size());
      for (const auto& point : cloud_->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) continue;
        const double x = c * point.x - s * point.y + translation_x_;
        const double y = s * point.x + c * point.y + translation_y_;
        const double z = point.z + translation_z_;
        if (isBelowBaseLinkPlane(z)) continue;
        if (crop_to_workspace_ &&
            (x < workspace_min_x_ || x > workspace_max_x_ ||
             y < workspace_min_y_ || y > workspace_max_y_ ||
             z < workspace_min_z_ || z > workspace_max_z_)) continue;
        addDisplayPoint(x, y, z);
        transformed.emplace_back(static_cast<float>(x), static_cast<float>(y),
                                 static_cast<float>(z));
      }
    }
    if (transformed.empty()) {
      ROS_ERROR("PCD file contains no points in the configured workspace: %s",
                pcd_file_.c_str());
      return false;
    }
    transformed.width = static_cast<std::uint32_t>(transformed.size());
    transformed.height = 1;
    // The source map is 1 cm resolution.  Reduce it to the OctoMap
    // resolution before inserting nodes so loading remains bounded by the
    // collision-map resolution rather than the raw PCD density.
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(transformed.makeShared());
    voxel_filter.setLeafSize(static_cast<float>(resolution_),
                             static_cast<float>(resolution_),
                             static_cast<float>(resolution_));
    pcl::PointCloud<pcl::PointXYZ> filtered;
    voxel_filter.filter(filtered);
    *cloud_ = filtered;
    for (const auto& point : cloud_->points) {
      octomap_->updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }
    octomap_->updateInnerOccupancy();
    // Serialize the compact max-likelihood tree.  Without pruning, the
    // large mapping archive retains every internal node and can spend minutes
    // in OctoMap's binary writer even after voxel filtering.
    octomap_->toMaxLikelihood();
    octomap_->prune();

    octomap_msgs::Octomap octomap_msg;
    if (!octomap_msgs::binaryMapToMsg(*octomap_, octomap_msg)) {
      ROS_ERROR("Failed to serialize PCD into an OctoMap");
      return false;
    }
    octomap_msg.header.frame_id = frame_id_;
    octomap_msg.header.stamp = ros::Time::now();
    scene_.is_diff = true;
    scene_.robot_state.is_diff = true;
    scene_.world.octomap.header.frame_id = frame_id_;
    scene_.world.octomap.header.stamp = octomap_msg.header.stamp;
    scene_.world.octomap.octomap = octomap_msg;
    scene_.world.octomap.origin = geometry_msgs::Pose();
    scene_.world.octomap.origin.orientation.w = 1.0;
    ROS_INFO("Loaded %zu/%zu cropped PCD points into OctoMap: %s "
             "(resolution %.3f m); RViz display points=%zu (full=%s, "
             "display resolution %.3f m)",
             cloud_->size(), raw_points, pcd_file_.c_str(), resolution_,
             display_cloud_->size(), display_full_cloud_ ? "true" : "false",
             display_resolution_);
    return true;
  }

  void publishCloud() {
    sensor_msgs::PointCloud2 message;
    const pcl::PointCloud<pcl::PointXYZ>& points =
        display_full_cloud_ ? *display_cloud_ : *cloud_;
    pcl::toROSMsg(points, message);
    message.header.stamp = ros::Time::now();
    message.header.frame_id = frame_id_;
    cloud_pub_.publish(message);
  }

  bool isBelowBaseLinkPlane(double z) const {
    return filter_below_base_link_ && z < base_link_z_;
  }

  struct DisplayVoxelKey {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;
    bool operator==(const DisplayVoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct DisplayVoxelKeyHash {
    std::size_t operator()(const DisplayVoxelKey& key) const {
      const std::size_t h1 = std::hash<std::int64_t>{}(key.x);
      const std::size_t h2 = std::hash<std::int64_t>{}(key.y);
      const std::size_t h3 = std::hash<std::int64_t>{}(key.z);
      return h1 ^ (h2 + static_cast<std::size_t>(0x9e3779b9) +
                   (h1 << 6) + (h1 >> 2)) ^
             (h3 + static_cast<std::size_t>(0x9e3779b9) +
              (h2 << 6) + (h2 >> 2));
    }
  };

  void addDisplayPoint(double x, double y, double z) {
    if (!display_full_cloud_ || display_resolution_ <= 0.0 ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return;
    }
    const double inverse = 1.0 / display_resolution_;
    const DisplayVoxelKey key{
        static_cast<std::int64_t>(std::floor(x * inverse)),
        static_cast<std::int64_t>(std::floor(y * inverse)),
        static_cast<std::int64_t>(std::floor(z * inverse))};
    if (display_voxels_.insert(key).second) {
      display_cloud_->emplace_back(static_cast<float>(x), static_cast<float>(y),
                                   static_cast<float>(z));
    }
  }

  void applyScene(const ros::TimerEvent&) {
    // move_group always subscribes to /planning_scene.  Publishing a latched
    // diff makes this work even when the package's optional
    // /apply_planning_scene service is not advertised.
    planning_scene_pub_.publish(scene_);
    if (!apply_client_.exists()) return;
    moveit_msgs::ApplyPlanningScene service;
    service.request.scene = scene_;
    if (apply_client_.call(service) && service.response.success) {
      ROS_INFO("Applied PCD OctoMap to MoveIt PlanningScene");
      retry_timer_.stop();
    } else {
      ROS_WARN_THROTTLE(5.0, "MoveIt rejected PCD OctoMap PlanningScene update");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cloud_pub_;
  ros::Publisher planning_scene_pub_;
  ros::ServiceClient apply_client_;
  ros::Timer retry_timer_;
  std::string pcd_file_;
  std::string frame_id_;
  std::string cloud_topic_;
  double resolution_ = 0.10;
  double translation_x_ = 0.0;
  double translation_y_ = 0.0;
  double translation_z_ = 0.0;
  double yaw_ = 0.0;
  bool filter_below_base_link_ = false;
  double base_link_z_ = 0.0;
  bool crop_to_workspace_ = true;
  double workspace_min_x_ = -5.0;
  double workspace_max_x_ = 5.0;
  double workspace_min_y_ = -5.0;
  double workspace_max_y_ = 5.0;
  double workspace_min_z_ = -1.0;
  double workspace_max_z_ = 5.0;
  bool display_full_cloud_ = true;
  double display_resolution_ = 1.0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr display_cloud_;
  std::unordered_set<DisplayVoxelKey, DisplayVoxelKeyHash> display_voxels_;
  std::unique_ptr<octomap::OcTree> octomap_;
  moveit_msgs::PlanningScene scene_;
};

}  // namespace moveit_whole_body_ik

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcd_octomap_planning_scene");
  moveit_whole_body_ik::PcdOctomapPlanningSceneNode node;
  ros::spin();
  return 0;
}
