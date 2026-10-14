#include <moveit_whole_body_ik/SolveSampledWholeBodyIK.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace moveit_whole_body_ik {
class SampledWholeBodyIkServiceNode {
 public:
  SampledWholeBodyIkServiceNode() : nh_(), pnh_("~") {
    pnh_.param("group_name", group_name_, std::string("arm"));
    pnh_.param("collision_group_name", collision_group_name_,
                std::string("whole_body"));
    pnh_.param("planning_scene_topic", planning_scene_topic_,
                std::string("/planning_scene"));
    pnh_.param("base_link", base_link_, std::string("base_link"));
    pnh_.param("tip_link", tip_link_, std::string("arm_gripper_link"));
    pnh_.param("default_samples", samples_, 40);
    pnh_.param("default_radius", radius_, 1.0);
    pnh_.param("default_yaw_step", yaw_step_, 0.5235987756);
    pnh_.param("ik_attempt_timeout", ik_attempt_timeout_, 0.01);
    pnh_.param("arm_seed_count", arm_seed_count_, 5);
    pnh_.param("goal_contact_radius", goal_contact_radius_, 0.35);
    robot_model_ = robot_model_loader::RobotModelLoader("robot_description").getModel();
    if (!robot_model_) throw std::runtime_error("failed to load robot_description");
    group_ = robot_model_->getJointModelGroup(group_name_);
    if (!group_) throw std::runtime_error("planning group not found: " + group_name_);
    if (!robot_model_->getJointModelGroup(collision_group_name_))
      throw std::runtime_error("collision group not found: " +
                               collision_group_name_);
    KDL::Tree tree;
    if (!kdl_parser::treeFromParam("robot_description", tree) ||
        !tree.getChain(base_link_, tip_link_, kdl_chain_))
      throw std::runtime_error("failed to build KDL chain for visualization");
    ROS_INFO("Visualization KDL chain has %u segments", kdl_chain_.getNrOfSegments());
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_);
    service_ = nh_.advertiseService("/whole_body_moveit_ik/solve",
                                    &SampledWholeBodyIkServiceNode::solve, this);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(
        "/whole_body_moveit_ik/solution_joint_states", 1, true);
    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/whole_body_moveit_ik/target_pose", 1, true);
    // Publish a valid initial robot TF immediately so RViz can construct the
    // RobotModel before the first service request arrives.
    last_joints_.assign(group_->getVariableCount(), 0.0);
    has_solution_ = true;
    tf_timer_ = nh_.createTimer(ros::Duration(0.05),
                                &SampledWholeBodyIkServiceNode::publishLastTf, this);
    ROS_INFO("Sampled whole-body IK ready: /whole_body_moveit_ik/solve");
  }

 private:
  static Eigen::Quaterniond quat(const geometry_msgs::Pose& p) {
    return Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                              p.orientation.y, p.orientation.z).normalized();
  }

  void setBase(moveit::core::RobotState& state, double x, double y, double yaw) const {
    const std::vector<std::pair<std::string, double>> values = {
        {"virtual_joint/x", x}, {"virtual_joint/y", y},
        {"virtual_joint/theta", yaw}};
    for (const auto& value : values) {
      try { state.setVariablePosition(value.first, value.second); }
      catch (const std::exception&) { /* fixed-joint models have no planar variables */ }
    }
    state.update();
  }

  static double halton(unsigned int index, unsigned int base) {
    double fraction = 1.0;
    double value = 0.0;
    while (index > 0) {
      fraction /= static_cast<double>(base);
      value += fraction * static_cast<double>(index % base);
      index /= base;
    }
    return value;
  }

  void setDeterministicArmSeed(moveit::core::RobotState& state,
                               int seed_index) const {
    if (seed_index <= 0) return;
    static const unsigned int bases[] = {2, 3, 5, 7, 11, 13, 17, 19};
    const std::vector<std::string>& names = group_->getVariableNames();
    std::vector<double> seed(names.size(), 0.0);
    for (std::size_t i = 0; i < names.size(); ++i) {
      const moveit::core::VariableBounds& bounds =
          robot_model_->getVariableBounds(names[i]);
      const double unit = halton(static_cast<unsigned int>(seed_index),
                                 bases[i % (sizeof(bases) / sizeof(bases[0]))]);
      if (bounds.position_bounded_) {
        // Keep seeds away from exact joint limits, where numerical IK is less
        // likely to converge to a useful alternate elbow/wrist branch.
        seed[i] = bounds.min_position_ +
                  (0.05 + 0.90 * unit) *
                      (bounds.max_position_ - bounds.min_position_);
      } else {
        seed[i] = -M_PI + 2.0 * M_PI * unit;
      }
    }
    state.setJointGroupPositions(group_, seed);
    state.update();
  }

  bool hasBaseOctomapCollision(
      const planning_scene_monitor::LockedPlanningSceneRO& scene,
      moveit::core::RobotState& state) const {
    collision_detection::CollisionRequest request;
    request.group_name = collision_group_name_;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, state);
    if (!result.collision) return false;
    for (const auto& contact : result.contacts) {
      const std::string& first = contact.first.first;
      const std::string& second = contact.first.second;
      if ((first == base_link_ &&
           second == planning_scene::PlanningScene::OCTOMAP_NS) ||
          (second == base_link_ &&
           first == planning_scene::PlanningScene::OCTOMAP_NS)) {
        return true;
      }
    }
    return false;
  }

  bool onlyGoalLocalGripperContacts(
      const collision_detection::CollisionResult& result,
      const Eigen::Vector3d& target) const {
    if (!result.collision || result.contacts.empty()) return false;
    for (const auto& pair_contacts : result.contacts) {
      const std::string& first = pair_contacts.first.first;
      const std::string& second = pair_contacts.first.second;
      const bool gripper_octomap =
          (first == "arm_gripper_link" &&
           second == planning_scene::PlanningScene::OCTOMAP_NS) ||
          (second == "arm_gripper_link" &&
           first == planning_scene::PlanningScene::OCTOMAP_NS);
      if (!gripper_octomap) return false;
      for (const collision_detection::Contact& contact : pair_contacts.second) {
        if ((contact.pos - target).norm() > goal_contact_radius_) return false;
      }
    }
    return true;
  }

  bool solve(SolveSampledWholeBodyIK::Request& req,
             SolveSampledWholeBodyIK::Response& res) {
    const ros::WallTime start = ros::WallTime::now();
    if (req.reference_frame != "world" && !req.reference_frame.empty()) {
      res.message = "reference_frame must be world";
      return true;
    }
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(
          planning_scene_monitor_);
      if (!scene || !scene->getWorld()->hasObject(
                        planning_scene::PlanningScene::OCTOMAP_NS)) {
        res.success = false;
        res.message = "collision OctoMap is not loaded";
        res.solve_time_ms =
            (ros::WallTime::now() - start).toSec() * 1000.0;
        return true;
      }
    }
    const int n = req.base_samples > 0 ? req.base_samples : samples_;
    const double radius = req.base_radius > 0.0 ? req.base_radius : radius_;
    const double step = req.base_yaw_step > 0.0 ? req.base_yaw_step : yaw_step_;
    const Eigen::Vector3d target(req.target_pose.position.x,
                                 req.target_pose.position.y,
                                 req.target_pose.position.z);
    const Eigen::Quaterniond target_q = quat(req.target_pose);
    const double timeout = req.timeout_sec > 0.0 ? req.timeout_sec : 5.0;
    std::map<std::string, int> collision_pair_counts;
    struct BaseCandidate {
      double x;
      double y;
      double angle;
    };
    std::vector<BaseCandidate> base_candidates;
    base_candidates.reserve(n);
    for (int i = 0; i < n; ++i) {
      const double a = (i == 0) ? 0.0 : 2.0 * M_PI * (i - 1) / std::max(1, n - 1);
      const double r = (i == 0) ? 0.0 : radius * (0.35 + 0.65 * ((i % 5) / 4.0));
      // Sample the mobile base around the requested target, rather than
      // around world origin.  The arm IK then solves the residual local pose.
      const double bx = target.x() + r * std::cos(a);
      const double by = target.y() + r * std::sin(a);
      base_candidates.push_back({bx, by, a});
    }
    // Search the most likely yaw for every base position before widening the
    // yaw offset.  This avoids spending the whole request timeout rotating a
    // single bad base sample.
    const int yaw_count = std::max(
        1, static_cast<int>(std::ceil(2.0 * M_PI / step)));
    for (int yaw_index = 0; yaw_index < yaw_count; ++yaw_index) {
      for (const BaseCandidate& candidate : base_candidates) {
        const int yaw_offset = yaw_index == 0
                                   ? 0
                                   : ((yaw_index + 1) / 2) *
                                         ((yaw_index % 2) ? 1 : -1);
        const double bx = candidate.x;
        const double by = candidate.y;
        const double byaw = candidate.angle + M_PI + yaw_offset * step;
        // The planar virtual joint is already part of the MoveIt state.  Once
        // its base pose is set below, pass the requested pose directly; doing
        // an additional manual world->base rotation here would apply the
        // mobile-base transform twice.
        geometry_msgs::Pose local = req.target_pose;
        moveit::core::RobotState base_state(robot_model_);
        base_state.setToDefaultValues();
        setBase(base_state, bx, by, byaw);
        {
          planning_scene_monitor::LockedPlanningSceneRO scene(
              planning_scene_monitor_);
          if (!scene || hasBaseOctomapCollision(scene, base_state)) continue;
        }
        for (int seed_index = 0;
             seed_index < std::max(1, arm_seed_count_); ++seed_index) {
          moveit::core::RobotState state(robot_model_);
          state.setToDefaultValues();
          setDeterministicArmSeed(state, seed_index);
          setBase(state, bx, by, byaw);
          const double elapsed = (ros::WallTime::now() - start).toSec();
          if (elapsed >= timeout) break;
          const double remaining = timeout - elapsed;
          const double attempt_timeout =
              std::max(0.001, std::min(ik_attempt_timeout_, remaining));
          if (!state.setFromIK(group_, local, tip_link_, attempt_timeout))
            continue;
          setBase(state, bx, by, byaw);
          planning_scene_monitor::LockedPlanningSceneRO scene(
              planning_scene_monitor_);
          if (!scene)
            continue;
          collision_detection::CollisionRequest collision_request;
          collision_request.group_name = collision_group_name_;
          collision_request.contacts = true;
          collision_request.max_contacts = 1000;
          collision_request.max_contacts_per_pair = 1000;
          collision_detection::CollisionResult collision_result;
          scene->checkCollision(collision_request, collision_result, state);
          if (collision_result.collision) {
            if (onlyGoalLocalGripperContacts(collision_result, target)) {
              ROS_DEBUG("Accepting goal-local gripper contact within %.3f m",
                        goal_contact_radius_);
            } else {
            if (!collision_result.contacts.empty()) {
              const collision_detection::CollisionResult::ContactMap::key_type&
                  pair = collision_result.contacts.begin()->first;
              ++collision_pair_counts[pair.first + " vs " + pair.second];
            }
            continue;
            }
          }
          res.success = true; res.message = "sampled whole-body IK solved";
          res.base_x = bx; res.base_y = by; res.base_yaw = byaw;
          res.joint_names = group_->getVariableNames();
          state.copyJointGroupPositions(group_, res.joint_positions);
          sensor_msgs::JointState joint_msg;
          joint_msg.header.stamp = ros::Time::now();
          joint_msg.name = res.joint_names;
          joint_msg.position = res.joint_positions;
          joint_state_pub_.publish(joint_msg);
          geometry_msgs::PoseStamped target_msg;
          target_msg.header.stamp = joint_msg.header.stamp;
          target_msg.header.frame_id = "world";
          target_msg.pose = req.target_pose;
          target_pose_pub_.publish(target_msg);
          geometry_msgs::TransformStamped base_tf;
          base_tf.header.stamp = joint_msg.header.stamp;
          base_tf.header.frame_id = "world";
          base_tf.child_frame_id = "base_link";
          base_tf.transform.translation.x = bx;
          base_tf.transform.translation.y = by;
          base_tf.transform.rotation.z = std::sin(0.5 * byaw);
          base_tf.transform.rotation.w = std::cos(0.5 * byaw);
          tf_broadcaster_.sendTransform(base_tf);
          publishKdlTransforms(res.joint_positions, bx, by, byaw,
                               joint_msg.header.stamp);
          last_joints_ = res.joint_positions;
          last_bx_ = bx; last_by_ = by; last_byaw_ = byaw;
          has_solution_ = true;
          const Eigen::Isometry3d achieved =
              state.getGlobalLinkTransform(tip_link_);
          const Eigen::Vector3d world_target(target.x(), target.y(),
                                             target.z());
          res.position_error =
              (achieved.translation() - world_target).norm();
          res.orientation_error = target_q.angularDistance(
              Eigen::Quaterniond(achieved.rotation()));
          res.solve_time_ms =
              (ros::WallTime::now() - start).toSec() * 1000.0;
          return true;
        }
      }
    }
    res.success = false;
    res.message = "no collision-free sampled base and arm IK solution";
    if (!collision_pair_counts.empty()) {
      const auto dominant = std::max_element(
          collision_pair_counts.begin(), collision_pair_counts.end(),
          [](const auto& lhs, const auto& rhs) {
            return lhs.second < rhs.second;
          });
      res.message += "; dominant collision: " + dominant->first +
                     " (" + std::to_string(dominant->second) + ")";
    }
    res.solve_time_ms = (ros::WallTime::now() - start).toSec() * 1000.0;
    return true;
  }

  void publishLastTf(const ros::TimerEvent&) {
    if (has_solution_) publishKdlTransforms(last_joints_, last_bx_, last_by_,
                                            last_byaw_, ros::Time::now());
  }

  void publishKdlTransforms(const std::vector<double>& joints, double bx,
                            double by, double byaw, const ros::Time& stamp) {
    geometry_msgs::TransformStamped base;
    base.header.stamp = stamp; base.header.frame_id = "world";
    base.child_frame_id = base_link_; base.transform.translation.x = bx;
    base.transform.translation.y = by;
    base.transform.rotation.z = std::sin(0.5 * byaw);
    base.transform.rotation.w = std::cos(0.5 * byaw);
    tf_broadcaster_.sendTransform(base);
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    setBase(state, bx, by, byaw);
    state.setJointGroupPositions(group_, joints);
    state.update();
    publishLinkTransforms(state, stamp);
  }

  void publishLinkTransforms(const moveit::core::RobotState& state,
                             const ros::Time& stamp) {
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (const moveit::core::LinkModel* link : robot_model_->getLinkModels()) {
      // base_link is published explicitly as world->base_link above.  Do not
      // publish the virtual-joint relative transform a second time, otherwise
      // TF receives two competing transforms for the same child frame.
      if (link->getName() == base_link_) continue;
      const moveit::core::JointModel* parent_joint = link->getParentJointModel();
      const moveit::core::LinkModel* parent =
          parent_joint ? parent_joint->getParentLinkModel() : nullptr;
      if (!parent) continue;
      try {
        const Eigen::Isometry3d relative =
            state.getGlobalLinkTransform(parent).inverse() *
            state.getGlobalLinkTransform(link);
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = parent->getName();
        msg.child_frame_id = link->getName();
        msg.transform.translation.x = relative.translation().x();
        msg.transform.translation.y = relative.translation().y();
        msg.transform.translation.z = relative.translation().z();
        const Eigen::Quaterniond q(relative.rotation());
        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        msg.transform.rotation.w = q.w();
        transforms.push_back(msg);
      } catch (const std::exception& error) {
        ROS_WARN_THROTTLE(2.0, "Skipping TF for link %s: %s",
                          link->getName().c_str(), error.what());
      }
    }
    if (!transforms.empty()) tf_broadcaster_.sendTransform(transforms);
  }

  ros::NodeHandle nh_, pnh_; ros::ServiceServer service_;
  ros::Timer tf_timer_;
  ros::Publisher joint_state_pub_, target_pose_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* group_ = nullptr;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  KDL::Chain kdl_chain_;
  std::string group_name_, collision_group_name_, planning_scene_topic_;
  std::string base_link_, tip_link_;
  int samples_, arm_seed_count_; double radius_, yaw_step_;
  double ik_attempt_timeout_{0.01}, goal_contact_radius_{0.35};
  bool has_solution_{false}; double last_bx_{0.0}, last_by_{0.0}, last_byaw_{0.0};
  std::vector<double> last_joints_;
};
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sampled_whole_body_ik_service");
  try { moveit_whole_body_ik::SampledWholeBodyIkServiceNode node; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
