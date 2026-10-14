#include <moveit_whole_body_ik/SolveMoveItIK.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace moveit_whole_body_ik {

class MoveItIkServiceNode {
 public:
  MoveItIkServiceNode() : nh_(), pnh_("~") {
    pnh_.param("group_name", group_name_, std::string("arm"));
    pnh_.param("base_link", base_link_, std::string("base_link"));
    pnh_.param("tip_link", tip_link_, std::string("arm_gripper_link"));
    pnh_.param("default_timeout_sec", default_timeout_sec_, 5.0);
    pnh_.param("planning_scene_service", planning_scene_service_,
               std::string("/move_group/get_planning_scene"));

    robot_model_loader::RobotModelLoader loader("robot_description");
    model_ = loader.getModel();
    if (!model_) {
      throw std::runtime_error("failed to load robot_description");
    }
    joint_group_ = model_->getJointModelGroup(group_name_);
    if (!joint_group_) {
      throw std::runtime_error("planning group not found: " + group_name_);
    }
    if (!model_->getLinkModel(tip_link_)) {
      throw std::runtime_error("tip link not found: " + tip_link_);
    }
    if (!model_->getLinkModel(base_link_)) {
      throw std::runtime_error("base link not found: " + base_link_);
    }

    service_ = nh_.advertiseService(
        "/moveit_ik/solve", &MoveItIkServiceNode::solveCallback, this);
    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/moveit_ik/target_pose", 1, true);
    solution_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(
        "/moveit_ik/solution_joint_states", 1, true);
    moveit::core::RobotState initial_state(model_);
    initial_state.setToDefaultValues();
    sensor_msgs::JointState initial_message;
    initial_message.header.stamp = ros::Time::now();
    initial_message.name = joint_group_->getVariableNames();
    initial_state.copyJointGroupPositions(joint_group_, initial_message.position);
    solution_joint_state_pub_.publish(initial_message);
    ROS_INFO("MoveIt IK service ready: /moveit_ik/solve (%s -> %s)",
             base_link_.c_str(), tip_link_.c_str());

    planning_scene_monitor_.reset(
        new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    planning_scene_monitor_->startSceneMonitor(
        "/move_group/monitored_planning_scene");
    // The monitored scene topic is sufficient; do not block on a
    // version-dependent get_planning_scene service name.
  }

 private:
  static Eigen::Quaterniond poseQuaternion(const geometry_msgs::Pose& pose) {
    const Eigen::Quaterniond input(pose.orientation.w, pose.orientation.x,
                                   pose.orientation.y, pose.orientation.z);
    return input.normalized();
  }

  bool solveCallback(moveit_whole_body_ik::SolveMoveItIK::Request& request,
                     moveit_whole_body_ik::SolveMoveItIK::Response& response) {
    const std::string frame = request.reference_frame.empty()
                                  ? base_link_
                                  : request.reference_frame;
    if (frame != base_link_) {
      response.message = "reference_frame must be " + base_link_;
      return true;
    }
    const Eigen::Quaterniond input_orientation(
        request.target_pose.orientation.w, request.target_pose.orientation.x,
        request.target_pose.orientation.y, request.target_pose.orientation.z);
    if (!std::isfinite(input_orientation.squaredNorm()) ||
        input_orientation.squaredNorm() < 1.0e-12 ||
        !std::isfinite(request.target_pose.position.x) ||
        !std::isfinite(request.target_pose.position.y) ||
        !std::isfinite(request.target_pose.position.z)) {
      response.message = "target pose is non-finite or has a zero quaternion";
      return true;
    }
    // const Eigen::Quaterniond target_orientation =
    //     poseQuaternion(request.target_pose);

    geometry_msgs::Pose adjusted_pose = request.target_pose;

    // 让 arm_gripper_link 的局部 +X 轴与原目标的局部 +Z 轴重合
    const Eigen::Quaterniond x_axis_alignment(
        Eigen::AngleAxisd(-M_PI / 2.0,
                          Eigen::Vector3d::UnitY()));

    const Eigen::Quaterniond target_orientation =
        (input_orientation * x_axis_alignment).normalized();

    adjusted_pose.orientation.x = target_orientation.x();
    adjusted_pose.orientation.y = target_orientation.y();
    adjusted_pose.orientation.z = target_orientation.z();
    adjusted_pose.orientation.w = target_orientation.w();

    geometry_msgs::PoseStamped target_message;
    target_message.header.stamp = ros::Time::now();
    target_message.header.frame_id = frame;
    target_message.pose = adjusted_pose;
    target_pose_pub_.publish(target_message);

    const double timeout = request.timeout_sec > 0.0
                               ? request.timeout_sec
                               : default_timeout_sec_;
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    const ros::WallTime start = ros::WallTime::now();
    const bool success = state.setFromIK(
        joint_group_, adjusted_pose, tip_link_, timeout);
    response.solve_time_ms =
        (ros::WallTime::now() - start).toSec() * 1000.0;
    response.joint_names = joint_group_->getVariableNames();
    state.copyJointGroupPositions(joint_group_, response.joint_positions);
    response.success = success;

    ROS_INFO("MoveIt IK target (%s): position=(%.6f, %.6f, %.6f), "
             "quaternion=(%.6f, %.6f, %.6f, %.6f)",
             frame.c_str(), request.target_pose.position.x,
             request.target_pose.position.y, request.target_pose.position.z,
             request.target_pose.orientation.x, request.target_pose.orientation.y,
             request.target_pose.orientation.z, request.target_pose.orientation.w);
    for (size_t i = 0; i < response.joint_names.size() &&
                       i < response.joint_positions.size();
         ++i) {
      ROS_INFO("MoveIt IK result: %s=%.9f", response.joint_names[i].c_str(),
               response.joint_positions[i]);
    }

    if (!success) {
      response.message = "MoveIt IK failed";
      return true;
    }

    {
      planning_scene_monitor::LockedPlanningSceneRO scene(
          planning_scene_monitor_);
      if (!scene) {
        response.success = false;
        response.message = "MoveIt PlanningScene is not available";
        return true;
      }
      if (scene->isStateColliding(state, group_name_)) {
        response.success = false;
        response.message = "IK solution is in collision";
        ROS_WARN("MoveIt IK solution rejected because of collision");
        return true;
      }
    }

    sensor_msgs::JointState joint_message;
    joint_message.header.stamp = ros::Time::now();
    joint_message.name = response.joint_names;
    joint_message.position = response.joint_positions;
    solution_joint_state_pub_.publish(joint_message);

    const Eigen::Isometry3d achieved = state.getGlobalLinkTransform(tip_link_);
    const Eigen::Vector3d target_position(request.target_pose.position.x,
                                           request.target_pose.position.y,
                                           request.target_pose.position.z);
    response.position_error =
        (achieved.translation() - target_position).norm();
    response.orientation_error =
        target_orientation.angularDistance(Eigen::Quaterniond(achieved.rotation()));
    response.message = "MoveIt IK solved";
    ROS_INFO("MoveIt IK solved in %.3f ms: position_error=%.6g, "
             "orientation_error=%.6g rad",
             response.solve_time_ms, response.position_error,
             response.orientation_error);
    return true;
  }


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::ServiceServer service_;
  ros::Publisher target_pose_pub_;
  ros::Publisher solution_joint_state_pub_;
  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup* joint_group_ = nullptr;
  std::string group_name_;
  std::string base_link_;
  std::string tip_link_;
  std::string planning_scene_service_ = "/move_group/get_planning_scene";
  double default_timeout_sec_ = 5.0;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};

}  // namespace moveit_whole_body_ik

int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_ik_service_node");
  try {
    moveit_whole_body_ik::MoveItIkServiceNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("MoveIt IK service failed to start: %s", error.what());
    return 1;
  }
  return 0;
}
