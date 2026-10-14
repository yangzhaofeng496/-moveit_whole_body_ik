#include <moveit_whole_body_ik/MoveToPose.h>

#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <ros/ros.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace moveit_whole_body_ik {

class MoveToPoseServiceNode {
 public:
  MoveToPoseServiceNode() : nh_(), pnh_("~") {
    pnh_.param("group_name", group_name_, std::string("arm"));
    pnh_.param("reference_frame", reference_frame_,
               std::string("base_link"));
    pnh_.param("tip_link", tip_link_,
               std::string("arm_gripper_link"));
    pnh_.param("planning_time", planning_time_, 5.0);

    // true：规划成功后立即执行；
    // false：只进行规划，不执行机械臂运动。
    pnh_.param("execute_after_plan", execute_after_plan_, true);

    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(
        group_name_));
    move_group_->setPoseReferenceFrame(reference_frame_);
    move_group_->setEndEffectorLink(tip_link_);
    move_group_->setPlanningTime(planning_time_);

    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/whole_body_moveit_ik/target_pose", 1, true);

    move_group_status_sub_ = nh_.subscribe(
        "/move_group/status", 10,
        &MoveToPoseServiceNode::moveGroupStatusCallback, this);
    move_group_result_sub_ = nh_.subscribe(
        "/move_group/result", 10,
        &MoveToPoseServiceNode::moveGroupResultCallback, this);

    service_ = nh_.advertiseService(
        "/move_to_pose", &MoveToPoseServiceNode::moveCallback, this);

    ROS_INFO("Move-to-pose service ready: /move_to_pose (%s, %s -> %s)",
             group_name_.c_str(), reference_frame_.c_str(),
             tip_link_.c_str());
    ROS_INFO("execute_after_plan: %s",
             execute_after_plan_ ? "true" : "false");
  }

 private:
  static geometry_msgs::PoseStamped displayPoseWithZAxisArrow(
      const geometry_msgs::PoseStamped& target) {
    geometry_msgs::PoseStamped display_target = target;
    const Eigen::Quaterniond target_orientation(
        target.pose.orientation.w, target.pose.orientation.x,
        target.pose.orientation.y, target.pose.orientation.z);

    // RViz 的 Pose/Arrow 显示沿局部 +X 轴，旋转显示方向使箭头表示目标
    // 位姿的局部 +Z 轴。该变换只影响显示，不影响实际运动目标。
    const Eigen::Quaterniond x_axis_to_z_axis(Eigen::AngleAxisd(
        -1.5707963267948966, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond display_orientation =
        (target_orientation * x_axis_to_z_axis).normalized();

    display_target.pose.orientation.x = display_orientation.x();
    display_target.pose.orientation.y = display_orientation.y();
    display_target.pose.orientation.z = display_orientation.z();
    display_target.pose.orientation.w = display_orientation.w();
    return display_target;
  }

  void moveGroupStatusCallback(
      const actionlib_msgs::GoalStatusArrayConstPtr& message) {
    if (message->status_list.empty()) {
      return;
    }

    const actionlib_msgs::GoalStatus& status = message->status_list.back();
    if (status.status == actionlib_msgs::GoalStatus::PENDING ||
        status.status == actionlib_msgs::GoalStatus::ACTIVE) {
      ROS_INFO_THROTTLE(1.0,
                        "move_group 正在执行: status=%u (%s), text=%s",
                        static_cast<unsigned int>(status.status),
                        statusName(status.status).c_str(),
                        status.text.c_str());
    }
  }

  void moveGroupResultCallback(
      const moveit_msgs::MoveGroupActionResultConstPtr& message) {
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      last_result_received_ = true;
      last_result_status_ = message->status.status;
      last_result_error_code_ = message->result.error_code.val;
      last_result_text_ = message->status.text;
    }

    ROS_INFO("move_group 返回结果: status=%u (%s), error_code=%d, text=%s",
             static_cast<unsigned int>(message->status.status),
             statusName(message->status.status).c_str(),
             message->result.error_code.val, message->status.text.c_str());
  }

  static std::string statusName(uint8_t status) {
    switch (status) {
      case actionlib_msgs::GoalStatus::PENDING:
        return "PENDING";
      case actionlib_msgs::GoalStatus::ACTIVE:
        return "ACTIVE";
      case actionlib_msgs::GoalStatus::PREEMPTED:
        return "PREEMPTED";
      case actionlib_msgs::GoalStatus::SUCCEEDED:
        return "SUCCEEDED";
      case actionlib_msgs::GoalStatus::ABORTED:
        return "ABORTED";
      case actionlib_msgs::GoalStatus::REJECTED:
        return "REJECTED";
      case actionlib_msgs::GoalStatus::PREEMPTING:
        return "PREEMPTING";
      case actionlib_msgs::GoalStatus::RECALLING:
        return "RECALLING";
      case actionlib_msgs::GoalStatus::RECALLED:
        return "RECALLED";
      case actionlib_msgs::GoalStatus::LOST:
        return "LOST";
      default:
        return "UNKNOWN";
    }
  }

  bool moveCallback(moveit_whole_body_ik::MoveToPose::Request& request,
                    moveit_whole_body_ik::MoveToPose::Response& response) {
    const double quaternion_norm = std::sqrt(
        request.qx * request.qx + request.qy * request.qy +
        request.qz * request.qz + request.qw * request.qw);

    if (!std::isfinite(request.x) || !std::isfinite(request.y) ||
        !std::isfinite(request.z) || !std::isfinite(request.qx) ||
        !std::isfinite(request.qy) || !std::isfinite(request.qz) ||
        !std::isfinite(request.qw) || !std::isfinite(quaternion_norm) ||
        quaternion_norm < 1.0e-12) {
      response.success = false;
      response.message = "target position or quaternion is invalid";
      return true;
    }

    geometry_msgs::PoseStamped target;
    target.header.frame_id = reference_frame_;
    target.header.stamp = ros::Time::now();
    target.pose.position.x = request.x;
    target.pose.position.y = request.y;
    target.pose.position.z = request.z;
    // 归一化输入四元数。
    const Eigen::Quaterniond input_orientation(
        request.qw / quaternion_norm,
        request.qx / quaternion_norm,
        request.qy / quaternion_norm,
        request.qz / quaternion_norm);

    // 暂时取消所有额外姿态变换，直接使用输入目标姿态。
    const Eigen::Quaterniond adjusted_orientation = input_orientation;

    target.pose.orientation.x = adjusted_orientation.x();
    target.pose.orientation.y = adjusted_orientation.y();
    target.pose.orientation.z = adjusted_orientation.z();
    target.pose.orientation.w = adjusted_orientation.w();

    // 发布实际用于规划的目标姿态。
    target_pose_pub_.publish(target);

    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();
    if (!move_group_->setPoseTarget(target, tip_link_)) {
      response.success = false;
      response.message = "MoveIt rejected the pose target";
      return true;
    }

    ROS_INFO("Planning pose target for tip [%s]: "
             "position=(%.4f, %.4f, %.4f), "
             "adjusted quaternion=(%.4f, %.4f, %.4f, %.4f)",
             tip_link_.c_str(),
             target.pose.position.x, target.pose.position.y,
             target.pose.position.z, target.pose.orientation.x,
             target.pose.orientation.y, target.pose.orientation.z,
             target.pose.orientation.w);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const moveit::core::MoveItErrorCode plan_result =
        move_group_->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->clearPoseTargets();
      response.success = false;
      response.message = "MoveIt planning failed";
      ROS_WARN("MoveIt 规划失败，未执行目标位姿");
      return true;
    }

    ROS_INFO("MoveIt 规划成功");

    if (!execute_after_plan_) {
      move_group_->clearPoseTargets();
      response.success = true;
      response.message =
          "MoveIt planning succeeded, execution skipped";
      ROS_INFO("execute_after_plan=false，仅规划，不执行");
      return true;
    }

    const moveit::core::MoveItErrorCode execute_result =
        move_group_->execute(plan);

    move_group_->clearPoseTargets();
    response.success =
        execute_result == moveit::core::MoveItErrorCode::SUCCESS;
    response.message = response.success
                           ? "MoveIt planning and execution succeeded"
                           : "MoveIt execution failed";

    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::ServiceServer service_;
  ros::Publisher target_pose_pub_;
  ros::Subscriber move_group_status_sub_;
  ros::Subscriber move_group_result_sub_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::mutex result_mutex_;
  bool last_result_received_ = false;
  uint8_t last_result_status_ = actionlib_msgs::GoalStatus::LOST;
  int32_t last_result_error_code_ = 0;
  std::string last_result_text_;
  std::string group_name_;
  std::string reference_frame_;
  std::string tip_link_;
  double planning_time_ = 5.0;
  bool execute_after_plan_ = true;
};

}  // namespace moveit_whole_body_ik

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_to_pose_service_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  try {
    moveit_whole_body_ik::MoveToPoseServiceNode node;
    ros::waitForShutdown();
  } catch (const std::exception& error) {
    ROS_FATAL("Move-to-pose service failed to start: %s", error.what());
    return 1;
  }

  return 0;
}
