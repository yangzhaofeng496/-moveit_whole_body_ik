#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace {
constexpr std::array<const char*, 6> kArmJoints = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
constexpr double kControlPeriodSeconds = 0.02;
}

class GazeboFollowJointTrajectory {
 public:
  GazeboFollowJointTrajectory()
      : private_nh_("~"),
        action_server_(nh_, "/arm_controller/follow_joint_trajectory",
                       boost::bind(&GazeboFollowJointTrajectory::execute, this, _1), false) {
    private_nh_.param("model_name", model_name_, std::string("robot"));
    private_nh_.param("urdf_param_name", urdf_param_name_,
                      std::string("robot_description"));
    private_nh_.param("publish_joint_states", publish_joint_states_, true);
    std::vector<double> initial_positions;
    if (private_nh_.getParam("initial_positions", initial_positions)) {
      if (initial_positions.size() != kArmJoints.size() ||
          !std::all_of(initial_positions.begin(), initial_positions.end(),
                       [](double position) { return std::isfinite(position); })) {
        ROS_WARN("Ignoring initial_positions: expected six finite joint angles");
      } else {
        std::copy(initial_positions.begin(), initial_positions.end(),
                  current_positions_.begin());
      }
    }
    configuration_client_ = nh_.serviceClient<gazebo_msgs::SetModelConfiguration>(
        "/gazebo/set_model_configuration", false);
    if (publish_joint_states_) {
      // This is a live state stream, not a latched state. A latched zero-time
      // sample would be replayed to MoveIt when it subscribes after startup.
      joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10, false);
    }
    state_timer_ = nh_.createWallTimer(ros::WallDuration(kControlPeriodSeconds),
                                       &GazeboFollowJointTrajectory::publishCurrentState, this);
    action_server_.start();
    publishState(current_positions_);
  }

 private:
  using Action = control_msgs::FollowJointTrajectoryAction;
  using GoalConstPtr = control_msgs::FollowJointTrajectoryGoalConstPtr;

  bool validGoal(const GoalConstPtr& goal, std::string& reason) const {
    const auto& trajectory = goal->trajectory;
    if (trajectory.joint_names.size() != kArmJoints.size()) {
      reason = "trajectory must contain exactly the six arm joints";
      return false;
    }
    for (std::size_t index = 0; index < kArmJoints.size(); ++index) {
      if (trajectory.joint_names[index] != kArmJoints[index]) {
        reason = "trajectory joint order must be joint1 through joint6";
        return false;
      }
    }
    if (trajectory.points.empty()) {
      reason = "trajectory contains no points";
      return false;
    }
    ros::Duration previous_time(0.0);
    for (const auto& point : trajectory.points) {
      if (point.positions.size() != kArmJoints.size()) {
        reason = "every trajectory point must contain six positions";
        return false;
      }
      if (point.time_from_start < previous_time) {
        reason = "trajectory point times must be monotonic";
        return false;
      }
      previous_time = point.time_from_start;
    }
    return true;
  }

  std::array<double, 6> interpolate(const std::array<double, 6>& start,
                                    const trajectory_msgs::JointTrajectoryPoint& end,
                                    double ratio) const {
    std::array<double, 6> values{};
    const double clamped_ratio = std::max(0.0, std::min(1.0, ratio));
    for (std::size_t index = 0; index < values.size(); ++index) {
      values[index] = start[index] +
          (end.positions[index] - start[index]) * clamped_ratio;
    }
    return values;
  }

  bool setGazeboConfiguration(const std::array<double, 6>& positions) {
    gazebo_msgs::SetModelConfiguration request;
    request.request.model_name = model_name_;
    request.request.urdf_param_name = urdf_param_name_;
    for (std::size_t index = 0; index < kArmJoints.size(); ++index) {
      request.request.joint_names.emplace_back(kArmJoints[index]);
      request.request.joint_positions.emplace_back(positions[index]);
    }
    if (!configuration_client_.call(request)) {
      ROS_ERROR("Unable to call /gazebo/set_model_configuration");
      return false;
    }
    if (!request.response.success) {
      ROS_ERROR("Gazebo rejected trajectory point: %s",
                request.response.status_message.c_str());
      return false;
    }
    return true;
  }

  void publishState(const std::array<double, 6>& positions) {
    const ros::Time stamp = ros::Time::now();
    // Gazebo starts paused. Do not publish zero-time or duplicate-time
    // samples: MoveIt correctly rejects them as stale joint states.
    if (stamp.isZero() || (!last_state_stamp_.isZero() && stamp <= last_state_stamp_)) {
      return;
    }
    sensor_msgs::JointState state;
    state.header.stamp = stamp;
    state.name.reserve(kArmJoints.size());
    state.position.reserve(kArmJoints.size());
    state.velocity.assign(kArmJoints.size(), 0.0);
    for (std::size_t index = 0; index < kArmJoints.size(); ++index) {
      state.name.emplace_back(kArmJoints[index]);
      state.position.emplace_back(positions[index]);
    }
    if (publish_joint_states_) {
      joint_state_publisher_.publish(state);
    }
    last_state_stamp_ = stamp;
  }

  void publishCurrentState(const ros::WallTimerEvent&) {
    // Keep a fresh, complete joint state available even while no trajectory is
    // executing. MoveIt's CurrentStateMonitor rejects stale/zero-time samples.
    publishState(current_positions_);
  }

  void execute(const GoalConstPtr& goal) {
    control_msgs::FollowJointTrajectoryResult result;
    std::string reason;
    if (!validGoal(goal, reason)) {
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      result.error_string = reason;
      action_server_.setAborted(result, reason);
      return;
    }

    const auto& points = goal->trajectory.points;
    const std::array<double, 6> start_positions = current_positions_;
    std::size_t segment = 0;
    ros::WallTime started_at = ros::WallTime::now();
    ros::WallRate rate(1.0 / kControlPeriodSeconds);
    const double duration = points.back().time_from_start.toSec();

    while (ros::ok()) {
      if (action_server_.isPreemptRequested()) {
        result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        action_server_.setPreempted(result, "trajectory preempted");
        return;
      }

      const double elapsed = (ros::WallTime::now() - started_at).toSec();
      while (segment + 1 < points.size() &&
             elapsed > points[segment + 1].time_from_start.toSec()) {
        ++segment;
      }

      std::array<double, 6> from = start_positions;
      double segment_start_time = 0.0;
      if (segment > 0) {
        for (std::size_t index = 0; index < from.size(); ++index) {
          from[index] = points[segment - 1].positions[index];
        }
        segment_start_time = points[segment - 1].time_from_start.toSec();
      }
      const double segment_end_time = points[segment].time_from_start.toSec();
      const double segment_duration = segment_end_time - segment_start_time;
      const double ratio = segment_duration > 0.0
          ? (elapsed - segment_start_time) / segment_duration : 1.0;
      const auto positions = interpolate(from, points[segment], ratio);

      if (!setGazeboConfiguration(positions)) {
        result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
        result.error_string = "Gazebo rejected a trajectory point";
        action_server_.setAborted(result, result.error_string);
        return;
      }
      current_positions_ = positions;
      publishState(positions);

      control_msgs::FollowJointTrajectoryFeedback feedback;
      feedback.header.stamp = ros::Time::now();
      feedback.joint_names = goal->trajectory.joint_names;
      feedback.desired.positions.assign(positions.begin(), positions.end());
      feedback.actual.positions.assign(positions.begin(), positions.end());
      feedback.error.positions.assign(kArmJoints.size(), 0.0);
      action_server_.publishFeedback(feedback);

      if (elapsed >= duration) {
        result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        action_server_.setSucceeded(result);
        return;
      }
      rate.sleep();
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  actionlib::SimpleActionServer<Action> action_server_;
  ros::ServiceClient configuration_client_;
  ros::Publisher joint_state_publisher_;
  ros::WallTimer state_timer_;
  std::array<double, 6> current_positions_{};
  std::string model_name_;
  std::string urdf_param_name_;
  bool publish_joint_states_{true};
  ros::Time last_state_stamp_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_follow_joint_trajectory");
  GazeboFollowJointTrajectory controller;
  ros::spin();
  return 0;
}
