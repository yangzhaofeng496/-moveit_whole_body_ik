#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetModelState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
constexpr std::array<const char*, 6> kArmJoints = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}

class MoveItToGazeboSync {
 public:
  MoveItToGazeboSync() : private_nh_("~") {
    private_nh_.param("source_topic", source_topic_,
                      std::string("/move_group/fake_controller_joint_states"));
    private_nh_.param("model_name", model_name_, std::string("robot"));
    private_nh_.param("urdf_param_name", urdf_param_name_,
                      std::string("robot_description"));
    private_nh_.param("service_name", service_name_,
                      std::string("/gazebo/set_model_configuration"));
    private_nh_.param("base_height", base_height_, 0.2);

    model_query_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>(
        "/gazebo/get_model_state", true);
    ROS_INFO("Waiting for Gazebo model '%s' before starting MoveIt-to-Gazebo sync",
             model_name_.c_str());
    while (ros::ok() && !model_query_client_.exists()) {
      ros::WallDuration(0.1).sleep();
    }
    if (ros::ok()) {
      gazebo_msgs::GetModelState request;
      request.request.model_name = model_name_;
      request.request.relative_entity_name = "world";
      ros::WallRate wait_rate(20.0);
      while (ros::ok()) {
        if (model_query_client_.call(request) && request.response.success) {
          model_ready_ = true;
          break;
        }
        wait_rate.sleep();
      }
    }
    if (model_ready_) {
      ROS_INFO("Gazebo model '%s' is ready for MoveIt synchronization",
               model_name_.c_str());
    }

    service_client_ = nh_.serviceClient<gazebo_msgs::SetModelConfiguration>(
        service_name_, false);
    model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(
        "/gazebo/set_model_state", false);
    joint_state_subscriber_ = nh_.subscribe(
        source_topic_, 1, &MoveItToGazeboSync::onJointState, this);
    base_lock_timer_ = nh_.createWallTimer(
        ros::WallDuration(0.02), &MoveItToGazeboSync::lockBasePose, this);
  }

 private:
  void onJointState(const sensor_msgs::JointState::ConstPtr& message) {
    if (!model_ready_) {
      return;
    }
    if (message->name.size() != message->position.size()) {
      ROS_WARN_THROTTLE(5.0, "MoveIt joint-state message has mismatched names and positions");
      return;
    }

    std::unordered_map<std::string, double> positions;
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      positions[message->name[index]] = message->position[index];
    }

    gazebo_msgs::SetModelConfiguration request;
    request.request.model_name = model_name_;
    request.request.urdf_param_name = urdf_param_name_;
    for (const char* joint_name : kArmJoints) {
      const auto position = positions.find(joint_name);
      if (position == positions.end()) {
        ROS_WARN_THROTTLE(5.0, "MoveIt state is missing %s", joint_name);
        return;
      }
      request.request.joint_names.emplace_back(joint_name);
      request.request.joint_positions.emplace_back(position->second);
    }

    if (!service_client_.call(request)) {
      ROS_WARN_THROTTLE(5.0, "Unable to call %s", service_name_.c_str());
    } else if (!request.response.success) {
      ROS_WARN_THROTTLE(5.0, "Gazebo rejected MoveIt joint state: %s",
                        request.response.status_message.c_str());
    }
  }

  void lockBasePose(const ros::WallTimerEvent&) {
    if (!model_ready_) {
      return;
    }
    gazebo_msgs::SetModelState request;
    request.request.model_state.model_name = model_name_;
    request.request.model_state.reference_frame = "world";
    request.request.model_state.pose.position.z = base_height_;
    request.request.model_state.pose.orientation.w = 1.0;
    request.request.model_state.twist.linear.x = 0.0;
    request.request.model_state.twist.linear.y = 0.0;
    request.request.model_state.twist.linear.z = 0.0;
    request.request.model_state.twist.angular.x = 0.0;
    request.request.model_state.twist.angular.y = 0.0;
    request.request.model_state.twist.angular.z = 0.0;
    if (!model_state_client_.call(request)) {
      ROS_WARN_THROTTLE(5.0, "Unable to lock Gazebo model base pose");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber joint_state_subscriber_;
  ros::WallTimer base_lock_timer_;
  ros::ServiceClient service_client_;
  ros::ServiceClient model_query_client_;
  ros::ServiceClient model_state_client_;
  std::string source_topic_;
  std::string model_name_;
  std::string urdf_param_name_;
  std::string service_name_;
  double base_height_;
  bool model_ready_{false};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_to_gazebo_sync");
  MoveItToGazeboSync node;
  ros::spin();
  return 0;
}
