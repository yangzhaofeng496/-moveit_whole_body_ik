#include <gazebo_msgs/GetModelState.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_group_ready_unpause_gazebo");
  ros::NodeHandle private_nh("~");

  std::string unpause_service("/gazebo/unpause_physics");
  std::string model_state_service("/gazebo/get_model_state");
  std::string model_name("robot");
  private_nh.param("unpause_service", unpause_service, unpause_service);
  private_nh.param("model_state_service", model_state_service, model_state_service);
  private_nh.param("model_name", model_name, model_name);

  // /get_planning_scene is not guaranteed to be advertised by every
  // MoveGroup capability configuration. Gazebo itself provides the reliable
  // readiness condition: the robot model must exist. Use wall time while the
  // simulation is paused so this wait cannot deadlock on /clock.
  ros::ServiceClient model_state_client =
      private_nh.serviceClient<gazebo_msgs::GetModelState>(model_state_service, true);
  ROS_INFO("Waiting for Gazebo model '%s' before unpausing physics", model_name.c_str());
  while (ros::ok() && !model_state_client.exists()) {
    ros::WallDuration(0.1).sleep();
  }
  if (!ros::ok()) {
    return 0;
  }

  gazebo_msgs::GetModelState model_request;
  model_request.request.model_name = model_name;
  model_request.request.relative_entity_name = "world";
  ros::WallRate model_wait_rate(20.0);
  while (ros::ok()) {
    if (model_state_client.call(model_request) && model_request.response.success) {
      break;
    }
    model_wait_rate.sleep();
  }
  if (!ros::ok()) {
    return 0;
  }
  ROS_INFO("Gazebo model '%s' is available; unpausing physics", model_name.c_str());

  ros::ServiceClient unpause_client =
      private_nh.serviceClient<std_srvs::Empty>(unpause_service, true);
  while (ros::ok() && !unpause_client.exists()) {
    ros::WallDuration(0.1).sleep();
  }
  if (!ros::ok()) {
    return 0;
  }

  std_srvs::Empty request;
  if (!unpause_client.call(request)) {
    ROS_ERROR("Failed to unpause Gazebo through %s", unpause_service.c_str());
    return 1;
  }

  ROS_INFO("Gazebo model is ready; physics unpaused");
  return 0;
}
