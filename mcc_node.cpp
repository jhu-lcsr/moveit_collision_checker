
#include <ros/ros.h>
#include <moveit_collision_checker/moveit_collision_checker.h>
#include <moveit_collision_checker/CheckState.h>

std::shared_ptr<moveit_collision_checker::Checker> mcc;

bool check(moveit_collision_checker::CheckState::Request &req,
           moveit_collision_checker::CheckState::Response &res)
{
  res.collision_free = mcc->checkState(req.pose, req.joint_state, req.ignore);
  return true;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "mcc_node");

  std::string robot_description_param, root_link, scene_topic;

  ros::NodeHandle nh("~");
  nh.getParam("robot_description_param", robot_description_param);
  nh.getParam("root_link", root_link);
  nh.getParam("scene_topic", scene_topic);

  mcc.reset( new moveit_collision_checker::Checker(robot_description_param, root_link, scene_topic));
  ros::ServiceServer service = nh.advertiseService("check_state", check);

  ros::spin();

  return 0;
};

