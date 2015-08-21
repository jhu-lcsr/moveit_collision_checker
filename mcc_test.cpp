
#include <ros/ros.h>
#include <moveit_collision_checker/moveit_collision_checker.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "mcc_test");

  std::string robot_description_param, root_link, scene_topic;

  ros::NodeHandle nh("~");
  nh.getParam("robot_description_param", robot_description_param);
  nh.getParam("root_link", root_link);
  nh.getParam("scene_topic", scene_topic);

  moveit_collision_checker::Checker mcc(robot_description_param, root_link, scene_topic);

  std::vector<double> position{0.5, 0.5, 0.25};
  std::vector<double> orientation{0.0, 0.0, 0.0, 1.0};
  std::vector<double> joint_positions(8, 0.0);
  std::vector<std::string> ignore{"ground_plane.link"};

  ros::Rate r(100.0);
  while(ros::ok()) {
    ros::spinOnce();
    bool valid = mcc.checkState(position, orientation, joint_positions, ignore);
    std::cerr<<"Valid: "<<valid<<std::endl;
    r.sleep();
  }

  return 0;
};

