
#include <algorithm>
#include <ros/ros.h>
#include <moveit_collision_checker/moveit_collision_checker.h>
#include <urdf_parser/urdf_parser.h>

using namespace moveit_collision_checker;

Checker::Checker(
    std::string robot_description_param,
    std::string root_link,
    std::string planning_scene_topic) :
  tf_listener_(new tf::TransformListener(ros::Duration(15.0))),
  robot_model_loader_(new robot_model_loader::RobotModelLoader(robot_description_param)),
  monitor_(robot_model_loader_, tf_listener_, "moveit_collision_checker"),
  robot_state_(robot_model_loader_->getModel()),
  root_link_(root_link)
{
  monitor_.startSceneMonitor(planning_scene_topic);
  monitor_.startWorldGeometryMonitor();
}

Checker::Checker(
    const Checker& other) :
  tf_listener_(new tf::TransformListener(ros::Duration(15.0))),
  robot_model_loader_(new robot_model_loader::RobotModelLoader(*other.robot_model_loader_)),
  monitor_(robot_model_loader_, tf_listener_, "moveit_collision_checker"),
  robot_state_(robot_model_loader_->getModel()),
  root_link_(other.root_link_)
{

}

void Checker::printObjects()
{
  monitor_.getPlanningScene()->printKnownObjects(std::cerr);
}

void Checker::getJointNames(std::vector<std::string> &joint_names)
{
  joint_names = robot_state_.getVariableNames();
}

bool Checker::checkState(
    std::vector<double> position,
    std::vector<double> orientation,
    std::vector<double> joint_positions,
    std::vector<std::string> ignore)
{
  for(auto &name:ignore) {
    ROS_DEBUG_STREAM("Ignoring: "<<name);
  }

  // Set robot_state_ joints
  robot_state_.setVariablePositions(joint_positions);

  // Convert pose to Affine3d
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform
    .translate( Eigen::Vector3d( position[0], position[1], position[2]))
    .rotate( Eigen::Quaterniond( orientation[3], orientation[0], orientation[1], orientation[2]));

  ROS_DEBUG_STREAM("transform: "<<std::endl<<transform.matrix());

  // Update root pose of robot in planning scene
  // See: https://github.com/ros-planning/moveit_core/issues/129#issuecomment-30884799
  robot_state_.updateStateWithLinkAt(
      root_link_,
      transform,
      true /* backward */);

  // Update so the link transforms aren't dirty
  robot_state_.update();

  // Check if the state is valid
  bool state_valid = monitor_.getPlanningScene()->isStateValid(robot_state_);

  // If the state isn't valid, figure out why
  if(not state_valid) {
    collision_detection::CollisionResult::ContactMap contact_map;
    monitor_.getPlanningScene()->getCollidingPairs(contact_map, robot_state_);
    for(auto &contact_pair : contact_map) {
      // Determine if this collision can be ignored
      bool ignore_first = std::find(ignore.begin(), ignore.end(), contact_pair.first.first) != ignore.end();
      bool ignore_second = std::find(ignore.begin(), ignore.end(), contact_pair.first.second) != ignore.end();

      ROS_DEBUG_STREAM("colliding: "<<contact_pair.first.first<<" ("<<(ignore_first?"ignore":"collide")<<") X "<<contact_pair.first.second<<" ("<<(ignore_second?"ignore":"collide")<<")");

      // If neither is on the ignore list, return false
      if(not (ignore_first or ignore_second)) {
        return false;
      }
    }
  }

  return true;
}

