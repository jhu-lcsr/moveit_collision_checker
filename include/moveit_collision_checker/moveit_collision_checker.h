#ifndef __MOVEIT_COLLISION_CHECKER_H
#define __MOVEIT_COLLISION_CHECKER_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace moveit_collision_checker {

  class Checker {
  public:
    Checker(
        std::string srdf_str,
        std::string root_link,
        std::string planning_scene_topic);

    Checker(const Checker& other);

    void printObjects();

    void getJointNames(std::vector<std::string> &joint_names);

    bool checkState(
        std::vector<double> &position,
        std::vector<double> &orientation,
        std::vector<double> &joint_positions,
        std::vector<std::string> &ignore);

    bool checkState(
        geometry_msgs::Pose &pose,
        sensor_msgs::JointState &joint_state,
        std::vector<std::string> &ignore);

    bool checkState(
        Eigen::Affine3d &transform,
        std::vector<double> &joint_positions,
        std::vector<std::string> &ignore);

  protected:
    void init();

  private:
    std::string robot_description_param_;
    std::string root_link_;
    std::string planning_scene_topic_;

    boost::shared_ptr<tf::TransformListener> tf_listener_;
    boost::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
    moveit::core::RobotStatePtr robot_state_;
  };

}

#endif // ifndef __MOVEIT_COLLISION_CHECKER_H
