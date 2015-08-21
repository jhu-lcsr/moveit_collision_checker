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

    bool checkState(
        std::vector<double> position,
        std::vector<double> orientation,
        std::vector<double> joint_positions);

  private:
  
    boost::shared_ptr<tf::TransformListener> tf_listener_;
    boost::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    planning_scene_monitor::PlanningSceneMonitor monitor_;
    moveit::core::RobotState robot_state_;
    std::string root_link_;
  };

}

#endif // ifndef __MOVEIT_COLLISION_CHECKER_H
