
#include <vector>
#include <ros/ros.h>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/algorithm/string.hpp>

#include <moveit_collision_checker/moveit_collision_checker.h>

using namespace boost::python;
using namespace moveit_collision_checker;

template<typename T>
void python_to_vector(boost::python::object o, std::vector<T> &v) {
    stl_input_iterator<T> begin(o);
    stl_input_iterator<T> end;
    v.clear();
    v.insert(v.end(), begin, end);
}

void init_roscpp() {
  object sys_module = import("sys");
  list py_argv = extract<list>(sys_module.attr("argv"));

  int argc = 0;
  char* argv[argc];

  for(int i=0; i<len(py_argv); i++) {
    std::string arg = extract<std::string>(py_argv[i]);
    if(arg.compare(0, 8, "__name:=") != 0) {
      argv[i] = extract<char*>(py_argv[i]);
      argc++;
    }
  }

  object rospy_module = import("rospy");
  object get_name = rospy_module.attr("get_name");
  std::string name = extract<std::string>(get_name());
  std::vector<std::string> strs;
  boost::split(strs, name, boost::is_any_of("/"));

  ros::init(argc, argv, strs.back()+"_cpp",
            ros::init_options::NoSigintHandler);
}

class CheckerWrapper : public Checker {
public:
  CheckerWrapper(
    std::string robot_description_param,
    std::string root_link,
    std::string planning_scene_topic) : Checker(robot_description_param, root_link, planning_scene_topic) { }

  CheckerWrapper(const CheckerWrapper &other) : Checker(other) { }

  bool check_state(object position_obj, object orientation_obj, object joint_positions_obj) {
    std::vector<double> position, orientation, joint_positions;
    python_to_vector(position_obj, position);
    python_to_vector(orientation_obj, orientation);
    python_to_vector(joint_positions_obj, joint_positions);
    return checkState(position, orientation, joint_positions);
  }

  void spin_once() {
    ros::spinOnce();
  }
};

BOOST_PYTHON_MODULE(moveit_collision_checker_py)
{
  def("init_roscpp", init_roscpp);
  class_<CheckerWrapper>("Checker", init<std::string, std::string, std::string>())
    .def("check_state", &CheckerWrapper::check_state)
    .def("spin_once", &CheckerWrapper::spin_once)
    ;
};
