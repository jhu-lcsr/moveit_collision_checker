
#include <vector>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/algorithm/string.hpp>

#include <moveit_collision_checker/moveit_collision_checker.h>

using namespace boost::python;
using namespace moveit_collision_checker;

template<typename T>
static void python_to_vector(boost::python::object o, std::vector<T> &v) {
    stl_input_iterator<T> begin(o);
    stl_input_iterator<T> end;
    v.clear();
    v.insert(v.end(), begin, end);
}

template<class T>
static list vector_to_list(const std::vector<T>& v)
{
    object get_iter = iterator<std::vector<T> >();
    object iter = get_iter(v);
    list l(iter);
    return l;
}

static int argc;
static char **argv;
static bool keep_running;
static boost::shared_ptr<boost::thread> spin_thread;

static void spin() {
  ros::NodeHandle nh;
  ros::Rate r(50);
  while(keep_running) {
    ros::spinOnce();
    r.sleep();
  }
}

static void init_roscpp() {
  keep_running = true;

  if(not ros::isInitialized()) {
    object sys_module = import("sys");
    list py_argv = extract<list>(sys_module.attr("argv"));

    argc = 0;
    argv = new char*[argc];

    for(int i=0; i<len(py_argv); i++) {
      std::string arg = extract<std::string>(py_argv[i]);
      if(arg.compare(0, 8, "__name:=") != 0) {
        argv[argc] = extract<char*>(py_argv[i]);
        argc++;
      }
    }

    object rospy_module = import("rospy");
    object get_name = rospy_module.attr("get_name");
    std::string name = extract<std::string>(get_name());
    std::vector<std::string> strs;
    boost::split(strs, name, boost::is_any_of("/"));

    std::cerr<<"Initializing roscpp with ROS name: \""<<strs.back()+"_cpp"<<"\""<<std::endl;

    ros::init(argc, argv, strs.back()+"_cpp",
              ros::init_options::NoSigintHandler);

    //spin_thread = boost::make_shared<boost::thread>(spin);

    std::cerr<<"Initialized."<<std::endl;
  }
}

static void shutdown_roscpp() {
  keep_running = false;
  //spin_thread->join();
  ros::shutdown();
  delete[] argv;
  argc = 0;
}

class CheckerWrapper : public Checker {
public:
  CheckerWrapper(
    std::string robot_description_param,
    std::string root_link,
    std::string planning_scene_topic) : Checker(robot_description_param, root_link, planning_scene_topic) { }

  CheckerWrapper(const CheckerWrapper &other) : Checker(other) { }

  list get_joint_names() {
    std::vector<std::string> joint_names;
    getJointNames(joint_names);
    list names;
    for(auto joint_name : joint_names) {
      names.append(joint_name);
    }
    return names;
  }

  bool check_state(object position_obj, object orientation_obj, object joint_positions_obj, object ignore_obj) {
    std::vector<double> position, orientation, joint_positions;
    std::vector<std::string> ignore;
    python_to_vector(position_obj, position);
    python_to_vector(orientation_obj, orientation);
    python_to_vector(joint_positions_obj, joint_positions);
    python_to_vector(ignore_obj, ignore);
    return checkState(position, orientation, joint_positions, ignore);
  }
};

BOOST_PYTHON_MODULE(moveit_collision_checker_py)
{
  def("init_roscpp", init_roscpp);
  def("shutdown_roscpp", shutdown_roscpp);
  def("spin_once", ros::spinOnce);
  class_<CheckerWrapper>("Checker", init<std::string, std::string, std::string>())
    .def("check_state", &CheckerWrapper::check_state)
    .def("get_joint_names", &CheckerWrapper::get_joint_names)
    .def("print_objects", &CheckerWrapper::printObjects)
    ;
};
