#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <suturo_manipulation_msgs/MoveRobotAction.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <streambuf>

using namespace std;

namespace YAML {
  template <>
  struct convert<suturo_manipulation_msgs::TypedParam> {
    static bool decode(const Node& node, suturo_manipulation_msgs::TypedParam& rhs) {
      rhs.name = node["name"].as<string>();
      rhs.isConst = node["const"].as<bool>();
      rhs.value = node["value"].as<string>();
      string type = node["type"].as<string>();
      if (type.compare("double") == 0) {
        rhs.type = suturo_manipulation_msgs::TypedParam::DOUBLE;
      } else if (type.compare("transform") == 0) {
        rhs.type = suturo_manipulation_msgs::TypedParam::TRANSFORM;
      } else if (type.compare("elapsedtime") == 0) {
        rhs.type = suturo_manipulation_msgs::TypedParam::ELAPSEDTIME;
      } else if (type.compare("visualize") == 0) {
        rhs.type = suturo_manipulation_msgs::TypedParam::VISUALIZE;
      } else {
        cerr << "Type '" << type << "' of node '" << rhs.name << "' is unknown!" << endl;
        return false;
      }
      return true;
    }
  };
}

// ARGUMENTS: Controller.yaml, Joints.yaml, Params.yaml, feedback
int main(int argc, char **argv)
{
  if (argc < 5) {
    cerr << "4 arguments needed: Controller.yaml, Joints.yaml, Params.yaml, feedback" << endl;
    return 0;
  }

  ros::init(argc, argv, "client_test");
  //ros::NodeHandle nh("~");
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<suturo_manipulation_msgs::MoveRobotAction> ac("/movement_server/movement_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  suturo_manipulation_msgs::MoveRobotGoal goal;
  
  std::ifstream t(argv[1]);

  if (!t.good()) {
    cerr << "Couldn't open controller file: " << argv[1] << endl;
    return 0;
  }

  t.seekg(0, std::ios::end);   
  goal.controller_yaml.reserve(t.tellg());
  t.seekg(0, std::ios::beg);

  goal.controller_yaml.assign((std::istreambuf_iterator<char>(t)),
                               std::istreambuf_iterator<char>());

  t.close();

  try {
    goal.controlled_joints = YAML::LoadFile(argv[2]).as<vector<string>>();
  } catch (const YAML::Exception& e) {
    cerr << "Parsing of joint names failed! File: " << argv[2] << endl;
  }

  try {
    goal.params = YAML::LoadFile(argv[3]).as<vector<suturo_manipulation_msgs::TypedParam>>();
  } catch (const YAML::Exception& e) {
    cerr << "Parsing of params failed! File: " << argv[3] << endl;
  }

  goal.feedbackValue = argv[4];

  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    //ac.cancelGoal();
    ROS_INFO("Action did not finish before the time out.");
  }

  //exit
  return 0;
}


/*
add_executable(client_test src/client_test.cpp)

add_dependencies(client_test 
  graspkard_msgs
  suturo_manipulation_msgs_generate_messages_cpp
  #${graspkard_EXPORTED_TARGETS}
)

target_link_libraries(client_test 
  ${catkin_LIBRARIES}
)*/