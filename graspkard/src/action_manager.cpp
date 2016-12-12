#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/MoveRobotAction.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <yaml-cpp/yaml.h>
#include <giskard/giskard.hpp>
//#include <kdl_conversions/kdl_msg.h>
#include <boost/lexical_cast.hpp>

#include <eigen3/Eigen/Eigen>
#include <r_libs/VisualizationManager.h>

#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "std_msgs/String.h"
#include <sstream>

#include <tf/transform_listener.h>


int nWSR_;

template<class T>
class ActionManager{
public:
  //virtual bool finished() = 0;
  //virtual void update() = 0;
  //virtual T get_status() = 0;
  virtual T get_current_value() = 0;
  virtual T get_alteration_rate() = 0;
};

void print_eigen(const Eigen::VectorXd& command)
{
  std::string cmd_str = " ";
  for(size_t i=0; i<command.rows(); ++i)
    cmd_str += boost::lexical_cast<std::string>(command[i]) + " ";
  ROS_INFO("Command: (%s)", cmd_str.c_str());
}

class MoveAction{//:public ActionManager<float>{
private:
  float last_feedback;
  ros::NodeHandle* nh_;
  std::string movement_controller;
  std::vector<std::string> joint_names;
  std::string frame_id;

  giskard::QPController controller_;
  std::vector<ros::Publisher> vel_controllers_;
  Eigen::VectorXd state_;
  bool controller_started_;
  //geometry_msgs::PointStamped goal_definition;
  ros::Subscriber js_sub_;

  ros::Publisher visPub;
  VisualizationManager visMan;
public:
  void set_goal(const geometry_msgs::PointStamped&);
  void js_callback(const sensor_msgs::JointState::ConstPtr&);

  //geometry_msgs::PointStamped::ConstPtr&
  MoveAction(std::string controller_description_, std::vector<std::string> joint_names_, std::vector<std::string> goal_def_str_, ros::NodeHandle* nh_):
    movement_controller(controller_description_),
    joint_names(joint_names_),
    nh_(nh_)
  {
    ROS_INFO("+++++++++++++++++++++++++++START");
    /*this->movement_controller = movement_controller;
    this->joint_names = joint_names;
    this->nh_ = nh_;*/
    this->last_feedback = 0;
    this->frame_id = "base_link";
    geometry_msgs::PointStamped goal_definition;
    if (goal_def_str_.size()!=7)ROS_ERROR("Incorrect size of paramsargument in goal definition for  MoveAction!!");
    goal_definition.header.seq = ::atoi(goal_def_str_[0].c_str());
    //ps_msg.header.stamp.secs = ::atof(goal_def_str_[1].c_str());
    //ps_msg.header.stamp.nsecs = ::atof(goal_def_str_[2].c_str());
    goal_definition.header.frame_id = goal_def_str_[3]; 
    goal_definition.point.x = ::atof(goal_def_str_[4].c_str());
    goal_definition.point.y = ::atof(goal_def_str_[5].c_str());
    goal_definition.point.z = ::atof(goal_def_str_[6].c_str());
    YAML::Node node = YAML::Load(this->movement_controller);
    giskard::QPControllerSpec spec = node.as< giskard::QPControllerSpec >();
    this->controller_ = giskard::generate(spec);
    this->state_ = Eigen::VectorXd::Zero(this->joint_names.size() + 2*6);
    this->controller_started_ = false;
    //meldet alle joints an
    for (std::vector<std::string>::iterator it = this->joint_names.begin(); it != this->joint_names.end(); ++it)
    {
      vel_controllers_.push_back(this->nh_->advertise<std_msgs::Float64>("/" + it->substr(0, it->size() - 6) + "_velocity_controller/command", 1));
    }
    this->set_goal(goal_definition);
    js_sub_ = this->nh_->subscribe("joint_states", 0, &MoveAction::js_callback, this);
    // Initialize visualization publisher
    visPub  = this->nh_->advertise<visualization_msgs::MarkerArray>("/graspkard_visualization", 1);
    visMan.addNamespace(0, "cylinder");
    ROS_INFO("+++++++++++++++++++++++++++END");
  }

  float get_current_value(){
    if (!this->controller_started_)
    {
      return 0;
    }
    else
    {
      return 0;
    }
  }

  float get_alteration_rate(){
    if (!this->controller_started_)
    {
      return 0;
    }
    else
    {
      return 0;
    }
  }

  void update(){
    if (!this->controller_started_){
      return;
    }
    ROS_INFO("WEEEEEEEEEEEEEEEEEEEEEE5");
    ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
    if (this->controller_.update(this->state_, nWSR_))
    {
    ROS_INFO("WEEEEEEEEEEEEEEEEEEEEEE IF");
      Eigen::VectorXd commands = this->controller_.get_command();
      for (unsigned int i=0; i < this->vel_controllers_.size(); i++)
      {
        std_msgs::Float64 command;
        command.data = commands[i];
        this->vel_controllers_[i].publish(command);
      }
      //std::cout << "published new joint states" << std::endl;
    }
    else
    {
    ROS_INFO("WEEEEEEEEEEEEEEEEEEEEEE ELSE");
      ROS_WARN("Update failed.");
      // TODO: remove or change to ros_debug
      std::cout << "State " << this->state_ << std::endl;
    }
  }
};



void MoveAction::js_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // TODO: turn this into a map!
  // is there a more efficient way?
  for (unsigned int i=0; i < this->joint_names.size(); i++)
  {
    for (unsigned int j=0; j < msg->name.size(); j++)
    {
      if (msg->name[j].compare(this->joint_names[i]) == 0)
      {
        this->state_[i] = msg->position[j];
      }
    }
  }

  if (!this->controller_started_)
    return;
////////////////
  if (this->controller_.update(this->state_, nWSR_))
  {
    Eigen::VectorXd commands = this->controller_.get_command();
    for (unsigned int i=0; i < this->vel_controllers_.size(); i++)
    {
      std_msgs::Float64 command;
      command.data = commands[i];
      this->vel_controllers_[i].publish(command);
    }
    //std::cout << "published new joint states" << std::endl;
  }
  else
  {
    ROS_WARN("Update failed.");
    // TODO: remove or change to ros_debug
    std::cout << "State " << this->state_ << std::endl;
  }
////////////
}

void MoveAction::set_goal(const geometry_msgs::PointStamped &msg)
{
  ROS_INFO("SSSSSEEEEEEEEEEEEEE START");
  if(msg.header.frame_id.compare(this->frame_id) != 0)
  {
    ROS_WARN("frame_id of right EE goal did not match expected '%s'. Ignoring goal", 
        this->frame_id.c_str());
    return;
  }

  this->visMan.beginNewDrawCycle();
  double cWidth = 0.06;
  double cHeight = 0.2;

  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(this->visMan.shapeMarker(0, 
                                  Affine3d::Identity(), 
                                  visualization_msgs::Marker::CYLINDER, 
                                  Vector3d(cWidth, cWidth, cHeight),
                                  0.f, 
                                  1.f, 
                                  0.f, 
                                  1.f, 
                                  "cylinder"));

  this->visMan.endDrawCycle(markers.markers);
  this->visPub.publish(markers);

  // copying position goal
  this->state_[joint_names.size() + 0] = msg.point.x;
  this->state_[joint_names.size() + 1] = msg.point.y;
  this->state_[joint_names.size() + 2] = msg.point.z;

  if (!this->controller_started_)
  {
    if (this->controller_.start(this->state_, nWSR_))
    {
      ROS_INFO("Controller started.");
      this->controller_started_ = true;
    }
    else
    {
      ROS_ERROR("Couldn't start controller.");
      print_eigen(this->state_);
    }
  }
}
void  testCB(const suturo_manipulation_msgs::MoveRobotGoalConstPtr &goal){
return;
}

//template<class ACTION, class FEEDBACK, class RESULT, class GOALCONSTPTR>
class ActionServer{

  private:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<suturo_manipulation_msgs::MoveRobotAction>* as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;

    // create messages that are used to published feedback/result
    suturo_manipulation_msgs::MoveRobotFeedback feedback_;
    suturo_manipulation_msgs::MoveRobotResult result_;
    MoveAction* ma;

  public:
    ActionServer(std::string name):
    action_name_(name), 
    nh_("~")
    {
      //action_name_ = "name";
      //nh_ = new ros::NodeHandle("~");
      nh_.param("nWSR", nWSR_, 10);
      as_ = new actionlib::SimpleActionServer<suturo_manipulation_msgs::MoveRobotAction>(nh_, action_name_, boost::bind(&ActionServer::executeCB, this, _1), false);
      as_->start();
      ROS_INFO("AS STARTED#########################");
    }    

    ~ActionServer(void)
    {
      delete[] ma;
    }

    void executeCB(const suturo_manipulation_msgs::MoveRobotGoalConstPtr &goal)
    {
      ROS_INFO("qwertzuiopztrertzuiop");
      if (!ma)
      {
        ma = new MoveAction(goal->controller_yaml, goal->controlled_joints, goal->params, &nh_);
      }

      // helper variables

      // get status of computation

      // publish info to the console for the user
      ROS_INFO("Started action: %s\n", action_name_.c_str());

      // start executing the action
      while (!(as_->isPreemptRequested() || !ros::ok())) //only the client can abort the action
      {
      //waitformsg /joint_states
      //dann update (siehe oben)
      // check that preempt has not been requested by the client

      //perform next computing step
      ma->update();
      feedback_.current_value = ma->get_current_value();
      feedback_.alteration_rate = ma->get_alteration_rate();
      // publish the feedback
      as_->publishFeedback(feedback_);
      }

      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_->setPreempted();
    }

};

int main(int argc, char **argv)
{
  std::string server_name("movement_server");
  ros::init(argc, argv, server_name);
  
  ROS_INFO("#############1");
  ActionServer movement_server(server_name);
  ROS_INFO("#############2");
  ros::spin();
  return 0;
}
