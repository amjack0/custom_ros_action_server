#include "ros/ros.h"
#include <iostream>
#include "actionlib/server/simple_action_server.h"
#include "ros_action_server/MyMsgAction.h"

#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "sensor_msgs/JointState.h"
#include "math.h"

using namespace std;

class MoveRobotAction{

protected:
  ros::NodeHandle nh;
  actionlib::SimpleActionServer<ros_action_server::MyMsgAction> action_server ; //TODO
  std::string action_name; // name of my action server
  ros_action_server::MyMsgFeedback feedback; // variables stores the feedback
  ros_action_server::MyMsgResult result;    // variables stores the result

  //defining my subscribers

  ros::Subscriber pos_sub;
  sensor_msgs::JointState pos_info;

  //defining my publishers

  ros::Publisher tau_pub; // TODO my be tau_multiple_pubs are required

public:
  //constructor
  MoveRobotAction(std::string name) :
    action_server(nh, name, boost::bind(&MoveRobotAction::actionCb, this, _1),false),
    action_name(name)
  {
   initializeSubscribers();
   initializePublishers();
   action_server.start();
  }

  ~MoveRobotAction(void){}

private:

  // initialize subscriber

  void initializeSubscribers(void)
  {
    pos_sub = nh.subscribe("/urbot/joint_states", 1, &MoveRobotAction::subscriberCb, this); //TODO
    ROS_INFO("[AS] Subscriber Initialized");
  } //66

  // initialize publisher

  void initializePublishers(void)
  {
    tau_pub = nh.advertise<std_msgs::Float64>("/urbot/endeffector_frc_trq_controller_2/command", 1);
    ROS_INFO("[AS] Publisher Initialized");
  }

  void subscriberCb(const sensor_msgs::JointStateConstPtr &info)
  {
    pos_info.position.push_back(info->position[1]);
    pos_info.effort.push_back(info->effort[1]);
    ROS_INFO("[AS] Reading joint_states");
    cout << "effort: " <<  pos_info.effort[1] << endl;
  }


  // function to calculate the tau error and decides the feedback

  double calError(sensor_msgs::JointState current, const ros_action_server::MyMsgGoalConstPtr &goal)
  {
    double error;
    error =  abs( goal->tau - current.effort[1] );
    return error;
  }



  //  main action server callback

  void actionCb(const ros_action_server::MyMsgGoalConstPtr &goal)
  {
    ros::Rate rate(50);
    bool success = true;
    // do your stuff here , I want to move a robot
    std_msgs::Float64 desired_tau;
    desired_tau.data = goal->tau;

    do
    {
      tau_pub.publish(desired_tau);
      feedback.error = calError(pos_info, goal);
      action_server.publishFeedback(feedback); //121
      // take care of preemption here
      if(action_server.isPreemptRequested() || !ros::ok() )
      {
        desired_tau.data = 0;    // robot will fall down !
        tau_pub.publish(desired_tau);
        ROS_INFO("%s: Preempted", action_name.c_str());
        action_server.setPreempted();
        success = false; //38.23
        break;
      }
      rate.sleep(); //38.40
    }

    while (feedback.error > 5);   // 39,33

    // check if succeeded--yes-->return result
    if(success)
    {
      result.status = "FINISHED";
      ROS_INFO("%s: Succeeded", action_name.c_str());
      //set the action state to succeeded
      action_server.setSucceeded(result); //41:00
    }
  } //42,27
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");
  MoveRobotAction robot("awesome_action");
  ros::Rate rate(0.5);
  int n=0;
  while (ros::ok())
  {
    ros::spinOnce();
    ROS_INFO("%d" ,n);
    n++;
    rate.sleep();
  }
  return 0;
}
