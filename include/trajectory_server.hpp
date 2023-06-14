#ifndef _TRAJECTORY_SERVER_H_
#define _TRAJECTORY_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <vector>

template <typename T>
std::vector<T> operator-(const std::vector<T>& lhs, const std::vector<T>& rhs) {
   std::vector<T> res;
   res.reserve(lhs.size());
   
   for (size_t idx = 0; idx < lhs.size(); idx++) {
      res.push_back(lhs[idx] - rhs[idx]);
   }
   
   return res;
}

class DynamixelTrajectoryAction {
   public:
   
      typedef std::map<int, int> JointMapType;
      
      DynamixelTrajectoryAction(const std::string& name);
      
      void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
      void onJointState(const sensor_msgs::JointState::ConstPtr& joints);
      
      static constexpr double max_position_error = 5;
      
   
   protected:
      ros::NodeHandle nh_;
      std::string action_name_;
      actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
      ros::Publisher jointPub;
      ros::Subscriber jointSub;
      
      control_msgs::FollowJointTrajectoryFeedback feedback_;
      control_msgs::FollowJointTrajectoryResult result_;
      sensor_msgs::JointState currentState;
      sensor_msgs::JointState targetState;
      std::vector<trajectory_msgs::JointTrajectoryPoint> points;
      int idx;
      
      std::vector<double> calculatePoint(ros::Duration curTime);
      
   private:
      
};

#endif // _TRAJECTORY_SERVER_H_
