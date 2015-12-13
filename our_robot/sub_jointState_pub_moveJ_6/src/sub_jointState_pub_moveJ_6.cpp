#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <moveit/move_group_interface/move_group.h>


std_msgs::Float32MultiArray joints;

ros::Publisher *ptr_pub;
ros::Rate *ptr_loop_rate;

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_INFO("%f,%f,%f,%f,%f,%f",msg->position[0],msg->position[1],msg->position[2],msg->position[3],
            msg->position[4],msg->position[5]);


      for(std::size_t i=0; i < 6; ++i)
      {
        joints.data[i] = msg->position[i];
      }
      ptr_pub->publish(joints);
   
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "sub_jointState_pub_moveJ"); 
  ros::NodeHandle n;

  ros::Publisher command_pub = n.advertise<std_msgs::Float32MultiArray> ("moveJ", 1000);
  ptr_pub = &command_pub;
  joints.data.resize(6);

 
  ros::Subscriber sub = n.subscribe("joint_states", 1000, chatterCallback);

  ros::spin();

  return 0;
}
