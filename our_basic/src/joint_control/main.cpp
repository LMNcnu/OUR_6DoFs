#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <config.h>
#include <jointcontrolcanbus.h>

JointControlCanBus *jointContorlHandler = new JointControlCanBus();

void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
  ROS_INFO("[%f,%f,%f,%f,%f,%f]",
           msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);

  bool isSent = jointContorlHandler->setTagPosRadio(0x05, msg->data[5], MODEL_TYPE_J60, CMDTYPE_WR_NR);
  ROS_INFO("Sent successfully? %d", isSent);

//    for(int i=0; i < ARM_DOF/2; ++i)
//    {
//        jointContorlHandler->setTagPosRadio(i+1, msg->data[i], MODEL_TYPE_J80, CMDTYPE_WR_NR);
//    }

//    for(int i=ARM_DOF/2; i < ARM_DOF; ++i)
//    {
//        jointContorlHandler->setTagPosRadio(i+1, msg->data[i], MODEL_TYPE_J60, CMDTYPE_WR_NR);
//    }
}

int main(int argc, char **argv)
{
  jointContorlHandler->JointControlInit(DEFAULT_NODE);
  
  ros::init(argc, argv, "joint_control");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("moveJ", 1000, chatterCallback);

  ros::spin();

  return 0;
}
